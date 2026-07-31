"""ROS-independent metrics for a manual localization motion trial."""

from collections import Counter
import math
import statistics


DEFAULT_THRESHOLDS = {
    "minimum_duration_ratio": 0.95,
    "minimum_localized_rate": 0.99,
    "minimum_valid_beams": 20,
    "maximum_expected_distance_error_m": 0.15,
    "maximum_odom_expected_distance_error_m": 0.15,
    "maximum_pose_odom_distance_difference_m": 0.15,
    "maximum_lateral_displacement_m": 0.15,
    "maximum_yaw_change_rad": 0.15,
    "maximum_final_x_span_m": 0.15,
    "maximum_final_y_span_m": 0.15,
    "maximum_final_yaw_span_rad": 0.10,
    "maximum_match_score_m2": 0.03,
    "maximum_scan_age_s": 0.30,
    "maximum_processing_time_ms": 50.0,
    "maximum_nonlocalized_duration_s": 0.30,
    "maximum_lidar_invalid_packet_delta": 0,
    "maximum_lidar_incomplete_scan_delta": 0,
    "maximum_lidar_sequence_gap_delta": 0,
    "maximum_lidar_session_changes": 0,
    "minimum_encoder_nonerror_rate": 1.0,
    "maximum_encoder_packet_age_s": 0.50,
    "maximum_encoder_invalid_packet_delta": 0,
    "maximum_encoder_sequence_gap_delta": 0,
    "maximum_encoder_session_change_delta": 0,
    "maximum_encoder_invalid_transition_delta": 0,
}


def _finite_values(rows, key):
    values = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def _phase_rows(rows, phase):
    return [row for row in rows if row.get("phase") == phase]


def _phase_tail(rows, phase, window_s):
    selected = _phase_rows(rows, phase)
    elapsed = _finite_values(selected, "elapsed_s")
    if not selected or not elapsed:
        return []
    start_s = max(elapsed) - max(0.0, float(window_s))
    return [
        row for row in selected
        if float(row.get("elapsed_s", -math.inf)) >= start_s
    ]


def _circular_mean(angles):
    if not angles:
        return None
    sin_mean = statistics.mean(math.sin(value) for value in angles)
    cos_mean = statistics.mean(math.cos(value) for value in angles)
    if abs(sin_mean) < 1e-12 and abs(cos_mean) < 1e-12:
        return statistics.median(angles)
    return math.atan2(sin_mean, cos_mean)


def _median_pose(rows):
    xs = _finite_values(rows, "x_m")
    ys = _finite_values(rows, "y_m")
    yaws = _finite_values(rows, "yaw_rad")
    if not xs or not ys or not yaws:
        return None
    return {
        "x_m": statistics.median(xs),
        "y_m": statistics.median(ys),
        "yaw_rad": _circular_mean(yaws),
        "samples": min(len(xs), len(ys), len(yaws)),
    }


def _normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _unwrapped_span(angles):
    if not angles:
        return None
    unwrapped = [float(angles[0])]
    for angle in angles[1:]:
        delta = _normalize_angle(float(angle) - unwrapped[-1])
        unwrapped.append(unwrapped[-1] + delta)
    return max(unwrapped) - min(unwrapped)


def _pose_delta(start_pose, end_pose):
    if start_pose is None or end_pose is None:
        return None
    dx_m = end_pose["x_m"] - start_pose["x_m"]
    dy_m = end_pose["y_m"] - start_pose["y_m"]
    yaw = start_pose["yaw_rad"]
    return {
        "dx_m": dx_m,
        "dy_m": dy_m,
        "distance_m": math.hypot(dx_m, dy_m),
        "forward_m": math.cos(yaw) * dx_m + math.sin(yaw) * dy_m,
        "lateral_m": -math.sin(yaw) * dx_m + math.cos(yaw) * dy_m,
        "yaw_change_rad": _normalize_angle(
            end_pose["yaw_rad"] - start_pose["yaw_rad"]
        ),
    }


def _maximum_nonlocalized_duration(status_rows, actual_duration_s):
    rows = sorted(
        status_rows,
        key=lambda row: float(row.get("elapsed_s", 0.0)),
    )
    active_start = None
    maximum = 0.0
    for row in rows:
        elapsed_s = float(row.get("elapsed_s", 0.0))
        if str(row.get("state", "")) != "localized":
            if active_start is None:
                active_start = elapsed_s
        elif active_start is not None:
            maximum = max(maximum, elapsed_s - active_start)
            active_start = None
    if active_start is not None:
        maximum = max(
            maximum,
            max(float(actual_duration_s), active_start) - active_start,
        )
    return maximum


def _counter_delta(rows, key):
    values = _finite_values(rows, key)
    if not values:
        return None
    return int(max(values) - min(values))


def _phase_counts(rows):
    return dict(sorted(Counter(
        str(row.get("phase", "unknown")) for row in rows
    ).items()))


def _append_maximum_failure(
    failures,
    label,
    value,
    configured,
    threshold_key,
    unit,
):
    if value is None:
        return
    threshold = float(configured[threshold_key])
    tolerance = 1e-9 * max(1.0, abs(threshold))
    if value > threshold + tolerance:
        failures.append(
            f"{label} {value:.6f} {unit} exceeds "
            f"{threshold:.6f} {unit}"
        )


def summarize_motion_trial(
    status_rows,
    pose_rows,
    odom_rows,
    lidar_rows,
    expected_distance_m,
    requested_duration_s,
    actual_duration_s,
    thresholds=None,
    interrupted=False,
    reference_window_s=2.0,
    encoder_rows=None,
):
    """Return pass/fail metrics for a manual straight-line motion trial."""

    configured = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        configured.update(thresholds)

    baseline_pose = _median_pose(
        _phase_tail(pose_rows, "baseline", reference_window_s)
    )
    final_pose = _median_pose(
        _phase_tail(pose_rows, "settle", reference_window_s)
    )
    baseline_odom = _median_pose(
        _phase_tail(odom_rows, "baseline", reference_window_s)
    )
    final_odom = _median_pose(
        _phase_tail(odom_rows, "settle", reference_window_s)
    )
    localizer_delta = _pose_delta(baseline_pose, final_pose)
    odom_delta = _pose_delta(baseline_odom, final_odom)

    final_pose_rows = _phase_tail(
        pose_rows,
        "settle",
        reference_window_s,
    )
    final_xs = _finite_values(final_pose_rows, "x_m")
    final_ys = _finite_values(final_pose_rows, "y_m")
    final_yaws = _finite_values(final_pose_rows, "yaw_rad")
    states = [str(row.get("state", "")) for row in status_rows]
    localized_count = sum(state == "localized" for state in states)
    localized_rate = (
        localized_count / len(states)
        if states
        else 0.0
    )
    match_scores = _finite_values(status_rows, "match_score_m2")
    valid_beams = _finite_values(status_rows, "valid_beams")
    scan_ages = _finite_values(status_rows, "scan_age_s")
    processing_times = _finite_values(status_rows, "processing_time_ms")
    duration_ratio = (
        float(actual_duration_s) / float(requested_duration_s)
        if float(requested_duration_s) > 0.0
        else 0.0
    )
    session_ids = {
        str(row.get("session_id", ""))
        for row in lidar_rows
        if str(row.get("session_id", ""))
    }
    encoder_rows = [] if encoder_rows is None else encoder_rows
    encoder_levels = _finite_values(encoder_rows, "level")
    encoder_nonerror_count = sum(
        level < 2 for level in encoder_levels
    )
    encoder_nonerror_rate = (
        encoder_nonerror_count / len(encoder_levels)
        if encoder_levels
        else 0.0
    )
    encoder_packet_ages = _finite_values(
        encoder_rows,
        "packet_age_s",
    )
    pose_odom_difference = None
    if localizer_delta is not None and odom_delta is not None:
        pose_odom_difference = abs(
            localizer_delta["distance_m"] - odom_delta["distance_m"]
        )

    summary = {
        "status": "passed",
        "failures": [],
        "expected_distance_m": float(expected_distance_m),
        "requested_duration_s": float(requested_duration_s),
        "actual_duration_s": float(actual_duration_s),
        "duration_ratio": duration_ratio,
        "reference_window_s": float(reference_window_s),
        "interrupted": bool(interrupted),
        "status_samples": len(status_rows),
        "pose_samples": len(pose_rows),
        "odom_samples": len(odom_rows),
        "lidar_status_samples": len(lidar_rows),
        "encoder_status_samples": len(encoder_rows),
        "status_samples_by_phase": _phase_counts(status_rows),
        "pose_samples_by_phase": _phase_counts(pose_rows),
        "odom_samples_by_phase": _phase_counts(odom_rows),
        "lidar_samples_by_phase": _phase_counts(lidar_rows),
        "encoder_samples_by_phase": _phase_counts(encoder_rows),
        "localized_samples": localized_count,
        "localized_rate": localized_rate,
        "state_counts": dict(sorted(Counter(states).items())),
        "selection_reason_counts": dict(sorted(Counter(
            str(row.get("selection_reason", "unknown"))
            for row in status_rows
        ).items())),
        "maximum_nonlocalized_duration_s": (
            _maximum_nonlocalized_duration(
                status_rows,
                actual_duration_s,
            )
        ),
        "baseline_pose": baseline_pose,
        "final_pose": final_pose,
        "baseline_odom": baseline_odom,
        "final_odom": final_odom,
        "localizer_delta": localizer_delta,
        "odom_delta": odom_delta,
        "localizer_expected_distance_error_m": (
            abs(localizer_delta["forward_m"] - expected_distance_m)
            if localizer_delta is not None
            else None
        ),
        "odom_expected_distance_error_m": (
            abs(odom_delta["forward_m"] - expected_distance_m)
            if odom_delta is not None
            else None
        ),
        "pose_odom_distance_difference_m": pose_odom_difference,
        "final_x_span_m": (
            max(final_xs) - min(final_xs)
            if final_xs
            else None
        ),
        "final_y_span_m": (
            max(final_ys) - min(final_ys)
            if final_ys
            else None
        ),
        "final_yaw_span_rad": _unwrapped_span(final_yaws),
        "match_score_m2_mean": (
            statistics.mean(match_scores)
            if match_scores
            else None
        ),
        "match_score_m2_max": (
            max(match_scores)
            if match_scores
            else None
        ),
        "valid_beams_min": min(valid_beams) if valid_beams else None,
        "scan_age_s_max": max(scan_ages) if scan_ages else None,
        "processing_time_ms_mean": (
            statistics.mean(processing_times)
            if processing_times
            else None
        ),
        "processing_time_ms_max": (
            max(processing_times)
            if processing_times
            else None
        ),
        "lidar_invalid_packet_delta": _counter_delta(
            lidar_rows,
            "invalid_packet_count",
        ),
        "lidar_incomplete_scan_delta": _counter_delta(
            lidar_rows,
            "incomplete_scans",
        ),
        "lidar_sequence_gap_delta": _counter_delta(
            lidar_rows,
            "sequence_gaps",
        ),
        "lidar_session_changes": max(0, len(session_ids) - 1),
        "encoder_nonerror_rate": encoder_nonerror_rate,
        "encoder_packet_age_s_max": (
            max(encoder_packet_ages)
            if encoder_packet_ages
            else None
        ),
        "encoder_invalid_packet_delta": _counter_delta(
            encoder_rows,
            "invalid_packet_count",
        ),
        "encoder_sequence_gap_delta": _counter_delta(
            encoder_rows,
            "sequence_gaps",
        ),
        "encoder_session_change_delta": _counter_delta(
            encoder_rows,
            "session_changes",
        ),
        "encoder_invalid_transition_delta": _counter_delta(
            encoder_rows,
            "invalid_transition_count",
        ),
        "thresholds": configured,
    }

    failures = summary["failures"]
    if interrupted:
        failures.append("trial was interrupted")
    if duration_ratio < configured["minimum_duration_ratio"]:
        failures.append(
            "recorded duration ratio "
            f"{duration_ratio:.3f} is below "
            f"{configured['minimum_duration_ratio']:.3f}"
        )
    required_rows = (
        ("localization status", status_rows),
        ("localization pose", pose_rows),
        ("odometry", odom_rows),
        ("LiDAR transport status", lidar_rows),
    )
    for label, rows in required_rows:
        if not rows:
            failures.append(f"no {label} samples were received")
    if not encoder_rows:
        failures.append("no encoder transport status samples were received")
    if baseline_pose is None or final_pose is None:
        failures.append(
            "baseline and settle localization pose samples are required"
        )
    if baseline_odom is None or final_odom is None:
        failures.append("baseline and settle odometry samples are required")
    if localized_rate < configured["minimum_localized_rate"]:
        failures.append(
            f"localized rate {localized_rate:.6f} is below "
            f"{configured['minimum_localized_rate']:.6f}"
        )
    if encoder_nonerror_rate < configured["minimum_encoder_nonerror_rate"]:
        failures.append(
            f"encoder non-error rate {encoder_nonerror_rate:.6f} is below "
            f"{configured['minimum_encoder_nonerror_rate']:.6f}"
        )

    comparisons = (
        (
            "localizer expected-distance error",
            summary["localizer_expected_distance_error_m"],
            "maximum_expected_distance_error_m",
            "m",
        ),
        (
            "odometry expected-distance error",
            summary["odom_expected_distance_error_m"],
            "maximum_odom_expected_distance_error_m",
            "m",
        ),
        (
            "localizer/odometry distance difference",
            summary["pose_odom_distance_difference_m"],
            "maximum_pose_odom_distance_difference_m",
            "m",
        ),
        (
            "final x span",
            summary["final_x_span_m"],
            "maximum_final_x_span_m",
            "m",
        ),
        (
            "final y span",
            summary["final_y_span_m"],
            "maximum_final_y_span_m",
            "m",
        ),
        (
            "final yaw span",
            summary["final_yaw_span_rad"],
            "maximum_final_yaw_span_rad",
            "rad",
        ),
        (
            "maximum match score",
            summary["match_score_m2_max"],
            "maximum_match_score_m2",
            "m2",
        ),
        (
            "maximum scan age",
            summary["scan_age_s_max"],
            "maximum_scan_age_s",
            "s",
        ),
        (
            "maximum processing time",
            summary["processing_time_ms_max"],
            "maximum_processing_time_ms",
            "ms",
        ),
        (
            "maximum nonlocalized duration",
            summary["maximum_nonlocalized_duration_s"],
            "maximum_nonlocalized_duration_s",
            "s",
        ),
        (
            "maximum encoder packet age",
            summary["encoder_packet_age_s_max"],
            "maximum_encoder_packet_age_s",
            "s",
        ),
    )
    for label, value, threshold_key, unit in comparisons:
        _append_maximum_failure(
            failures,
            label,
            value,
            configured,
            threshold_key,
            unit,
        )
    if localizer_delta is not None:
        _append_maximum_failure(
            failures,
            "localizer lateral displacement",
            abs(localizer_delta["lateral_m"]),
            configured,
            "maximum_lateral_displacement_m",
            "m",
        )
        _append_maximum_failure(
            failures,
            "localizer yaw change",
            abs(localizer_delta["yaw_change_rad"]),
            configured,
            "maximum_yaw_change_rad",
            "rad",
        )
    if (
        summary["valid_beams_min"] is not None
        and summary["valid_beams_min"] < configured["minimum_valid_beams"]
    ):
        failures.append(
            f"minimum valid beams {summary['valid_beams_min']:.0f} is below "
            f"{configured['minimum_valid_beams']:.0f}"
        )

    counter_comparisons = (
        (
            "LiDAR invalid packet delta",
            summary["lidar_invalid_packet_delta"],
            "maximum_lidar_invalid_packet_delta",
        ),
        (
            "LiDAR incomplete scan delta",
            summary["lidar_incomplete_scan_delta"],
            "maximum_lidar_incomplete_scan_delta",
        ),
        (
            "LiDAR sequence gap delta",
            summary["lidar_sequence_gap_delta"],
            "maximum_lidar_sequence_gap_delta",
        ),
        (
            "LiDAR session changes",
            summary["lidar_session_changes"],
            "maximum_lidar_session_changes",
        ),
        (
            "encoder invalid packet delta",
            summary["encoder_invalid_packet_delta"],
            "maximum_encoder_invalid_packet_delta",
        ),
        (
            "encoder sequence gap delta",
            summary["encoder_sequence_gap_delta"],
            "maximum_encoder_sequence_gap_delta",
        ),
        (
            "encoder session change delta",
            summary["encoder_session_change_delta"],
            "maximum_encoder_session_change_delta",
        ),
        (
            "encoder invalid transition delta",
            summary["encoder_invalid_transition_delta"],
            "maximum_encoder_invalid_transition_delta",
        ),
    )
    for label, value, threshold_key in counter_comparisons:
        if value is not None and value > configured[threshold_key]:
            failures.append(
                f"{label} {value} exceeds {configured[threshold_key]}"
            )

    summary["status"] = "failed" if failures else "passed"
    return summary


__all__ = ["DEFAULT_THRESHOLDS", "summarize_motion_trial"]
