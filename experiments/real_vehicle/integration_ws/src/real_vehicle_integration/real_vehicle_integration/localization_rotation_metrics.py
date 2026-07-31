"""ROS-independent metrics for a manual localization rotation trial."""

from collections import Counter
import math
import statistics


DEFAULT_THRESHOLDS = {
    "minimum_duration_ratio": 0.95,
    "minimum_localized_rate": 0.99,
    "minimum_valid_beams": 20,
    "maximum_expected_yaw_error_rad": math.radians(10.0),
    "maximum_position_drift_m": 0.20,
    "maximum_final_x_span_m": 0.15,
    "maximum_final_y_span_m": 0.15,
    "maximum_final_yaw_span_rad": 0.10,
    "maximum_match_score_m2": 0.03,
    "maximum_scan_age_s": 0.30,
    "maximum_processing_time_ms": 50.0,
    "maximum_nonlocalized_duration_s": 0.30,
    "minimum_correction_event_delta": 1,
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


def normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


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
        row
        for row in selected
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


def _pose_delta(start_pose, end_pose):
    if start_pose is None or end_pose is None:
        return None
    dx_m = end_pose["x_m"] - start_pose["x_m"]
    dy_m = end_pose["y_m"] - start_pose["y_m"]
    return {
        "dx_m": dx_m,
        "dy_m": dy_m,
        "distance_m": math.hypot(dx_m, dy_m),
        "yaw_change_rad": normalize_angle(
            end_pose["yaw_rad"] - start_pose["yaw_rad"]
        ),
    }


def _unwrapped_span(angles):
    if not angles:
        return None
    unwrapped = [float(angles[0])]
    for angle in angles[1:]:
        delta = normalize_angle(float(angle) - unwrapped[-1])
        unwrapped.append(unwrapped[-1] + delta)
    return max(unwrapped) - min(unwrapped)


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


def summarize_rotation_trial(
    status_rows,
    pose_rows,
    odom_rows,
    lidar_rows,
    expected_yaw_rad,
    requested_duration_s,
    actual_duration_s,
    thresholds=None,
    interrupted=False,
    reference_window_s=2.0,
    encoder_rows=None,
):
    """Return pass/fail metrics for a known-angle manual rotation trial."""

    configured = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        configured.update(thresholds)

    expected_yaw_rad = float(expected_yaw_rad)
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

    localizer_yaw_change = (
        localizer_delta["yaw_change_rad"]
        if localizer_delta is not None
        else None
    )
    expected_yaw_error = (
        abs(normalize_angle(localizer_yaw_change - expected_yaw_rad))
        if localizer_yaw_change is not None
        else None
    )
    direction_correct = (
        localizer_yaw_change * expected_yaw_rad > 0.0
        if localizer_yaw_change is not None
        else False
    )
    localizer_odom_yaw_difference = (
        abs(normalize_angle(
            localizer_delta["yaw_change_rad"]
            - odom_delta["yaw_change_rad"]
        ))
        if localizer_delta is not None and odom_delta is not None
        else None
    )

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
    correction_event_delta = _counter_delta(
        status_rows,
        "correction_event_id",
    )
    rotation_rows = _phase_rows(status_rows, "rotation")
    rotation_correction_samples = sum(
        str(row.get("selection_reason", "")) == "scan_correction"
        for row in rotation_rows
    )

    summary = {
        "status": "passed",
        "failures": [],
        "expected_yaw_rad": expected_yaw_rad,
        "expected_yaw_deg": math.degrees(expected_yaw_rad),
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
        "measured_yaw_change_rad": localizer_yaw_change,
        "measured_yaw_change_deg": (
            math.degrees(localizer_yaw_change)
            if localizer_yaw_change is not None
            else None
        ),
        "expected_yaw_error_rad": expected_yaw_error,
        "expected_yaw_error_deg": (
            math.degrees(expected_yaw_error)
            if expected_yaw_error is not None
            else None
        ),
        "rotation_direction_correct": direction_correct,
        "position_drift_m": (
            localizer_delta["distance_m"]
            if localizer_delta is not None
            else None
        ),
        "localizer_odom_yaw_difference_rad": (
            localizer_odom_yaw_difference
        ),
        "correction_event_delta": correction_event_delta,
        "rotation_scan_correction_samples": rotation_correction_samples,
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
    if localizer_yaw_change is not None and not direction_correct:
        failures.append(
            "localizer yaw changed in the opposite direction "
            f"({localizer_yaw_change:.6f} rad)"
        )
    if (
        configured["minimum_correction_event_delta"] > 0
        and (
            correction_event_delta is None
            or correction_event_delta
            < configured["minimum_correction_event_delta"]
        )
    ):
        failures.append(
            f"correction event delta {correction_event_delta} is below "
            f"{configured['minimum_correction_event_delta']}"
        )

    comparisons = (
        (
            "localizer expected-yaw error",
            summary["expected_yaw_error_rad"],
            "maximum_expected_yaw_error_rad",
            "rad",
        ),
        (
            "localizer position drift",
            summary["position_drift_m"],
            "maximum_position_drift_m",
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


__all__ = [
    "DEFAULT_THRESHOLDS",
    "normalize_angle",
    "summarize_rotation_trial",
]
