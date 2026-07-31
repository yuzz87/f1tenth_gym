"""ROS-independent metrics for a stationary localization trial."""

from collections import Counter
import math
import statistics


DEFAULT_THRESHOLDS = {
    "minimum_duration_ratio": 0.95,
    "minimum_localized_rate": 1.0,
    "minimum_valid_beams": 20,
    "maximum_x_span_m": 0.15,
    "maximum_y_span_m": 0.15,
    "maximum_yaw_span_rad": 0.10,
    "maximum_match_score_m2": 0.02,
    "maximum_scan_age_s": 0.30,
    "maximum_processing_time_ms": 50.0,
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


def _span(values):
    return max(values) - min(values) if values else None


def _maximum(values):
    return max(values) if values else None


def _minimum(values):
    return min(values) if values else None


def _mean(values):
    return statistics.mean(values) if values else None


def _unwrapped_span(angles):
    if not angles:
        return None
    unwrapped = [float(angles[0])]
    for angle in angles[1:]:
        delta = (float(angle) - unwrapped[-1] + math.pi) % (
            2.0 * math.pi
        ) - math.pi
        unwrapped.append(unwrapped[-1] + delta)
    return _span(unwrapped)


def summarize_static_trial(
    status_rows,
    pose_rows,
    requested_duration_s,
    actual_duration_s,
    thresholds=None,
    interrupted=False,
):
    """Return pass/fail metrics for stationary localization samples."""

    configured = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        configured.update(thresholds)

    states = [str(row.get("state", "")) for row in status_rows]
    localized_count = sum(state == "localized" for state in states)
    localized_rate = (
        localized_count / len(states)
        if states
        else 0.0
    )
    xs = _finite_values(pose_rows, "x_m")
    ys = _finite_values(pose_rows, "y_m")
    yaws = _finite_values(pose_rows, "yaw_rad")
    match_scores = _finite_values(status_rows, "match_score_m2")
    valid_beams = _finite_values(status_rows, "valid_beams")
    scan_ages = _finite_values(status_rows, "scan_age_s")
    processing_times = _finite_values(status_rows, "processing_time_ms")
    position_corrections = _finite_values(
        status_rows,
        "position_correction_m",
    )
    yaw_corrections = _finite_values(status_rows, "yaw_correction_rad")
    duration_ratio = (
        float(actual_duration_s) / float(requested_duration_s)
        if float(requested_duration_s) > 0.0
        else 0.0
    )
    position_changes = sum(
        (
            abs(float(current["x_m"]) - float(previous["x_m"])) > 1e-9
            or abs(float(current["y_m"]) - float(previous["y_m"])) > 1e-9
        )
        for previous, current in zip(pose_rows, pose_rows[1:])
    )

    summary = {
        "status": "passed",
        "failures": [],
        "requested_duration_s": float(requested_duration_s),
        "actual_duration_s": float(actual_duration_s),
        "duration_ratio": duration_ratio,
        "interrupted": bool(interrupted),
        "status_samples": len(status_rows),
        "pose_samples": len(pose_rows),
        "localized_samples": localized_count,
        "localized_rate": localized_rate,
        "state_counts": dict(sorted(Counter(states).items())),
        "selection_reason_counts": dict(sorted(Counter(
            str(row.get("selection_reason", "unknown"))
            for row in status_rows
        ).items())),
        "x_span_m": _span(xs),
        "y_span_m": _span(ys),
        "yaw_span_rad": _unwrapped_span(yaws),
        "position_changes": position_changes,
        "first_pose": (
            {"x_m": xs[0], "y_m": ys[0], "yaw_rad": yaws[0]}
            if xs and ys and yaws
            else None
        ),
        "last_pose": (
            {"x_m": xs[-1], "y_m": ys[-1], "yaw_rad": yaws[-1]}
            if xs and ys and yaws
            else None
        ),
        "match_score_m2_mean": _mean(match_scores),
        "match_score_m2_max": _maximum(match_scores),
        "valid_beams_min": _minimum(valid_beams),
        "scan_age_s_max": _maximum(scan_ages),
        "processing_time_ms_mean": _mean(processing_times),
        "processing_time_ms_max": _maximum(processing_times),
        "position_correction_m_max": _maximum(position_corrections),
        "yaw_correction_rad_max": _maximum(yaw_corrections),
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
    if not status_rows:
        failures.append("no localization status samples were received")
    if not pose_rows:
        failures.append("no localization pose samples were received")
    if localized_rate < configured["minimum_localized_rate"]:
        failures.append(
            f"localized rate {localized_rate:.6f} is below "
            f"{configured['minimum_localized_rate']:.6f}"
        )

    comparisons = (
        ("x span", summary["x_span_m"], "maximum_x_span_m", "m"),
        ("y span", summary["y_span_m"], "maximum_y_span_m", "m"),
        ("yaw span", summary["yaw_span_rad"], "maximum_yaw_span_rad", "rad"),
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
    )
    for label, value, threshold_key, unit in comparisons:
        if value is not None and value > configured[threshold_key]:
            failures.append(
                f"{label} {value:.6f} {unit} exceeds "
                f"{configured[threshold_key]:.6f} {unit}"
            )
    if (
        summary["valid_beams_min"] is not None
        and summary["valid_beams_min"] < configured["minimum_valid_beams"]
    ):
        failures.append(
            f"minimum valid beams {summary['valid_beams_min']:.0f} is below "
            f"{configured['minimum_valid_beams']:.0f}"
        )
    summary["status"] = "failed" if failures else "passed"
    return summary


__all__ = ["DEFAULT_THRESHOLDS", "summarize_static_trial"]
