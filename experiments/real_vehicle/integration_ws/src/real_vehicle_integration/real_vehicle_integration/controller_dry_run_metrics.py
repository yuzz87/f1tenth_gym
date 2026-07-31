"""ROS-independent metrics for real-sensor controller dry-run trials."""

from collections import Counter
import math
import statistics


DEFAULT_THRESHOLDS = {
    "minimum_duration_ratio": 0.95,
    "minimum_command_samples": 10,
    "minimum_safe_command_coverage": 0.95,
    "minimum_localized_rate": 0.99,
    "minimum_guard_healthy_rate": 1.0,
    "minimum_running_rate": 0.95,
    "maximum_controller_overrun_rate": 0.01,
    "maximum_controller_deadline_rate": 0.01,
    "maximum_scan_age_s": 0.50,
    "maximum_pose_age_s": 0.35,
    "maximum_control_age_s": 0.10,
    "maximum_match_score_m2": 0.03,
    "speed_min_mps": 0.0,
    "speed_max_mps": 0.30,
    "steer_min_rad": -0.3141592653589793,
    "steer_max_rad": 0.3141592653589793,
    "steer_rate_min_rad_s": -0.70,
    "steer_rate_max_rad_s": 0.70,
    "command_tolerance": 1e-6,
}

FAULT_REASON_BY_MODE = {
    "scan_timeout": "scan_timeout",
    "pose_timeout": "pose_timeout",
    "control_timeout": "command_timeout",
    "emergency_stop": "emergency_stop",
}

FAULT_RESPONSE_LIMIT_S = {
    "scan_timeout": 0.55,
    "pose_timeout": 0.35,
    "control_timeout": 0.25,
    "emergency_stop": 0.10,
}

RECOVERABLE_PREFLIGHT_FAULTS = frozenset({
    "scan_timeout",
    "pose_timeout",
    "command_timeout",
})


def is_recoverable_preflight_fault(state, fault_reason):
    """Return whether a latched startup fault may be reset automatically."""

    return (
        str(state).upper() == "FAULT"
        and str(fault_reason) in RECOVERABLE_PREFLIGHT_FAULTS
    )


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


def _boolean_value(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _rate(count, total):
    return float(count) / float(total) if total else 0.0


def _mean(values):
    return statistics.mean(values) if values else None


def _maximum(values):
    return max(values) if values else None


def _percentile(values, percentile):
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(percentile)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _command_violations(rows, thresholds):
    tolerance = float(thresholds["command_tolerance"])
    limits = (
        ("speed_mps", "speed_min_mps", "speed_max_mps"),
        ("steering_angle_rad", "steer_min_rad", "steer_max_rad"),
        (
            "steering_rate_rad_s",
            "steer_rate_min_rad_s",
            "steer_rate_max_rad_s",
        ),
    )
    violations = []
    for index, row in enumerate(rows):
        for field, lower_key, upper_key in limits:
            try:
                value = float(row[field])
            except (KeyError, TypeError, ValueError):
                violations.append(f"row {index} has invalid {field}")
                continue
            lower = float(thresholds[lower_key]) - tolerance
            upper = float(thresholds[upper_key]) + tolerance
            if not math.isfinite(value) or not lower <= value <= upper:
                violations.append(
                    f"row {index} {field}={value!r} is outside "
                    f"[{thresholds[lower_key]}, {thresholds[upper_key]}]"
                )
    return violations


def _is_neutral(row, tolerance=1e-6):
    try:
        return (
            abs(float(row["speed_mps"])) <= tolerance
            and abs(float(row["steering_angle_rad"])) <= tolerance
            and abs(float(row["steering_rate_rad_s"])) <= tolerance
        )
    except (KeyError, TypeError, ValueError):
        return False


def summarize_controller_dry_run(
    request_rows,
    safe_rows,
    controller_rows,
    safety_rows,
    localization_rows,
    guard_rows,
    requested_duration_s,
    actual_duration_s,
    controller_type,
    fault_mode="none",
    fault_trigger_elapsed_s=None,
    thresholds=None,
    interrupted=False,
):
    """Return pass/fail metrics for a baseline or fault-injection trial."""

    configured = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        configured.update(thresholds)
    controller_type = str(controller_type).strip().lower()
    fault_mode = str(fault_mode).strip().lower()
    duration_ratio = (
        float(actual_duration_s) / float(requested_duration_s)
        if float(requested_duration_s) > 0.0
        else 0.0
    )

    solve_times = _finite_values(controller_rows, "solve_time_ms")
    periods = _finite_values(controller_rows, "period_ms")
    controller_overruns = sum(
        _boolean_value(row.get("overrun", False))
        for row in controller_rows
    )
    controller_deadlines = sum(
        _boolean_value(row.get("deadline_exceeded", False))
        for row in controller_rows
    )
    optimizer_successes = sum(
        _boolean_value(row.get("optimizer_success", False))
        for row in controller_rows
    )
    localized_count = sum(
        str(row.get("state", "")) == "localized"
        for row in localization_rows
    )
    guard_healthy_count = sum(
        _boolean_value(row.get("hardware_path_absent", False))
        for row in guard_rows
    )
    running_count = sum(
        str(row.get("state", "")).upper() == "RUNNING"
        for row in safety_rows
    )
    safety_states = [
        str(row.get("state", "")).upper() for row in safety_rows
    ]
    safety_fault_reasons = [
        str(row.get("fault_reason", ""))
        for row in safety_rows
        if str(row.get("fault_reason", ""))
    ]
    safe_command_coverage = min(
        1.0,
        _rate(len(safe_rows), len(request_rows)),
    )
    request_violations = _command_violations(request_rows, configured)
    safe_violations = _command_violations(safe_rows, configured)

    summary = {
        "status": "passed",
        "failures": [],
        "controller_type": controller_type,
        "fault_mode": fault_mode,
        "requested_duration_s": float(requested_duration_s),
        "actual_duration_s": float(actual_duration_s),
        "duration_ratio": duration_ratio,
        "interrupted": bool(interrupted),
        "request_samples": len(request_rows),
        "safe_command_samples": len(safe_rows),
        "controller_status_samples": len(controller_rows),
        "safety_status_samples": len(safety_rows),
        "localization_status_samples": len(localization_rows),
        "guard_status_samples": len(guard_rows),
        "safe_command_coverage": safe_command_coverage,
        "request_constraint_violations": len(request_violations),
        "safe_constraint_violations": len(safe_violations),
        "constraint_violation_examples": (
            request_violations[:3] + safe_violations[:3]
        ),
        "controller_solve_time_ms_mean": _mean(solve_times),
        "controller_solve_time_ms_p95": _percentile(solve_times, 0.95),
        "controller_solve_time_ms_max": _maximum(solve_times),
        "controller_period_ms_mean": _mean(periods),
        "controller_overrun_count": controller_overruns,
        "controller_overrun_rate": _rate(
            controller_overruns,
            len(controller_rows),
        ),
        "controller_deadline_count": controller_deadlines,
        "controller_deadline_rate": _rate(
            controller_deadlines,
            len(controller_rows),
        ),
        "optimizer_success_rate": _rate(
            optimizer_successes,
            len(controller_rows),
        ),
        "localized_rate": _rate(
            localized_count,
            len(localization_rows),
        ),
        "guard_healthy_rate": _rate(
            guard_healthy_count,
            len(guard_rows),
        ),
        "safety_running_rate": _rate(running_count, len(safety_rows)),
        "safety_state_counts": dict(sorted(Counter(safety_states).items())),
        "safety_fault_reason_counts": dict(sorted(
            Counter(safety_fault_reasons).items()
        )),
        "localization_scan_age_s_max": _maximum(
            _finite_values(localization_rows, "scan_age_s")
        ),
        "localization_match_score_m2_max": _maximum(
            _finite_values(localization_rows, "match_score_m2")
        ),
        "safety_scan_age_s_max": _maximum(
            _finite_values(safety_rows, "scan_age_s")
        ),
        "safety_pose_age_s_max": _maximum(
            _finite_values(safety_rows, "pose_age_s")
        ),
        "safety_control_age_s_max": _maximum(
            _finite_values(safety_rows, "control_age_s")
        ),
        "request_speed_mps_mean": _mean(
            _finite_values(request_rows, "speed_mps")
        ),
        "request_speed_mps_max": _maximum(
            _finite_values(request_rows, "speed_mps")
        ),
        "request_abs_steering_rad_max": _maximum([
            abs(value)
            for value in _finite_values(
                request_rows,
                "steering_angle_rad",
            )
        ]),
        "hardware_path_absent": bool(guard_rows) and (
            guard_healthy_count == len(guard_rows)
        ),
        "fault_trigger_elapsed_s": fault_trigger_elapsed_s,
        "fault_response_s": None,
        "fault_observed_reason": "",
        "safe_non_neutral_after_fault": None,
        "thresholds": configured,
    }

    failures = summary["failures"]
    if interrupted:
        failures.append("trial was interrupted")
    if duration_ratio < configured["minimum_duration_ratio"]:
        failures.append(
            f"duration ratio {duration_ratio:.6f} is below "
            f"{configured['minimum_duration_ratio']:.6f}"
        )
    if len(request_rows) < configured["minimum_command_samples"]:
        failures.append(
            f"request samples {len(request_rows)} are below "
            f"{configured['minimum_command_samples']}"
        )
    if not controller_rows:
        failures.append("no controller status samples were received")
    if not safety_rows:
        failures.append("no safety status samples were received")
    if not localization_rows:
        failures.append("no localization status samples were received")
    if not guard_rows:
        failures.append("no hardware isolation guard samples were received")
    if request_violations:
        failures.append(
            f"controller request constraint violations: "
            f"{len(request_violations)}"
        )
    if safe_violations:
        failures.append(
            f"safe command constraint violations: {len(safe_violations)}"
        )
    if (
        safe_command_coverage
        < configured["minimum_safe_command_coverage"]
    ):
        failures.append(
            f"safe command coverage {safe_command_coverage:.6f} is below "
            f"{configured['minimum_safe_command_coverage']:.6f}"
        )
    if (
        summary["localized_rate"]
        < configured["minimum_localized_rate"]
    ):
        failures.append(
            f"localized rate {summary['localized_rate']:.6f} is below "
            f"{configured['minimum_localized_rate']:.6f}"
        )
    if (
        summary["guard_healthy_rate"]
        < configured["minimum_guard_healthy_rate"]
    ):
        failures.append(
            f"hardware guard healthy rate "
            f"{summary['guard_healthy_rate']:.6f} is below "
            f"{configured['minimum_guard_healthy_rate']:.6f}"
        )
    if (
        summary["controller_overrun_rate"]
        > configured["maximum_controller_overrun_rate"]
    ):
        failures.append(
            f"controller overrun rate "
            f"{summary['controller_overrun_rate']:.6f} exceeds "
            f"{configured['maximum_controller_overrun_rate']:.6f}"
        )
    if (
        summary["controller_deadline_rate"]
        > configured["maximum_controller_deadline_rate"]
    ):
        failures.append(
            f"controller deadline rate "
            f"{summary['controller_deadline_rate']:.6f} exceeds "
            f"{configured['maximum_controller_deadline_rate']:.6f}"
        )

    comparisons = (
        (
            "localization scan age",
            summary["localization_scan_age_s_max"],
            "maximum_scan_age_s",
            "s",
        ),
        (
            "localization match score",
            summary["localization_match_score_m2_max"],
            "maximum_match_score_m2",
            "m2",
        ),
    )
    for label, value, key, unit in comparisons:
        if value is not None and value > configured[key]:
            failures.append(
                f"{label} {value:.6f} {unit} exceeds "
                f"{configured[key]:.6f} {unit}"
            )

    if fault_mode == "none":
        baseline_age_checks = (
            (
                "safety scan age",
                summary["safety_scan_age_s_max"],
                "maximum_scan_age_s",
            ),
            (
                "safety pose age",
                summary["safety_pose_age_s_max"],
                "maximum_pose_age_s",
            ),
            (
                "safety control age",
                summary["safety_control_age_s_max"],
                "maximum_control_age_s",
            ),
        )
        for label, value, key in baseline_age_checks:
            if value is not None and value > configured[key]:
                failures.append(
                    f"{label} {value:.6f} s exceeds "
                    f"{configured[key]:.6f} s"
                )
        if (
            summary["safety_running_rate"]
            < configured["minimum_running_rate"]
        ):
            failures.append(
                f"safety running rate "
                f"{summary['safety_running_rate']:.6f} is below "
                f"{configured['minimum_running_rate']:.6f}"
            )
        unexpected_faults = [
            state for state in safety_states if state == "FAULT"
        ]
        if unexpected_faults:
            failures.append("safety entered FAULT during baseline trial")
    else:
        expected_reason = FAULT_REASON_BY_MODE.get(fault_mode)
        if expected_reason is None:
            failures.append(f"unsupported fault mode: {fault_mode}")
        elif fault_trigger_elapsed_s is None:
            failures.append("fault trigger timestamp is missing")
        else:
            fault_rows = [
                row for row in safety_rows
                if (
                    float(row.get("elapsed_s", -1.0))
                    >= float(fault_trigger_elapsed_s)
                    and str(row.get("state", "")).upper() == "FAULT"
                    and str(row.get("fault_reason", "")) == expected_reason
                )
            ]
            if not fault_rows:
                failures.append(
                    f"expected safety fault was not observed: "
                    f"{expected_reason}"
                )
            else:
                fault_elapsed = min(
                    float(row["elapsed_s"]) for row in fault_rows
                )
                response_s = fault_elapsed - float(
                    fault_trigger_elapsed_s
                )
                summary["fault_response_s"] = response_s
                summary["fault_observed_reason"] = expected_reason
                post_fault_commands = [
                    row for row in safe_rows
                    if float(row.get("elapsed_s", -1.0)) >= fault_elapsed
                ]
                non_neutral = sum(
                    not _is_neutral(
                        row,
                        configured["command_tolerance"],
                    )
                    for row in post_fault_commands
                )
                summary["safe_non_neutral_after_fault"] = non_neutral
                if response_s > FAULT_RESPONSE_LIMIT_S[fault_mode]:
                    failures.append(
                        f"fault response {response_s:.6f} s exceeds "
                        f"{FAULT_RESPONSE_LIMIT_S[fault_mode]:.6f} s"
                    )
                if non_neutral:
                    failures.append(
                        f"{non_neutral} non-neutral safe commands were "
                        "published after FAULT"
                    )

    summary["status"] = "failed" if failures else "passed"
    return summary


__all__ = [
    "DEFAULT_THRESHOLDS",
    "FAULT_REASON_BY_MODE",
    "FAULT_RESPONSE_LIMIT_S",
    "summarize_controller_dry_run",
]
