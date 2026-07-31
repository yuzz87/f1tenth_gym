import pytest

from real_vehicle_integration.controller_dry_run_metrics import (
    is_recoverable_preflight_fault,
    summarize_controller_dry_run,
)


def command_row(index, speed=0.05, steering=0.01, steering_rate=0.02):
    return {
        "elapsed_s": index * 0.01,
        "speed_mps": speed,
        "steering_angle_rad": steering,
        "steering_rate_rad_s": steering_rate,
    }


def controller_row(index, controller_type="mppi"):
    return {
        "elapsed_s": index * 0.01,
        "controller_type": controller_type,
        "solve_time_ms": 5.0,
        "period_ms": 10.0 if controller_type == "mppi" else 20.0,
        "overrun": "false",
        "deadline_exceeded": "false",
        "optimizer_success": "true",
    }


def safety_row(index, state="RUNNING", fault_reason=""):
    return {
        "elapsed_s": index * 0.01,
        "state": state,
        "fault_reason": fault_reason,
        "scan_age_s": 0.10,
        "pose_age_s": 0.10,
        "control_age_s": 0.01,
    }


def localization_row(index):
    return {
        "elapsed_s": index * 0.01,
        "state": "localized",
        "scan_age_s": 0.10,
        "match_score_m2": 0.01,
    }


def guard_row(index, healthy=True):
    return {
        "elapsed_s": index * 0.01,
        "hardware_path_absent": str(healthy).lower(),
    }


def baseline_inputs(samples=100):
    return (
        [command_row(index) for index in range(samples)],
        [command_row(index) for index in range(samples)],
        [controller_row(index) for index in range(samples)],
        [safety_row(index) for index in range(samples)],
        [localization_row(index) for index in range(samples)],
        [guard_row(index) for index in range(samples)],
    )


def test_baseline_summary_passes_for_isolated_healthy_commands():
    summary = summarize_controller_dry_run(
        *baseline_inputs(),
        requested_duration_s=1.0,
        actual_duration_s=1.0,
        controller_type="mppi",
    )

    assert summary["status"] == "passed"
    assert summary["hardware_path_absent"]
    assert summary["controller_solve_time_ms_p95"] == pytest.approx(5.0)


@pytest.mark.parametrize(
    "reason",
    ("scan_timeout", "pose_timeout", "command_timeout"),
)
def test_preflight_timeout_faults_are_recoverable(reason):
    assert is_recoverable_preflight_fault("FAULT", reason)


def test_preflight_does_not_reset_nonrecoverable_fault():
    assert not is_recoverable_preflight_fault("FAULT", "emergency_stop")
    assert not is_recoverable_preflight_fault(
        "RUNNING",
        "pose_timeout",
    )


def test_baseline_rejects_hardware_guard_violation():
    values = list(baseline_inputs())
    values[-1][50] = guard_row(50, healthy=False)

    summary = summarize_controller_dry_run(
        *values,
        requested_duration_s=1.0,
        actual_duration_s=1.0,
        controller_type="mppi",
    )

    assert summary["status"] == "failed"
    assert any("hardware guard" in failure for failure in summary["failures"])


def test_baseline_rejects_command_outside_vehicle_limits():
    values = list(baseline_inputs())
    values[0][20] = command_row(20, speed=0.5)

    summary = summarize_controller_dry_run(
        *values,
        requested_duration_s=1.0,
        actual_duration_s=1.0,
        controller_type="mppi",
    )

    assert summary["status"] == "failed"
    assert summary["request_constraint_violations"] == 1


def test_scan_fault_passes_when_safety_faults_and_stays_neutral():
    requests = [command_row(index) for index in range(1000)]
    safe = [command_row(index) for index in range(531)]
    safe.extend([
        command_row(
            index,
            speed=0.0,
            steering=0.0,
            steering_rate=0.0,
        )
        for index in range(531, 1000)
    ])
    controllers = [controller_row(index) for index in range(1000)]
    safety = [safety_row(index) for index in range(531)]
    safety.extend([
        safety_row(index, "FAULT", "scan_timeout")
        for index in range(531, 1000)
    ])
    localizations = [localization_row(index) for index in range(1000)]
    guards = [guard_row(index) for index in range(1000)]

    summary = summarize_controller_dry_run(
        requests,
        safe,
        controllers,
        safety,
        localizations,
        guards,
        requested_duration_s=10.0,
        actual_duration_s=10.0,
        controller_type="mppi",
        fault_mode="scan_timeout",
        fault_trigger_elapsed_s=5.0,
    )

    assert summary["status"] == "passed"
    assert summary["fault_response_s"] == pytest.approx(0.31)
    assert summary["safe_non_neutral_after_fault"] == 0


def test_fault_trial_fails_if_non_neutral_command_follows_fault():
    values = list(baseline_inputs())
    values[3] = [
        safety_row(index)
        if index < 51
        else safety_row(index, "FAULT", "emergency_stop")
        for index in range(100)
    ]

    summary = summarize_controller_dry_run(
        *values,
        requested_duration_s=1.0,
        actual_duration_s=1.0,
        controller_type="mppi",
        fault_mode="emergency_stop",
        fault_trigger_elapsed_s=0.5,
    )

    assert summary["status"] == "failed"
    assert summary["safe_non_neutral_after_fault"] > 0
