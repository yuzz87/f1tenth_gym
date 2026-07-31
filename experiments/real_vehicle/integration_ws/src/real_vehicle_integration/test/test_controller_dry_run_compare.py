from pathlib import Path

from real_vehicle_integration.controller_dry_run_compare import (
    compare_summaries,
)


def summary(controller, p95):
    return {
        "recording_format_version": 1,
        "controller_type": controller,
        "fault_mode": "none",
        "status": "passed",
        "actual_duration_s": 60.0,
        "controller_solve_time_ms_mean": p95 * 0.8,
        "controller_solve_time_ms_p95": p95,
        "controller_solve_time_ms_max": p95 * 1.2,
        "controller_overrun_rate": 0.0,
        "controller_deadline_rate": 0.0,
        "localized_rate": 1.0,
        "hardware_path_absent": True,
    }


def test_comparison_recommends_lower_p95_passing_controller():
    entries = [
        (Path("/tmp/mpc/summary.json"), summary("mpc", 8.0)),
        (Path("/tmp/mppi/summary.json"), summary("mppi", 6.0)),
    ]

    result = compare_summaries(entries)

    assert result["status"] == "passed"
    assert result["recommended_controller"] == "mppi"
    assert len(result["aggregate"]) == 2
