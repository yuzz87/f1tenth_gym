import math

import pytest

from real_vehicle_integration.localization_static_metrics import (
    summarize_static_trial,
)


def status_row(index, state="localized"):
    return {
        "stamp_s": index * 0.1,
        "state": state,
        "match_score_m2": 0.005,
        "valid_beams": 260,
        "scan_age_s": 0.08,
        "processing_time_ms": 15.0,
        "selection_reason": "prediction_best",
        "position_correction_m": 0.0,
        "yaw_correction_rad": 0.0,
    }


def pose_row(index, x_m=0.0, y_m=0.0, yaw_rad=0.0):
    return {
        "stamp_s": index * 0.1,
        "x_m": x_m,
        "y_m": y_m,
        "yaw_rad": yaw_rad,
    }


def test_stationary_summary_passes_for_stable_localization():
    statuses = [status_row(index) for index in range(100)]
    poses = [
        pose_row(index, x_m=0.01 * (index % 2), y_m=0.0)
        for index in range(100)
    ]

    summary = summarize_static_trial(
        statuses,
        poses,
        requested_duration_s=10.0,
        actual_duration_s=10.0,
    )

    assert summary["status"] == "passed"
    assert summary["localized_rate"] == 1.0
    assert summary["x_span_m"] == pytest.approx(0.01)
    assert summary["failures"] == []


def test_stationary_summary_rejects_position_oscillation():
    statuses = [status_row(index) for index in range(100)]
    poses = [
        pose_row(index, x_m=0.30 if index % 2 else 0.0)
        for index in range(100)
    ]

    summary = summarize_static_trial(
        statuses,
        poses,
        requested_duration_s=10.0,
        actual_duration_s=10.0,
    )

    assert summary["status"] == "failed"
    assert summary["x_span_m"] == pytest.approx(0.30)
    assert any("x span" in failure for failure in summary["failures"])


def test_yaw_span_handles_pi_wrap_without_false_failure():
    statuses = [status_row(index) for index in range(10)]
    poses = [
        pose_row(index, yaw_rad=math.pi - 0.01)
        if index % 2
        else pose_row(index, yaw_rad=-math.pi + 0.01)
        for index in range(10)
    ]

    summary = summarize_static_trial(
        statuses,
        poses,
        requested_duration_s=1.0,
        actual_duration_s=1.0,
    )

    assert summary["status"] == "passed"
    assert summary["yaw_span_rad"] == pytest.approx(0.02)


def test_missing_pose_samples_fails_safely():
    summary = summarize_static_trial(
        [status_row(index) for index in range(10)],
        [],
        requested_duration_s=1.0,
        actual_duration_s=1.0,
    )

    assert summary["status"] == "failed"
    assert "no localization pose samples were received" in summary["failures"]
