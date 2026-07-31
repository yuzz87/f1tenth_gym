import math

import pytest

from real_vehicle_integration.localization_rotation_metrics import (
    summarize_rotation_trial,
)


def _phase(index):
    if index < 50:
        return "baseline"
    if index < 200:
        return "rotation"
    return "settle"


def _rotation_fraction(index):
    if index < 50:
        return 0.0
    if index >= 200:
        return 1.0
    return (index - 50) / 150.0


def _normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _rows(
    expected_yaw_rad=math.radians(30.0),
    measured_yaw_rad=None,
    yaw_start=0.0,
    position_drift_m=0.02,
    correction_events=2,
):
    if measured_yaw_rad is None:
        measured_yaw_rad = expected_yaw_rad
    status_rows = []
    pose_rows = []
    odom_rows = []
    lidar_rows = []
    encoder_rows = []
    for index in range(250):
        elapsed_s = index * 0.1
        phase = _phase(index)
        fraction = _rotation_fraction(index)
        yaw = _normalize_angle(
            yaw_start + measured_yaw_rad * fraction
        )
        correction_event_id = min(
            correction_events,
            int(fraction * (correction_events + 1)),
        )
        selection_reason = (
            "scan_correction"
            if (
                correction_event_id > 0
                and correction_event_id
                != min(
                    correction_events,
                    int(
                        _rotation_fraction(max(0, index - 1))
                        * (correction_events + 1)
                    ),
                )
            )
            else "prediction_best"
        )
        pose_rows.append({
            "elapsed_s": elapsed_s,
            "phase": phase,
            "x_m": position_drift_m * fraction,
            "y_m": 0.0,
            "yaw_rad": yaw,
        })
        odom_rows.append({
            "elapsed_s": elapsed_s,
            "phase": phase,
            "x_m": 0.05 * fraction,
            "y_m": 0.0,
            "yaw_rad": yaw_start,
        })
        status_rows.append({
            "elapsed_s": elapsed_s,
            "phase": phase,
            "state": "localized",
            "match_score_m2": 0.01,
            "valid_beams": 260,
            "scan_age_s": 0.08,
            "processing_time_ms": 8.0,
            "selection_reason": selection_reason,
            "correction_event_id": correction_event_id,
        })
        lidar_rows.append({
            "elapsed_s": elapsed_s,
            "phase": phase,
            "session_id": "session-1",
            "invalid_packet_count": 0,
            "incomplete_scans": 0,
            "sequence_gaps": 0,
        })
        encoder_rows.append({
            "elapsed_s": elapsed_s,
            "phase": phase,
            "level": 0,
            "packet_age_s": 0.02,
            "invalid_packet_count": 0,
            "sequence_gaps": 0,
            "session_changes": 0,
            "invalid_transition_count": 0,
        })
    return status_rows, pose_rows, odom_rows, lidar_rows, encoder_rows


def _summarize(rows, expected_yaw_rad=math.radians(30.0)):
    status_rows, pose_rows, odom_rows, lidar_rows, encoder_rows = rows
    return summarize_rotation_trial(
        status_rows,
        pose_rows,
        odom_rows,
        lidar_rows,
        expected_yaw_rad=expected_yaw_rad,
        requested_duration_s=25.0,
        actual_duration_s=25.0,
        encoder_rows=encoder_rows,
    )


def test_rotation_summary_passes_for_left_thirty_degrees():
    summary = _summarize(_rows())

    assert summary["status"] == "passed"
    assert summary["measured_yaw_change_deg"] == pytest.approx(30.0)
    assert summary["expected_yaw_error_deg"] == pytest.approx(0.0)
    assert summary["rotation_direction_correct"] is True
    assert summary["correction_event_delta"] == 2
    assert summary["position_drift_m"] == pytest.approx(0.02)
    assert summary["failures"] == []


def test_rotation_summary_handles_angle_wrap():
    expected = math.radians(30.0)
    summary = _summarize(
        _rows(
            expected_yaw_rad=expected,
            measured_yaw_rad=expected,
            yaw_start=math.radians(170.0),
        ),
        expected_yaw_rad=expected,
    )

    assert summary["status"] == "passed"
    assert summary["measured_yaw_change_deg"] == pytest.approx(30.0)


def test_rotation_summary_rejects_wrong_yaw():
    summary = _summarize(_rows(measured_yaw_rad=math.radians(10.0)))

    assert summary["status"] == "failed"
    assert summary["expected_yaw_error_deg"] == pytest.approx(20.0)
    assert any(
        "expected-yaw error" in failure
        for failure in summary["failures"]
    )


def test_rotation_summary_rejects_opposite_direction():
    summary = _summarize(_rows(measured_yaw_rad=math.radians(-30.0)))

    assert summary["status"] == "failed"
    assert summary["rotation_direction_correct"] is False
    assert any(
        "opposite direction" in failure
        for failure in summary["failures"]
    )


def test_rotation_summary_requires_lidar_correction():
    summary = _summarize(_rows(correction_events=0))

    assert summary["status"] == "failed"
    assert summary["correction_event_delta"] == 0
    assert any(
        "correction event delta" in failure
        for failure in summary["failures"]
    )


def test_rotation_summary_rejects_position_drift():
    summary = _summarize(_rows(position_drift_m=0.25))

    assert summary["status"] == "failed"
    assert summary["position_drift_m"] == pytest.approx(0.25)
    assert any(
        "position drift" in failure
        for failure in summary["failures"]
    )


def test_rotation_summary_rejects_lidar_sequence_gap():
    rows = list(_rows())
    for row in rows[3][125:]:
        row["sequence_gaps"] = 1

    summary = _summarize(tuple(rows))

    assert summary["status"] == "failed"
    assert summary["lidar_sequence_gap_delta"] == 1
    assert any(
        "LiDAR sequence gap delta" in failure
        for failure in summary["failures"]
    )
