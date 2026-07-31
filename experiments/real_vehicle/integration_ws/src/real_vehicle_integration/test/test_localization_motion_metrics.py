import math

import pytest

from real_vehicle_integration.localization_motion_metrics import (
    summarize_motion_trial,
)


def _phase(index):
    if index < 50:
        return "baseline"
    if index < 200:
        return "motion"
    return "settle"


def _progress(index):
    if index < 50:
        return 0.0
    if index >= 200:
        return 0.5
    return 0.5 * (index - 50) / 150.0


def _normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _rows(
    localizer_scale=1.0,
    odom_scale=1.0,
    yaw_start=0.0,
    yaw_end=0.0,
    lateral_m=0.0,
):
    status_rows = []
    pose_rows = []
    odom_rows = []
    lidar_rows = []
    encoder_rows = []
    for index in range(250):
        elapsed_s = index * 0.1
        phase = _phase(index)
        progress = _progress(index)
        fraction = progress / 0.5
        yaw_change = _normalize_angle(yaw_end - yaw_start)
        yaw = _normalize_angle(yaw_start + yaw_change * fraction)
        forward_x = math.cos(yaw_start)
        forward_y = math.sin(yaw_start)
        lateral_x = -math.sin(yaw_start)
        lateral_y = math.cos(yaw_start)
        pose_rows.append({
            "elapsed_s": elapsed_s,
            "phase": phase,
            "x_m": (
                localizer_scale * progress * forward_x
                + lateral_m * fraction * lateral_x
            ),
            "y_m": (
                localizer_scale * progress * forward_y
                + lateral_m * fraction * lateral_y
            ),
            "yaw_rad": yaw,
        })
        odom_rows.append({
            "elapsed_s": elapsed_s,
            "phase": phase,
            "x_m": odom_scale * progress * forward_x,
            "y_m": odom_scale * progress * forward_y,
            "yaw_rad": yaw,
        })
        status_rows.append({
            "elapsed_s": elapsed_s,
            "phase": phase,
            "state": "localized",
            "match_score_m2": 0.01,
            "valid_beams": 260,
            "scan_age_s": 0.08,
            "processing_time_ms": 8.0,
            "selection_reason": "search_improved",
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
            "invalid_transition_count": 3,
        })
    return status_rows, pose_rows, odom_rows, lidar_rows, encoder_rows


def _summarize(rows):
    status_rows, pose_rows, odom_rows, lidar_rows, encoder_rows = rows
    return summarize_motion_trial(
        status_rows,
        pose_rows,
        odom_rows,
        lidar_rows,
        expected_distance_m=0.5,
        requested_duration_s=25.0,
        actual_duration_s=25.0,
        encoder_rows=encoder_rows,
    )


def test_motion_summary_passes_for_consistent_fifty_centimeters():
    summary = _summarize(_rows())

    assert summary["status"] == "passed"
    assert summary["localizer_delta"]["distance_m"] == pytest.approx(0.5)
    assert summary["odom_delta"]["distance_m"] == pytest.approx(0.5)
    assert summary["pose_odom_distance_difference_m"] == pytest.approx(0.0)
    assert summary["failures"] == []


def test_motion_summary_rejects_wrong_travel_distance():
    summary = _summarize(_rows(localizer_scale=0.4, odom_scale=0.4))

    assert summary["status"] == "failed"
    assert summary["localizer_delta"]["distance_m"] == pytest.approx(0.2)
    assert any(
        "localizer expected-distance error" in failure
        for failure in summary["failures"]
    )


def test_motion_summary_rejects_localizer_odom_disagreement():
    summary = _summarize(_rows(localizer_scale=1.0, odom_scale=0.4))

    assert summary["status"] == "failed"
    assert summary["pose_odom_distance_difference_m"] == pytest.approx(0.3)
    assert any(
        "localizer/odometry distance difference" in failure
        for failure in summary["failures"]
    )


def test_motion_summary_handles_yaw_wrap():
    rows = _rows(
        yaw_start=math.pi - 0.02,
        yaw_end=-math.pi + 0.02,
    )
    summary = _summarize(rows)

    assert summary["status"] == "passed"
    assert abs(summary["localizer_delta"]["yaw_change_rad"]) == pytest.approx(
        0.04
    )


def test_motion_summary_rejects_lidar_sequence_gap():
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


def test_motion_summary_accepts_value_at_threshold_with_float_roundoff():
    rows = _rows()
    for index, row in enumerate(rows[1]):
        if row["phase"] == "settle":
            row["yaw_rad"] = (
                0.10000000000000009
                if index % 2
                else 0.0
            )

    summary = _summarize(rows)

    assert summary["final_yaw_span_rad"] == pytest.approx(0.1)
    assert summary["status"] == "passed"
