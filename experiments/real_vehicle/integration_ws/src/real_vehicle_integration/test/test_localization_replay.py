import csv
import json

import numpy as np
import pytest

from real_vehicle_integration.localization_replay import (
    REPLAY_PRESETS,
    _summarize_replay,
    compare_trial,
    decode_scan_row,
    replay_scans,
)
from real_vehicle_integration.localization_replay_batch import compare_trials
from real_vehicle_integration.real_map_localizer import GridMapLocalizer


class ConstantScoreMap:
    def scan_score(self, _pose, *_args, **_kwargs):
        return 0.01, 1


def scan_row(stamp_s, elapsed_s, phase):
    return {
        "stamp_s": str(stamp_s),
        "elapsed_s": str(elapsed_s),
        "phase": phase,
        "angle_min_rad": "0.0",
        "angle_increment_rad": "1.0",
        "range_min_m": "0.15",
        "range_max_m": "12.0",
        "beam_count": "2",
        "ranges_json": json.dumps([1.0, None]),
    }


def odom_row(stamp_s, x_m):
    return {
        "stamp_s": str(stamp_s),
        "x_m": str(x_m),
        "y_m": "0.0",
        "yaw_rad": "0.0",
    }


def test_decode_scan_row_restores_invalid_ranges_as_infinity():
    decoded = decode_scan_row(scan_row(1.0, 0.0, "baseline"))

    assert np.allclose(decoded["ranges"][:1], [1.0])
    assert np.isinf(decoded["ranges"][1])


def test_replay_scans_applies_timestamp_aligned_odometry():
    localizer = GridMapLocalizer(
        ConstantScoreMap(),
        xy_step_m=0.05,
        theta_step_rad=0.05,
        search_xy_m=0.0,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
    )
    replay_rows, pose_rows = replay_scans(
        [
            scan_row(1.0, 0.0, "baseline"),
            scan_row(2.0, 1.0, "settle"),
        ],
        [
            odom_row(0.9, 0.0),
            odom_row(1.9, 0.5),
        ],
        np.array([0.0, 0.0, 0.0]),
        localizer,
    )

    assert len(replay_rows) == 2
    assert len(pose_rows) == 2
    assert replay_rows[-1]["estimated_x_m"] == 0.5
    assert replay_rows[-1]["selection_reason"] == "prediction_best"


def test_presets_keep_historical_and_improved_settings_separate():
    assert REPLAY_PRESETS["baseline"]["xy_step_m"] == 0.15
    assert REPLAY_PRESETS["baseline"][
        "minimum_objective_improvement_m2"
    ] == 0.0
    assert REPLAY_PRESETS["improved"]["xy_step_m"] == 0.05
    assert REPLAY_PRESETS["improved"][
        "minimum_objective_improvement_m2"
    ] == 0.001
    assert REPLAY_PRESETS["guarded"]["xy_step_m"] == 0.15
    assert REPLAY_PRESETS["guarded"][
        "correction_confirmation_scans"
    ] == 2
    assert REPLAY_PRESETS["guarded"]["correction_cooldown_scans"] == 2


def _write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def _write_map(directory):
    pgm_path = directory / "map.pgm"
    rows = []
    for _ in range(20):
        pixels = [255] * 40
        pixels[10] = 0
        rows.append(" ".join(str(value) for value in pixels))
    pgm_path.write_text(
        "P2\n40 20\n255\n" + "\n".join(rows) + "\n",
        encoding="ascii",
    )
    yaml_path = directory / "map.yaml"
    yaml_path.write_text(
        "\n".join([
            "image: map.pgm",
            "resolution: 0.1",
            "origin: [0.0, -1.0, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.25",
        ]) + "\n",
        encoding="ascii",
    )
    return yaml_path


def test_compare_trial_writes_both_preset_artifacts(tmp_path):
    trial = tmp_path / "trial"
    trial.mkdir()
    map_yaml = _write_map(tmp_path)
    scans = []
    odometry = []
    poses = []
    for index, (phase, x_m) in enumerate((
        ("baseline", 0.0),
        ("baseline", 0.0),
        ("settle", 0.5),
        ("settle", 0.5),
    )):
        stamp = float(index + 1)
        scan = scan_row(stamp, float(index), phase)
        scan["beam_count"] = "20"
        scan["angle_increment_rad"] = "0.0"
        scan["ranges_json"] = json.dumps([0.75 - x_m] * 20)
        scans.append(scan)
        pose = {
            "elapsed_s": str(index),
            "phase": phase,
            "stamp_s": str(stamp - 0.1),
            "x_m": str(x_m),
            "y_m": "0.0",
            "yaw_rad": "0.0",
        }
        odometry.append(dict(pose))
        poses.append(dict(pose))
    _write_csv(trial / "scan.csv", scans)
    _write_csv(trial / "odom.csv", odometry)
    _write_csv(trial / "pose.csv", poses)
    _write_csv(trial / "localization_status.csv", [{
        "map_yaml": str(map_yaml),
    }])
    (trial / "summary.json").write_text(
        json.dumps({
            "expected_distance_m": 0.5,
            "thresholds": {
                "minimum_localized_rate": 0.99,
                "maximum_expected_distance_error_m": 0.15,
                "maximum_pose_odom_distance_difference_m": 0.15,
                "maximum_lateral_displacement_m": 0.15,
            },
        }),
        encoding="utf-8",
    )

    output = tmp_path / "comparison"
    comparison = compare_trial(trial, output)

    assert comparison["presets"] == ["baseline", "improved", "guarded"]
    assert (output / "baseline_replay.csv").is_file()
    assert (output / "baseline_summary.json").is_file()
    assert (output / "improved_replay.csv").is_file()
    assert (output / "improved_summary.json").is_file()
    assert (output / "guarded_replay.csv").is_file()
    assert (output / "guarded_summary.json").is_file()
    assert (output / "comparison.json").is_file()

    batch_output = tmp_path / "batch"
    batch = compare_trials(
        [trial],
        batch_output,
        maximum_mean_pose_odom_difference_m=1.0,
        maximum_trial_pose_odom_difference_m=1.0,
    )

    assert batch["status"] == "passed"
    assert batch["recommended_preset"] == "guarded"
    assert batch["trial_count"] == 1
    assert (batch_output / "aggregate.json").is_file()
    assert (batch_output / "aggregate.csv").is_file()
    assert (
        batch_output
        / "trials"
        / "trial"
        / "comparison.json"
    ).is_file()


def _rotation_replay_row(elapsed_s, phase, event_id):
    return {
        "elapsed_s": elapsed_s,
        "phase": phase,
        "state": "localized",
        "match_score_m2": 0.01,
        "processing_time_ms": 3.0,
        "selection_reason": (
            "scan_correction" if event_id else "prediction_best"
        ),
        "correction_event_id": event_id,
        "correction_proposal_count": event_id,
        "correction_pending_count": event_id,
        "correction_rejection_count": 0,
        "pending_reset_count": 0,
        "confirmation_count": min(event_id, 2),
    }


def _rotation_pose_row(elapsed_s, phase, yaw_rad):
    return {
        "elapsed_s": elapsed_s,
        "phase": phase,
        "stamp_s": elapsed_s,
        "x_m": 0.02 * elapsed_s / 5.0,
        "y_m": 0.0,
        "yaw_rad": yaw_rad,
    }


def test_rotation_replay_uses_known_yaw_instead_of_odom_distance():
    expected_yaw = np.deg2rad(30.0)
    replay_rows = [
        _rotation_replay_row(0.0, "baseline", 0),
        _rotation_replay_row(1.0, "baseline", 0),
        _rotation_replay_row(2.0, "rotation", 1),
        _rotation_replay_row(3.0, "rotation", 2),
        _rotation_replay_row(4.0, "settle", 2),
        _rotation_replay_row(5.0, "settle", 2),
    ]
    pose_rows = [
        _rotation_pose_row(0.0, "baseline", 0.0),
        _rotation_pose_row(1.0, "baseline", 0.0),
        _rotation_pose_row(4.0, "settle", expected_yaw),
        _rotation_pose_row(5.0, "settle", expected_yaw),
    ]
    odom_rows = [
        _rotation_pose_row(0.0, "baseline", 0.0),
        _rotation_pose_row(1.0, "baseline", 0.0),
        _rotation_pose_row(4.0, "settle", 0.0),
        _rotation_pose_row(5.0, "settle", 0.0),
    ]
    summary = _summarize_replay(
        replay_rows,
        pose_rows,
        odom_rows,
        {
            "trial_type": "manual_rotation",
            "expected_yaw_rad": expected_yaw,
            "thresholds": {
                "minimum_localized_rate": 0.99,
                "maximum_expected_yaw_error_rad": np.deg2rad(10.0),
                "maximum_position_drift_m": 0.20,
                "minimum_correction_event_delta": 1,
            },
        },
        "guarded",
    )

    assert summary["status"] == "passed"
    assert summary["trial_type"] == "manual_rotation"
    assert summary["expected_yaw_error_deg"] == 0.0
    assert summary["correction_events"] == 2
    assert summary["pose_odom_distance_difference_m"] is None


def test_rotation_batch_selects_guarded_by_yaw_error(
    tmp_path,
    monkeypatch,
):
    trials = []
    for name in ("left_30_trial1", "left_30_trial2"):
        trial = tmp_path / name
        trial.mkdir()
        for filename in ("scan.csv", "odom.csv", "pose.csv"):
            (trial / filename).write_text("", encoding="utf-8")
        (trial / "summary.json").write_text(
            json.dumps({
                "trial_type": "manual_rotation",
                "interrupted": False,
            }),
            encoding="utf-8",
        )
        trials.append(trial)

    def fake_compare(trial_dir, output_dir, map_yaml=None, presets=()):
        del map_yaml
        summaries = {}
        for preset in presets:
            yaw_error = {
                "baseline": np.deg2rad(7.0),
                "improved": np.deg2rad(12.0),
                "guarded": np.deg2rad(3.0),
            }[preset]
            summaries[preset] = {
                "status": "failed" if preset == "improved" else "passed",
                "expected_yaw_error_rad": yaw_error,
                "position_drift_m": 0.05,
                "processing_time_ms_mean": 4.0,
                "processing_time_ms_max": 6.0,
                "correction_events": 2,
                "selection_reason_counts": {"scan_correction": 2},
            }
        return {
            "preferred_preset": "guarded",
            "preferred_metric": "yaw_error_then_position_drift",
            "preferred_by_pose_odom_distance": None,
            "preferred_by_rotation_error": "guarded",
            "summaries": summaries,
        }

    monkeypatch.setattr(
        "real_vehicle_integration.localization_replay_batch.compare_trial",
        fake_compare,
    )
    result = compare_trials(
        trials,
        tmp_path / "rotation_batch",
    )

    assert result["status"] == "passed"
    assert result["trial_type"] == "manual_rotation"
    assert result["recommended_preset"] == "guarded"
    assert result["aggregates"]["guarded"][
        "expected_yaw_error_mean_deg"
    ] == pytest.approx(3.0)
