"""Offline comparison of saved real-vehicle localization trials."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np

from .real_map_localizer import (
    GridMapLocalizer,
    OccupancyMap,
    _compose_pose,
    normalize_angle,
    odom_delta,
)


REPLAY_PRESETS = {
    "baseline": {
        "xy_step_m": 0.15,
        "theta_step_rad": 0.10,
        "search_xy_m": 0.30,
        "search_theta_rad": 0.25,
        "minimum_valid_beams": 20,
        "maximum_match_score_m2": 0.25,
        "max_scan_beams": 90,
        "max_error_m": 0.40,
        "unknown_penalty_m": 0.25,
        "outside_penalty_m": 0.75,
        "lidar_x_m": 0.25,
        "lidar_y_m": 0.0,
        "lidar_yaw_rad": 0.0,
        "position_prior_weight": 0.20,
        "yaw_prior_weight": 0.05,
        "minimum_scan_score_improvement_m2": 0.001,
        "minimum_objective_improvement_m2": 0.0,
        "maximum_position_correction_m": 0.22,
        "maximum_yaw_correction_rad": 0.16,
        "correction_confirmation_scans": 1,
        "correction_cooldown_scans": 0,
        "correction_consistency_position_m": 0.01,
        "correction_consistency_yaw_rad": 0.01,
    },
    "improved": {
        "xy_step_m": 0.05,
        "theta_step_rad": 0.05,
        "search_xy_m": 0.30,
        "search_theta_rad": 0.25,
        "minimum_valid_beams": 20,
        "maximum_match_score_m2": 0.25,
        "max_scan_beams": 90,
        "max_error_m": 0.40,
        "unknown_penalty_m": 0.25,
        "outside_penalty_m": 0.75,
        "lidar_x_m": 0.25,
        "lidar_y_m": 0.0,
        "lidar_yaw_rad": 0.0,
        "position_prior_weight": 0.20,
        "yaw_prior_weight": 0.05,
        "minimum_scan_score_improvement_m2": 0.001,
        "minimum_objective_improvement_m2": 0.001,
        "maximum_position_correction_m": 0.10,
        "maximum_yaw_correction_rad": 0.10,
        "correction_confirmation_scans": 1,
        "correction_cooldown_scans": 0,
        "correction_consistency_position_m": 0.01,
        "correction_consistency_yaw_rad": 0.01,
    },
    "guarded": {
        "xy_step_m": 0.15,
        "theta_step_rad": 0.10,
        "search_xy_m": 0.30,
        "search_theta_rad": 0.25,
        "minimum_valid_beams": 20,
        "maximum_match_score_m2": 0.25,
        "max_scan_beams": 90,
        "max_error_m": 0.40,
        "unknown_penalty_m": 0.25,
        "outside_penalty_m": 0.75,
        "lidar_x_m": 0.25,
        "lidar_y_m": 0.0,
        "lidar_yaw_rad": 0.0,
        "position_prior_weight": 0.20,
        "yaw_prior_weight": 0.05,
        "minimum_scan_score_improvement_m2": 0.001,
        "minimum_objective_improvement_m2": 0.001,
        "maximum_position_correction_m": 0.22,
        "maximum_yaw_correction_rad": 0.16,
        "correction_confirmation_scans": 2,
        "correction_cooldown_scans": 2,
        "correction_consistency_position_m": 0.01,
        "correction_consistency_yaw_rad": 0.01,
    },
}

REPLAY_FIELDS = (
    "elapsed_s",
    "phase",
    "scan_stamp_s",
    "odom_stamp_s",
    "state",
    "predicted_x_m",
    "predicted_y_m",
    "predicted_yaw_rad",
    "estimated_x_m",
    "estimated_y_m",
    "estimated_yaw_rad",
    "match_score_m2",
    "predicted_match_score_m2",
    "objective_score_m2",
    "prior_cost_m2",
    "scan_improvement_m2",
    "objective_improvement_m2",
    "selection_reason",
    "correction_dx_m",
    "correction_dy_m",
    "correction_dyaw_rad",
    "proposed_correction_dx_m",
    "proposed_correction_dy_m",
    "proposed_correction_dyaw_rad",
    "correction_event_id",
    "correction_proposal_count",
    "correction_pending_count",
    "correction_rejection_count",
    "pending_reset_count",
    "confirmation_count",
    "cooldown_scans_remaining",
    "pending_correction_dx_m",
    "pending_correction_dy_m",
    "pending_correction_dyaw_rad",
    "valid_beams",
    "candidate_count",
    "processing_time_ms",
)


def _read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _float(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def decode_scan_row(row):
    """Decode one scan.csv row into the GridMapLocalizer scan contract."""

    encoded = json.loads(row["ranges_json"])
    if not isinstance(encoded, list):
        raise ValueError("ranges_json must contain a JSON array")
    ranges = np.asarray([
        float("inf") if value is None else float(value)
        for value in encoded
    ], dtype=float)
    expected = int(row.get("beam_count", ranges.size))
    if ranges.size != expected:
        raise ValueError(
            f"scan beam count mismatch: ranges={ranges.size} field={expected}"
        )
    return {
        "angle_min": float(row["angle_min_rad"]),
        "angle_increment": float(row["angle_increment_rad"]),
        "range_min": float(row["range_min_m"]),
        "range_max": float(row["range_max_m"]),
        "ranges": ranges,
    }


def _unique_sorted_scans(rows):
    unique = {}
    for row in rows:
        stamp = _float(row, "stamp_s")
        if stamp is None:
            continue
        unique.setdefault(stamp, row)
    return [unique[stamp] for stamp in sorted(unique)]


def _sorted_pose_rows(rows):
    return sorted(
        (
            row for row in rows
            if all(_float(row, key) is not None for key in (
                "stamp_s",
                "x_m",
                "y_m",
                "yaw_rad",
            ))
        ),
        key=lambda row: float(row["stamp_s"]),
    )


def _pose_array(row):
    return np.array([
        float(row["yaw_rad"]),
        float(row["x_m"]),
        float(row["y_m"]),
    ], dtype=float)


def replay_scans(scan_rows, odom_rows, initial_pose, localizer):
    """Replay recorded scans using timestamp-aligned odometry."""

    scans = _unique_sorted_scans(scan_rows)
    odometry = _sorted_pose_rows(odom_rows)
    if not scans:
        raise ValueError("scan.csv contains no decodable scan rows")
    if not odometry:
        raise ValueError("odom.csv contains no valid odometry rows")

    predicted_pose = np.asarray(initial_pose, dtype=float).copy()
    last_odom = None
    current_odom = None
    odom_index = 0
    replay_rows = []
    pose_rows = []

    for scan_row in scans:
        scan_stamp_s = float(scan_row["stamp_s"])
        while (
            odom_index < len(odometry)
            and float(odometry[odom_index]["stamp_s"])
            <= scan_stamp_s + 1e-9
        ):
            current_odom = odometry[odom_index]
            odom_index += 1
        if current_odom is None:
            continue

        current_odom_pose = _pose_array(current_odom)
        if last_odom is None:
            last_odom = current_odom_pose.copy()
        else:
            predicted_pose = _compose_pose(
                predicted_pose,
                odom_delta(last_odom, current_odom_pose),
            )
            last_odom = current_odom_pose.copy()
        prediction = predicted_pose.copy()

        started = time.perf_counter()
        estimate, score = localizer.estimate(
            decode_scan_row(scan_row),
            prediction,
        )
        processing_time_ms = 1000.0 * (time.perf_counter() - started)
        state = "localized" if localizer.last_success else "match_failed"
        if localizer.last_success:
            predicted_pose = estimate.copy()

        result = {
            "elapsed_s": float(scan_row.get("elapsed_s", 0.0)),
            "phase": str(scan_row.get("phase", "unknown")),
            "scan_stamp_s": scan_stamp_s,
            "odom_stamp_s": float(current_odom["stamp_s"]),
            "state": state,
            "predicted_x_m": float(prediction[1]),
            "predicted_y_m": float(prediction[2]),
            "predicted_yaw_rad": float(prediction[0]),
            "estimated_x_m": float(estimate[1]),
            "estimated_y_m": float(estimate[2]),
            "estimated_yaw_rad": float(estimate[0]),
            "match_score_m2": float(score),
            "predicted_match_score_m2": (
                float(localizer.last_predicted_score)
            ),
            "objective_score_m2": float(localizer.last_objective_score),
            "prior_cost_m2": float(localizer.last_prior_cost),
            "scan_improvement_m2": (
                float(localizer.last_scan_improvement_m2)
            ),
            "objective_improvement_m2": (
                float(localizer.last_objective_improvement_m2)
            ),
            "selection_reason": localizer.last_selection_reason,
            "correction_dx_m": float(localizer.last_correction_dx_m),
            "correction_dy_m": float(localizer.last_correction_dy_m),
            "correction_dyaw_rad": (
                float(localizer.last_correction_dyaw_rad)
            ),
            "proposed_correction_dx_m": (
                float(localizer.last_proposed_correction_dx_m)
            ),
            "proposed_correction_dy_m": (
                float(localizer.last_proposed_correction_dy_m)
            ),
            "proposed_correction_dyaw_rad": (
                float(localizer.last_proposed_correction_dyaw_rad)
            ),
            "correction_event_id": int(localizer.correction_event_id),
            "correction_proposal_count": int(
                localizer.correction_proposal_count
            ),
            "correction_pending_count": int(
                localizer.correction_pending_count
            ),
            "correction_rejection_count": int(
                localizer.correction_rejection_count
            ),
            "pending_reset_count": int(localizer.pending_reset_count),
            "confirmation_count": int(
                localizer.last_confirmation_count
            ),
            "cooldown_scans_remaining": int(
                localizer.last_cooldown_scans_remaining
            ),
            "pending_correction_dx_m": float(
                localizer.pending_correction_dx_m
            ),
            "pending_correction_dy_m": float(
                localizer.pending_correction_dy_m
            ),
            "pending_correction_dyaw_rad": float(
                localizer.pending_correction_dyaw_rad
            ),
            "valid_beams": int(localizer.last_valid_beams),
            "candidate_count": int(localizer.last_candidate_count),
            "processing_time_ms": processing_time_ms,
        }
        replay_rows.append(result)
        if localizer.last_success:
            pose_rows.append({
                "elapsed_s": result["elapsed_s"],
                "phase": result["phase"],
                "stamp_s": result["scan_stamp_s"],
                "x_m": result["estimated_x_m"],
                "y_m": result["estimated_y_m"],
                "yaw_rad": result["estimated_yaw_rad"],
            })

    if not replay_rows:
        raise ValueError("no scan had matching odometry")
    return replay_rows, pose_rows


def _phase_tail(rows, phase, window_s=2.0):
    selected = [
        row for row in rows
        if row.get("phase") == phase
        and _float(row, "elapsed_s") is not None
    ]
    if not selected:
        return []
    end_s = max(float(row["elapsed_s"]) for row in selected)
    return [
        row for row in selected
        if float(row["elapsed_s"]) >= end_s - float(window_s)
    ]


def _median_pose(rows):
    if not rows:
        return None
    xs = [float(row["x_m"]) for row in rows]
    ys = [float(row["y_m"]) for row in rows]
    yaws = [float(row["yaw_rad"]) for row in rows]
    sine = statistics.mean(math.sin(value) for value in yaws)
    cosine = statistics.mean(math.cos(value) for value in yaws)
    return {
        "x_m": statistics.median(xs),
        "y_m": statistics.median(ys),
        "yaw_rad": math.atan2(sine, cosine),
        "samples": len(rows),
    }


def _pose_delta(start, end):
    if start is None or end is None:
        return None
    dx = end["x_m"] - start["x_m"]
    dy = end["y_m"] - start["y_m"]
    yaw = start["yaw_rad"]
    return {
        "dx_m": dx,
        "dy_m": dy,
        "distance_m": math.hypot(dx, dy),
        "forward_m": math.cos(yaw) * dx + math.sin(yaw) * dy,
        "lateral_m": -math.sin(yaw) * dx + math.cos(yaw) * dy,
        "yaw_change_rad": normalize_angle(end["yaw_rad"] - yaw),
    }


def _unwrapped_span(angles):
    if not angles:
        return None
    unwrapped = [float(angles[0])]
    for angle in angles[1:]:
        delta = normalize_angle(float(angle) - unwrapped[-1])
        unwrapped.append(unwrapped[-1] + delta)
    return max(unwrapped) - min(unwrapped)


def _summarize_rotation_replay(
    replay_rows,
    pose_rows,
    odom_rows,
    original_summary,
    preset,
):
    baseline_pose = _median_pose(_phase_tail(pose_rows, "baseline"))
    final_pose = _median_pose(_phase_tail(pose_rows, "settle"))
    baseline_odom = _median_pose(_phase_tail(odom_rows, "baseline"))
    final_odom = _median_pose(_phase_tail(odom_rows, "settle"))
    localizer_delta = _pose_delta(baseline_pose, final_pose)
    replay_odom_delta = _pose_delta(baseline_odom, final_odom)
    expected_yaw_rad = float(original_summary["expected_yaw_rad"])
    thresholds = dict(original_summary.get("thresholds", {}))
    failures = []
    success_count = sum(
        row["state"] == "localized" for row in replay_rows
    )
    localized_rate = success_count / len(replay_rows)

    expected_yaw_error = None
    position_drift = None
    measured_yaw_change = None
    direction_correct = False
    if localizer_delta is None:
        failures.append("baseline or settle replay pose is unavailable")
    else:
        measured_yaw_change = localizer_delta["yaw_change_rad"]
        expected_yaw_error = abs(normalize_angle(
            measured_yaw_change - expected_yaw_rad
        ))
        position_drift = localizer_delta["distance_m"]
        direction_correct = measured_yaw_change * expected_yaw_rad > 0.0
        if not direction_correct:
            failures.append(
                "replay yaw changed in the opposite direction "
                f"({measured_yaw_change:.6f} rad)"
            )
        maximum_yaw_error = float(
            thresholds.get(
                "maximum_expected_yaw_error_rad",
                math.radians(10.0),
            )
        )
        if expected_yaw_error > maximum_yaw_error + 1e-9:
            failures.append(
                f"expected-yaw error {expected_yaw_error:.6f} rad "
                f"exceeds {maximum_yaw_error:.6f} rad"
            )
        maximum_position_drift = float(
            thresholds.get("maximum_position_drift_m", 0.20)
        )
        if position_drift > maximum_position_drift + 1e-9:
            failures.append(
                f"position drift {position_drift:.6f} m exceeds "
                f"{maximum_position_drift:.6f} m"
            )

    minimum_rate = float(thresholds.get("minimum_localized_rate", 0.99))
    if localized_rate + 1e-12 < minimum_rate:
        failures.append(
            f"localized rate {localized_rate:.6f} is below "
            f"{minimum_rate:.6f}"
        )

    event_ids = [
        int(row["correction_event_id"]) for row in replay_rows
    ]
    correction_events = max(event_ids) - min(event_ids)
    minimum_events = int(
        thresholds.get("minimum_correction_event_delta", 1)
    )
    if correction_events < minimum_events:
        failures.append(
            f"correction event delta {correction_events} is below "
            f"{minimum_events}"
        )

    final_yaws = [
        float(row["yaw_rad"])
        for row in _phase_tail(pose_rows, "settle")
    ]
    final_yaw_span = _unwrapped_span(final_yaws)
    maximum_final_yaw_span = float(
        thresholds.get("maximum_final_yaw_span_rad", 0.10)
    )
    if (
        final_yaw_span is not None
        and final_yaw_span > maximum_final_yaw_span + 1e-9
    ):
        failures.append(
            f"final yaw span {final_yaw_span:.6f} rad exceeds "
            f"{maximum_final_yaw_span:.6f} rad"
        )

    finite_scores = [
        float(row["match_score_m2"])
        for row in replay_rows
        if math.isfinite(float(row["match_score_m2"]))
    ]
    processing = [
        float(row["processing_time_ms"]) for row in replay_rows
    ]
    reasons = Counter(row["selection_reason"] for row in replay_rows)
    return {
        "status": "failed" if failures else "passed",
        "failures": failures,
        "trial_type": "manual_rotation",
        "preset": preset,
        "configuration": dict(REPLAY_PRESETS[preset]),
        "scan_samples": len(replay_rows),
        "localized_samples": success_count,
        "localized_rate": localized_rate,
        "baseline_pose": baseline_pose,
        "final_pose": final_pose,
        "localizer_delta": localizer_delta,
        "odom_delta": replay_odom_delta,
        "expected_yaw_rad": expected_yaw_rad,
        "expected_yaw_deg": math.degrees(expected_yaw_rad),
        "measured_yaw_change_rad": measured_yaw_change,
        "measured_yaw_change_deg": (
            math.degrees(measured_yaw_change)
            if measured_yaw_change is not None
            else None
        ),
        "expected_yaw_error_rad": expected_yaw_error,
        "expected_yaw_error_deg": (
            math.degrees(expected_yaw_error)
            if expected_yaw_error is not None
            else None
        ),
        "position_drift_m": position_drift,
        "rotation_direction_correct": direction_correct,
        "final_yaw_span_rad": final_yaw_span,
        "pose_odom_distance_difference_m": None,
        "localizer_expected_distance_error_m": None,
        "correction_events": correction_events,
        "correction_proposals": max(
            int(row["correction_proposal_count"]) for row in replay_rows
        ),
        "correction_pending_decisions": max(
            int(row["correction_pending_count"]) for row in replay_rows
        ),
        "correction_rejections": max(
            int(row["correction_rejection_count"]) for row in replay_rows
        ),
        "pending_resets": max(
            int(row["pending_reset_count"]) for row in replay_rows
        ),
        "maximum_confirmation_count": max(
            int(row["confirmation_count"]) for row in replay_rows
        ),
        "selection_reason_counts": dict(sorted(reasons.items())),
        "match_score_m2_mean": (
            statistics.mean(finite_scores) if finite_scores else None
        ),
        "match_score_m2_max": max(finite_scores) if finite_scores else None,
        "processing_time_ms_mean": statistics.mean(processing),
        "processing_time_ms_max": max(processing),
    }


def _summarize_replay(
    replay_rows,
    pose_rows,
    odom_rows,
    original_summary,
    preset,
):
    if original_summary.get("trial_type") == "manual_rotation":
        return _summarize_rotation_replay(
            replay_rows,
            pose_rows,
            odom_rows,
            original_summary,
            preset,
        )

    baseline_pose = _median_pose(_phase_tail(pose_rows, "baseline"))
    final_pose = _median_pose(_phase_tail(pose_rows, "settle"))
    baseline_odom = _median_pose(_phase_tail(odom_rows, "baseline"))
    final_odom = _median_pose(_phase_tail(odom_rows, "settle"))
    localizer_delta = _pose_delta(baseline_pose, final_pose)
    replay_odom_delta = _pose_delta(baseline_odom, final_odom)
    expected_distance = float(
        original_summary.get("expected_distance_m", 0.5)
    )
    thresholds = dict(original_summary.get("thresholds", {}))
    failures = []
    success_count = sum(
        row["state"] == "localized" for row in replay_rows
    )
    localized_rate = success_count / len(replay_rows)

    localizer_error = None
    pose_odom_difference = None
    if localizer_delta is None:
        failures.append("baseline or settle replay pose is unavailable")
    else:
        localizer_error = abs(
            localizer_delta["forward_m"] - expected_distance
        )
        maximum = float(
            thresholds.get("maximum_expected_distance_error_m", 0.15)
        )
        if localizer_error > maximum + 1e-9:
            failures.append(
                f"expected-distance error {localizer_error:.6f} m "
                f"exceeds {maximum:.6f} m"
            )
        maximum_lateral = float(
            thresholds.get("maximum_lateral_displacement_m", 0.15)
        )
        if abs(localizer_delta["lateral_m"]) > maximum_lateral + 1e-9:
            failures.append(
                f"lateral displacement "
                f"{abs(localizer_delta['lateral_m']):.6f} m exceeds "
                f"{maximum_lateral:.6f} m"
            )

    if localizer_delta is not None and replay_odom_delta is not None:
        pose_odom_difference = abs(
            localizer_delta["distance_m"]
            - replay_odom_delta["distance_m"]
        )
        maximum = float(
            thresholds.get(
                "maximum_pose_odom_distance_difference_m",
                0.15,
            )
        )
        if pose_odom_difference > maximum + 1e-9:
            failures.append(
                f"pose/odometry distance difference "
                f"{pose_odom_difference:.6f} m exceeds {maximum:.6f} m"
            )

    minimum_rate = float(thresholds.get("minimum_localized_rate", 0.99))
    if localized_rate + 1e-12 < minimum_rate:
        failures.append(
            f"localized rate {localized_rate:.6f} is below "
            f"{minimum_rate:.6f}"
        )

    finite_scores = [
        float(row["match_score_m2"])
        for row in replay_rows
        if math.isfinite(float(row["match_score_m2"]))
    ]
    processing = [
        float(row["processing_time_ms"]) for row in replay_rows
    ]
    reasons = Counter(row["selection_reason"] for row in replay_rows)
    return {
        "status": "failed" if failures else "passed",
        "failures": failures,
        "trial_type": "manual_motion",
        "preset": preset,
        "configuration": dict(REPLAY_PRESETS[preset]),
        "scan_samples": len(replay_rows),
        "localized_samples": success_count,
        "localized_rate": localized_rate,
        "baseline_pose": baseline_pose,
        "final_pose": final_pose,
        "localizer_delta": localizer_delta,
        "odom_delta": replay_odom_delta,
        "expected_distance_m": expected_distance,
        "localizer_expected_distance_error_m": localizer_error,
        "pose_odom_distance_difference_m": pose_odom_difference,
        "correction_events": max(
            int(row["correction_event_id"]) for row in replay_rows
        ),
        "correction_proposals": max(
            int(row["correction_proposal_count"]) for row in replay_rows
        ),
        "correction_pending_decisions": max(
            int(row["correction_pending_count"]) for row in replay_rows
        ),
        "correction_rejections": max(
            int(row["correction_rejection_count"]) for row in replay_rows
        ),
        "pending_resets": max(
            int(row["pending_reset_count"]) for row in replay_rows
        ),
        "maximum_confirmation_count": max(
            int(row["confirmation_count"]) for row in replay_rows
        ),
        "selection_reason_counts": dict(sorted(reasons.items())),
        "match_score_m2_mean": (
            statistics.mean(finite_scores) if finite_scores else None
        ),
        "match_score_m2_max": max(finite_scores) if finite_scores else None,
        "processing_time_ms_mean": statistics.mean(processing),
        "processing_time_ms_max": max(processing),
    }


def _write_csv(path, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=REPLAY_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def _derive_map_yaml(trial_dir, explicit_path):
    if explicit_path is not None:
        return explicit_path.expanduser().resolve()
    status_path = trial_dir / "localization_status.csv"
    if status_path.exists():
        for row in _read_csv(status_path):
            value = str(row.get("map_yaml", "")).strip()
            if value:
                return Path(value).expanduser().resolve()
    raise ValueError(
        "map YAML could not be derived; pass --map-yaml explicitly"
    )


def _initial_pose(pose_rows):
    poses = _sorted_pose_rows(pose_rows)
    if not poses:
        raise ValueError("pose.csv contains no valid pose")
    return _pose_array(poses[0])


def compare_trial(
    trial_dir,
    output_dir,
    map_yaml=None,
    presets=("baseline", "improved", "guarded"),
):
    """Replay one recorded trial and write per-preset comparison artifacts."""

    trial_dir = Path(trial_dir).expanduser().resolve()
    output_dir = Path(output_dir).expanduser().resolve()
    required = ("scan.csv", "odom.csv", "pose.csv", "summary.json")
    missing = [name for name in required if not (trial_dir / name).exists()]
    if missing:
        raise FileNotFoundError(
            "trial is missing replay data: " + ", ".join(missing)
        )
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(
            f"replay output directory is not empty: {output_dir}"
        )
    output_dir.mkdir(parents=True, exist_ok=True)

    unknown = [name for name in presets if name not in REPLAY_PRESETS]
    if unknown:
        raise ValueError(f"unknown replay presets: {', '.join(unknown)}")
    map_path = _derive_map_yaml(trial_dir, map_yaml)
    occupancy_map = OccupancyMap(map_path)
    scan_rows = _read_csv(trial_dir / "scan.csv")
    odom_rows = _read_csv(trial_dir / "odom.csv")
    pose_rows = _read_csv(trial_dir / "pose.csv")
    original_summary = json.loads(
        (trial_dir / "summary.json").read_text(encoding="utf-8")
    )
    initial_pose = _initial_pose(pose_rows)

    summaries = {}
    for preset in presets:
        localizer = GridMapLocalizer(
            occupancy_map,
            **REPLAY_PRESETS[preset],
        )
        replay_rows, replay_pose_rows = replay_scans(
            scan_rows,
            odom_rows,
            initial_pose,
            localizer,
        )
        summary = _summarize_replay(
            replay_rows,
            replay_pose_rows,
            odom_rows,
            original_summary,
            preset,
        )
        summaries[preset] = summary
        _write_csv(output_dir / f"{preset}_replay.csv", replay_rows)
        (output_dir / f"{preset}_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    trial_type = str(
        original_summary.get("trial_type", "manual_motion")
    )
    if trial_type == "manual_rotation":
        comparable = {
            name: (
                summary["expected_yaw_error_rad"],
                summary["position_drift_m"],
                summary["processing_time_ms_mean"],
            )
            for name, summary in summaries.items()
            if summary["expected_yaw_error_rad"] is not None
            and summary["position_drift_m"] is not None
        }
        preferred = (
            min(comparable, key=comparable.get) if comparable else None
        )
        preferred_metric = "yaw_error_then_position_drift"
    else:
        comparable = {
            name: summary["pose_odom_distance_difference_m"]
            for name, summary in summaries.items()
            if summary["pose_odom_distance_difference_m"] is not None
        }
        preferred = (
            min(comparable, key=comparable.get) if comparable else None
        )
        preferred_metric = "pose_odom_distance"
    comparison = {
        "trial_dir": str(trial_dir),
        "trial_type": trial_type,
        "map_yaml": str(map_path),
        "presets": list(presets),
        "preferred_preset": preferred,
        "preferred_metric": preferred_metric,
        "preferred_by_pose_odom_distance": (
            preferred if trial_type != "manual_rotation" else None
        ),
        "preferred_by_rotation_error": (
            preferred if trial_type == "manual_rotation" else None
        ),
        "summaries": summaries,
    }
    (output_dir / "comparison.json").write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return comparison


def _parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial-dir", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument(
        "--presets",
        default="baseline,improved,guarded",
        help="Comma-separated preset names: baseline, improved, guarded",
    )
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args(args)


def main(args=None):
    options = _parse_args(args)
    presets = tuple(
        value.strip()
        for value in options.presets.split(",")
        if value.strip()
    )
    output_dir = options.output_dir
    if output_dir is None:
        output_dir = options.trial_dir / "replay_comparison"
    try:
        comparison = compare_trial(
            options.trial_dir,
            output_dir,
            map_yaml=options.map_yaml,
            presets=presets,
        )
    except (FileExistsError, FileNotFoundError, ValueError) as exc:
        print(f"replay failed: {exc}")
        return 2
    print(json.dumps(comparison, indent=2, sort_keys=True))
    print(f"output_dir: {Path(output_dir).expanduser().resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "REPLAY_PRESETS",
    "compare_trial",
    "decode_scan_row",
    "replay_scans",
]
