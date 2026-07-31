"""Closed-loop controller simulation on the saved real-vehicle 2D map."""

import argparse
import copy
import json
from pathlib import Path
import time

import numpy as np

from ..localization import OccupancyGridLidar
from ..models import ActuatorModel, step_model
from .common import DEFAULT_CONFIG, load_real_config, make_controller, write_rows

from ..integration_ws.src.real_vehicle_integration.real_vehicle_integration.real_map_localizer import (
    GridMapLocalizer,
    OccupancyMap,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MAP = (
    REPO_ROOT
    / "experiments/real_vehicle/maps/small_test_area_03/small_test_area_03.yaml"
)
CONTROLLER_RATES_HZ = {
    "pure_pursuit": 50.0,
    "mpc": 50.0,
    "mppi": 100.0,
}
LIDAR_CONDITIONS = {
    "ideal": {},
    "noise": {
        "noise_std_m": 0.01,
        "distance_noise_slope": 0.002,
    },
    "noise_delay_dropout": {
        "noise_std_m": 0.01,
        "distance_noise_slope": 0.002,
        "dropout_probability": 0.05,
        "delay_s": 0.05,
    },
}
GUARDED_LOCALIZER = {
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
}


def _straight_reference(
    time_s,
    horizon,
    controller_dt,
    speed_mps,
    start_x_m,
    y_m,
    distance_m,
):
    times = time_s + controller_dt * np.arange(1, horizon + 1, dtype=float)
    progress = np.minimum(speed_mps * times, distance_m)
    speeds = np.where(progress < distance_m - 1e-9, speed_mps, 0.0)
    return np.column_stack((
        np.zeros(horizon),
        start_x_m + progress,
        np.full(horizon, y_m),
        np.zeros(horizon),
        speeds,
        np.zeros(horizon),
    ))


def _clearance(occupancy_map, state):
    row, column, inside = occupancy_map.world_to_cell(state[1], state[2])
    if not inside:
        return 0.0, True
    collision = bool(occupancy_map.occupied[row, column])
    return float(occupancy_map.distance_to_occupied_m[row, column]), collision


def run_map_case(
    controller_name,
    condition_name,
    seed,
    map_yaml=DEFAULT_MAP,
    vehicle_config=DEFAULT_CONFIG,
    output_dir=None,
    duration_s=4.0,
    distance_m=0.45,
    speed_mps=0.15,
    pose_source="localized",
):
    """Run one hardware-free map, LiDAR, localization, and control case."""

    if controller_name not in CONTROLLER_RATES_HZ:
        raise ValueError(f"unknown controller: {controller_name}")
    if condition_name not in LIDAR_CONDITIONS:
        raise ValueError(f"unknown LiDAR condition: {condition_name}")
    if pose_source not in ("ground_truth", "localized"):
        raise ValueError("pose_source must be ground_truth or localized")

    config, parameters, limits = load_real_config(vehicle_config)
    config = copy.deepcopy(config)
    if bool(config["simulation"].get("hardware_output_enabled", False)):
        raise RuntimeError("map simulation refuses hardware_output_enabled=true")
    config["controller"]["seed"] = int(seed)
    config["controller"]["speed_target_mps"] = float(speed_mps)
    dt = float(config["simulation"]["timestep_s"])
    controller_rate = CONTROLLER_RATES_HZ[controller_name]
    controller_dt = 1.0 / controller_rate
    controller_steps = max(int(round(controller_dt / dt)), 1)
    controller = make_controller(
        controller_name,
        parameters,
        limits,
        controller_dt,
        config["controller"],
    )
    actuator = ActuatorModel(
        parameters,
        limits,
        config["simulation"].get("actuator", {}),
        seed=int(seed),
    )
    occupancy_map = OccupancyMap(map_yaml)
    lidar_config = copy.deepcopy(config["lidar"])
    lidar_config.update({
        "num_beams": 72,
        "update_rate_hz": 5.0,
        "x_offset_m": 0.25,
        "y_offset_m": 0.0,
        "yaw_offset_rad": 0.0,
    })
    lidar_config.update(LIDAR_CONDITIONS[condition_name])
    lidar = OccupancyGridLidar(occupancy_map, lidar_config, seed=seed)
    localizer = GridMapLocalizer(occupancy_map, **GUARDED_LOCALIZER)

    state = np.array([0.0, -0.15, -0.006, 0.0], dtype=float)
    start_x_m = float(state[1])
    goal_x_m = start_x_m + float(distance_m)
    estimated_state = state.copy()
    actuator.reset(state)
    requested_control = np.zeros(2, dtype=float)
    next_scan_s = 0.0
    scan_period_s = 1.0 / float(lidar_config["update_rate_hz"])
    controller_times = []
    position_errors = []
    localization_errors = []
    valid_ratios = []
    rows = []
    collision = False
    reached_goal = False
    localizer_failures = 0

    total_steps = int(np.ceil(float(duration_s) / dt))
    for step_index in range(total_steps):
        now_s = step_index * dt
        if now_s + 1e-12 >= next_scan_s:
            scan = lidar.scan(state, now_s)
            valid_ratios.append(float(scan["valid_ratio"]))
            if pose_source == "localized":
                estimate, _score = localizer.estimate(
                    scan,
                    estimated_state[:3],
                )
                if localizer.last_success:
                    estimated_state[:3] = estimate
                else:
                    localizer_failures += 1
            next_scan_s += scan_period_s
        if pose_source == "ground_truth":
            estimated_state = state.copy()

        reference = _straight_reference(
            now_s,
            controller.horizon,
            controller_dt,
            speed_mps,
            start_x_m,
            state[2] if step_index == 0 else -0.006,
            distance_m,
        )
        if step_index % controller_steps == 0:
            requested_control, info = controller.plan(
                estimated_state,
                reference,
            )
            controller_times.append(1000.0 * float(info["solve_time_s"]))
        if reached_goal:
            requested_control = np.zeros(2, dtype=float)

        next_state, clamp, actuator_info = actuator.step(
            state,
            requested_control,
            parameters,
            dt,
            integrator=str(config["simulation"].get("integrator", "rk4")),
        )
        if pose_source == "localized":
            estimated_state, _ = step_model(
                estimated_state,
                [actuator_info.applied_v, clamp.applied_omega],
                parameters,
                limits,
                dt,
                integrator=str(config["simulation"].get("integrator", "rk4")),
            )

        reference_now = _straight_reference(
            now_s,
            1,
            dt,
            speed_mps,
            start_x_m,
            -0.006,
            distance_m,
        )[0]
        position_error = float(np.linalg.norm(
            state[1:3] - reference_now[1:3]
        ))
        localization_error = float(np.linalg.norm(
            state[1:3] - estimated_state[1:3]
        ))
        clearance_m, point_collision = _clearance(occupancy_map, state)
        collision = collision or point_collision
        reached_goal = reached_goal or bool(state[1] >= goal_x_m - 0.02)
        position_errors.append(position_error)
        localization_errors.append(localization_error)
        rows.append({
            "timestamp_s": now_s,
            "controller": controller_name,
            "lidar_condition": condition_name,
            "pose_source": pose_source,
            "x_m": float(state[1]),
            "y_m": float(state[2]),
            "phi_rad": float(state[0]),
            "estimated_x_m": float(estimated_state[1]),
            "estimated_y_m": float(estimated_state[2]),
            "estimated_phi_rad": float(estimated_state[0]),
            "reference_x_m": float(reference_now[1]),
            "reference_y_m": float(reference_now[2]),
            "position_error_m": position_error,
            "localization_error_m": localization_error,
            "requested_speed_mps": float(requested_control[0]),
            "requested_steer_rate_rad_s": float(requested_control[1]),
            "applied_speed_mps": float(actuator_info.applied_v),
            "applied_steer_rad": float(next_state[3]),
            "map_clearance_m": clearance_m,
            "map_collision": int(point_collision),
            "lidar_valid_ratio": float(lidar.last_valid_ratio),
            "localizer_state": (
                "localized"
                if pose_source == "ground_truth" or localizer.last_success
                else "match_failed"
            ),
        })
        state = next_state

    timing = np.asarray(controller_times or [0.0], dtype=float)
    summary = {
        "controller": controller_name,
        "lidar_condition": condition_name,
        "pose_source": pose_source,
        "seed": int(seed),
        "steps": len(rows),
        "duration_s": float(len(rows) * dt),
        "distance_target_m": float(distance_m),
        "completed": int(reached_goal and not collision),
        "reached_goal": int(reached_goal),
        "map_collision": int(collision),
        "final_x_m": float(state[1]),
        "final_y_m": float(state[2]),
        "position_rmse_m": float(np.sqrt(np.mean(np.square(position_errors)))),
        "localization_xy_rmse_m": float(
            np.sqrt(np.mean(np.square(localization_errors)))
        ),
        "controller_mean_ms": float(np.mean(timing)),
        "controller_p95_ms": float(np.percentile(timing, 95)),
        "controller_max_ms": float(np.max(timing)),
        "lidar_valid_ratio": float(np.mean(valid_ratios or [0.0])),
        "localizer_failure_rate": float(
            localizer_failures / max(len(valid_ratios), 1)
        ),
        "minimum_map_clearance_m": float(
            min(row["map_clearance_m"] for row in rows)
        ),
        "map_yaml": str(Path(map_yaml).resolve()),
        "hardware_output_enabled": False,
    }
    if output_dir is not None:
        run_id = (
            f"{controller_name}_{condition_name}_{pose_source}_seed{int(seed)}"
        )
        write_rows(Path(output_dir) / "trajectories" / f"{run_id}.csv", rows)
    return summary, rows


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=CONTROLLER_RATES_HZ, default="mppi")
    parser.add_argument("--condition", choices=LIDAR_CONDITIONS, default="ideal")
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--map-yaml", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--vehicle-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--duration-s", type=float, default=4.0)
    parser.add_argument("--distance-m", type=float, default=0.45)
    parser.add_argument("--speed-mps", type=float, default=0.15)
    parser.add_argument(
        "--pose-source",
        choices=("ground_truth", "localized"),
        default="localized",
    )
    options = parser.parse_args(args)
    started = time.perf_counter()
    summary, _rows = run_map_case(
        options.controller,
        options.condition,
        options.seed,
        map_yaml=options.map_yaml,
        vehicle_config=options.vehicle_config,
        output_dir=options.output_dir,
        duration_s=options.duration_s,
        distance_m=options.distance_m,
        speed_mps=options.speed_mps,
        pose_source=options.pose_source,
    )
    summary["wall_time_s"] = time.perf_counter() - started
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
