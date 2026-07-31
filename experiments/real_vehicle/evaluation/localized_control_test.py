"""Connect virtual LiDAR localization output to the real-vehicle controller."""

import argparse
import json
from pathlib import Path

import numpy as np

from ..localization import DEFAULT_COMPLEX_OBSTACLES, GridSearchLocalizer, VirtualLidar
from ..models import ActuatorModel, step_model
from .common import DEFAULT_CONFIG, load_real_config, run_simulation, straight_reference, write_rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=("mpc", "mppi"), default="mppi")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=Path("experiments/real_vehicle/results/localized_control.csv"))
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--noise-std", type=float, default=None)
    parser.add_argument("--dropout-probability", type=float, default=None)
    parser.add_argument("--delay-s", type=float, default=None)
    parser.add_argument("--initial-x-error", type=float, default=0.0)
    parser.add_argument("--initial-y-error", type=float, default=0.0)
    parser.add_argument("--initial-heading-error", type=float, default=0.0)
    parser.add_argument("--complex-map", action="store_true")
    args = parser.parse_args()
    config, parameters, limits = load_real_config(args.config)
    dt = float(config["simulation"]["timestep_s"])
    controller_config = config["controller"]
    from ..evaluation.common import make_controller

    controller = make_controller(args.controller, parameters, limits, dt, controller_config)
    lidar_config = dict(config["lidar"])
    if args.noise_std is not None:
        lidar_config["noise_std_m"] = args.noise_std
    if args.dropout_probability is not None:
        lidar_config["dropout_probability"] = args.dropout_probability
    if args.delay_s is not None:
        lidar_config["delay_s"] = args.delay_s
    if args.complex_map:
        lidar_config["obstacles"] = DEFAULT_COMPLEX_OBSTACLES
    lidar = VirtualLidar(lidar_config, seed=7)
    localizer = GridSearchLocalizer(lidar)
    true_state = np.array([0.0, 0.0, 0.0, 0.0])
    actuator = ActuatorModel(
        parameters,
        limits,
        config["simulation"].get("actuator", {}),
        seed=int(controller_config.get("seed", 7)),
    )
    actuator.reset(true_state)
    estimated_state = true_state.copy()
    predicted_state = true_state.copy()
    predicted_state[:3] += np.array([
        args.initial_heading_error,
        args.initial_x_error,
        args.initial_y_error,
    ])
    speed = float(controller_config["speed_target_mps"])
    goal_x = float(config["simulation"]["straight_goal_x_m"])
    rows = []
    for index in range(args.steps):
        timestamp = index * dt
        scan = lidar.scan(true_state, timestamp)
        estimated_pose, match_score = localizer.estimate(scan, predicted_state[:3])
        estimated_state[:3] = estimated_pose
        reference = straight_reference(timestamp, controller.horizon, dt, speed, goal_x, y=0.0)
        control, info = controller.plan(estimated_state, reference)
        next_true, clamp, _actuator_log = actuator.step(
            true_state,
            control,
            parameters,
            dt,
            integrator=str(config["simulation"].get("integrator", "rk4")),
        )
        next_predicted, _ = step_model(predicted_state, control, parameters, limits, dt)
        rows.append({
            "timestamp_s": timestamp,
            "true_x_m": true_state[1],
            "true_y_m": true_state[2],
            "true_phi_rad": true_state[0],
            "estimated_x_m": estimated_state[1],
            "estimated_y_m": estimated_state[2],
            "estimated_phi_rad": estimated_state[0],
            "xy_error_m": float(np.linalg.norm(true_state[1:3] - estimated_state[1:3])),
            "heading_error_rad": float((true_state[0] - estimated_state[0] + np.pi) % (2 * np.pi) - np.pi),
            "match_score": match_score,
            "scan_age_s": timestamp - float(scan["header"]["stamp_s"]),
            "control_speed_mps": control[0],
            "control_steer_rate_rad_s": control[1],
            "controller_time_ms": 1000.0 * info["solve_time_s"],
            "clamped_speed": int(clamp.clamped_speed),
            "clamped_steer": int(clamp.clamped_steer),
        })
        true_state = next_true
        predicted_state = next_predicted
        estimated_state[3] = predicted_state[3]
    write_rows(args.output, rows)
    summary = {
        "controller": args.controller,
        "steps": len(rows),
        "xy_rmse_m": float(np.sqrt(np.mean(np.square([row["xy_error_m"] for row in rows])))),
        "heading_rmse_rad": float(np.sqrt(np.mean(np.square([row["heading_error_rad"] for row in rows])))),
        "mean_controller_time_ms": float(np.mean([row["controller_time_ms"] for row in rows])),
        "mean_scan_age_s": float(np.mean([row["scan_age_s"] for row in rows])),
    }
    args.output.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"csv: {args.output}")


if __name__ == "__main__":
    main()
