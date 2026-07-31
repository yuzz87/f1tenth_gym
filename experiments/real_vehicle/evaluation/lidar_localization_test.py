"""Evaluate virtual LaserScan fields and known-map localization."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from ..localization import DEFAULT_COMPLEX_OBSTACLES, GridSearchLocalizer, VirtualLidar
from ..models import ActuatorModel, step_model
from .common import DEFAULT_CONFIG, load_real_config, write_rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=Path("experiments/real_vehicle/results/lidar_localization.csv"))
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
    dt = float(config["simulation"]["timestep_s"])
    state = np.array([0.0, 0.0, 0.0, 0.0])
    actuator = ActuatorModel(
        parameters,
        limits,
        config["simulation"].get("actuator", {}),
        seed=int(config["controller"].get("seed", 7)),
    )
    actuator.reset(state)
    predicted = state[:3].copy() + np.array([
        args.initial_heading_error,
        args.initial_x_error,
        args.initial_y_error,
    ])
    rows = []
    controls = np.array([0.20, 0.0])
    for index in range(args.steps):
        timestamp = index * dt
        scan = lidar.scan(state, timestamp)
        estimate, score = localizer.estimate(scan, predicted)
        rows.append({
            "timestamp_s": timestamp,
            "true_phi_rad": state[0],
            "true_x_m": state[1],
            "true_y_m": state[2],
            "estimated_phi_rad": estimate[0],
            "estimated_x_m": estimate[1],
            "estimated_y_m": estimate[2],
            "xy_error_m": float(np.linalg.norm(state[1:3] - estimate[1:3])),
            "heading_error_rad": float((state[0] - estimate[0] + np.pi) % (2 * np.pi) - np.pi),
            "scan_valid_ratio": float(np.mean(np.asarray(scan["ranges"]) < lidar.range_max_m)),
            "scan_age_s": timestamp - float(scan["header"]["stamp_s"]),
            "match_score": score,
        })
        predicted = estimate.copy()
        predicted[1] += controls[0] * np.cos(predicted[0]) * dt
        predicted[2] += controls[0] * np.sin(predicted[0]) * dt
        state, _, _ = actuator.step(
            state,
            controls,
            parameters,
            dt,
            integrator=str(config["simulation"].get("integrator", "rk4")),
        )
    write_rows(args.output, rows)
    summary = {
        "steps": len(rows),
        "xy_rmse_m": float(np.sqrt(np.mean(np.square([row["xy_error_m"] for row in rows])))),
        "heading_rmse_rad": float(np.sqrt(np.mean(np.square([row["heading_error_rad"] for row in rows])))),
        "mean_scan_valid_ratio": float(np.mean([row["scan_valid_ratio"] for row in rows])),
        "mean_scan_age_s": float(np.mean([row["scan_age_s"] for row in rows])),
    }
    args.output.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"csv: {args.output}")


if __name__ == "__main__":
    main()
