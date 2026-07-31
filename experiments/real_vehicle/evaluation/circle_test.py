"""Run one 4 m-diameter circular reference lap."""

import argparse
import json
from pathlib import Path

import numpy as np

from .common import DEFAULT_CONFIG, circle_reference, load_real_config, run_simulation, write_rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=("mpc", "mppi"), default="mppi")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=Path("experiments/real_vehicle/results/circle.csv"))
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()
    config, parameters, limits = load_real_config(args.config)
    simulation = config["simulation"]
    speed = float(config["controller"]["speed_target_mps"])
    radius = float(simulation["circle_radius_m"])
    lap_time = 2.0 * np.pi * radius / max(speed, 1e-9)
    duration = args.duration or (lap_time + 1.0)
    reference_function = lambda t, h, _state, _reached: circle_reference(
        t, h, float(simulation["timestep_s"]), speed, radius, parameters
    )
    rows, summary = run_simulation(
        args.controller,
        reference_function,
        [np.pi / 2.0, radius, 0.0, np.arctan(parameters.wheelbase_m / radius)],
        duration,
        config,
        parameters,
        limits,
    )
    radius_errors = []
    for row in rows:
        radius_errors.append(abs(np.hypot(row["actual_x_m"], row["actual_y_m"]) - radius))
    summary["circle_radius_rmse_m"] = float(np.sqrt(np.mean(np.square(radius_errors))))
    summary["steer_within_limit"] = summary["steer_clamp_count"] == 0
    final_reference_phase = float(np.arctan2(rows[-1]["reference_y_m"], rows[-1]["reference_x_m"]))
    final_actual_phase = float(np.arctan2(rows[-1]["actual_y_m"], rows[-1]["actual_x_m"]))
    phase_error = (final_actual_phase - final_reference_phase + np.pi) % (2.0 * np.pi) - np.pi
    summary["final_phase_error_rad"] = phase_error
    summary["lap_completed"] = bool(abs(phase_error) < 0.5 and summary["circle_radius_rmse_m"] < 0.2)
    write_rows(args.output, rows)
    summary_path = args.output.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"csv: {args.output}")


if __name__ == "__main__":
    main()
