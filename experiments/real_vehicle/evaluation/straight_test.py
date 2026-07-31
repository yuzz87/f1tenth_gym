"""Run the 4 m straight-line test for the real-vehicle controllers."""

import argparse
import json
from pathlib import Path

from .common import DEFAULT_CONFIG, load_real_config, run_simulation, straight_reference, write_rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=("mpc", "mppi"), default="mpc")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=Path("experiments/real_vehicle/results/straight.csv"))
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()
    config, parameters, limits = load_real_config(args.config)
    simulation = config["simulation"]
    speed = float(config["controller"]["speed_target_mps"])
    goal_x = float(simulation["straight_goal_x_m"])
    duration = args.duration or max(float(simulation["duration_s"]), goal_x / max(speed, 1e-9) + 2.0)
    reference_function = lambda t, h, _state, reached: straight_reference(
        t, h, float(simulation["timestep_s"]), speed, goal_x, stop_at_goal=reached
    )
    rows, summary = run_simulation(
        args.controller,
        reference_function,
        [0.0, 0.0, -2.0, 0.0],
        duration,
        config,
        parameters,
        limits,
        goal_function=lambda state, _time: state[1] >= goal_x - 0.05,
    )
    write_rows(args.output, rows)
    summary_path = args.output.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"csv: {args.output}")


if __name__ == "__main__":
    main()
