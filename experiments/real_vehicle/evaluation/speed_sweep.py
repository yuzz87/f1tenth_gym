"""Compare controllers across independent virtual model-speed conditions."""

import argparse
import copy
import csv
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from ..models import VehicleLimits
from .common import (
    DEFAULT_CONFIG,
    circle_reference,
    load_real_config,
    run_simulation,
    straight_reference,
)


DEFAULT_SPEEDS = (0.30, 1.00, 3.00, 6.10)


def _run_case(config, parameters, limits, controller, scenario, speed, duration=None):
    simulation = config["simulation"]
    config = copy.deepcopy(config)
    config["controller"]["speed_target_mps"] = float(speed)
    limits = replace(limits, speed_max_mps=max(float(limits.speed_max_mps), float(speed)))
    timestep = float(simulation["timestep_s"])
    radius = float(simulation["circle_radius_m"])
    goal_x = float(simulation["straight_goal_x_m"])
    if scenario == "straight":
        duration = duration or max(float(simulation["duration_s"]), goal_x / speed + 2.0)
        reference = lambda t, h, _state, reached: straight_reference(
            t, h, timestep, speed, goal_x, y=0.0, stop_at_goal=reached
        )
        initial_state = [0.0, 0.0, 0.0, 0.0]
        goal = lambda state, _time: state[1] >= goal_x - 0.05
    else:
        duration = duration or (2.0 * np.pi * radius / speed + 1.0)
        reference = lambda t, h, _state, _reached: circle_reference(
            t, h, timestep, speed, radius, parameters
        )
        initial_state = [
            np.pi / 2.0,
            radius,
            0.0,
            np.arctan(parameters.wheelbase_m / radius),
        ]
        goal = None
    rows, summary = run_simulation(
        controller,
        reference,
        initial_state,
        duration,
        config,
        parameters,
        limits,
        goal_function=goal,
    )
    summary.update({
        "scenario": scenario,
        "target_speed_mps": float(speed),
        "speed_limit_mps": float(limits.speed_max_mps),
    })
    if scenario == "circle" and rows:
        radius_errors = [
            abs(np.hypot(row["actual_x_m"], row["actual_y_m"]) - radius)
            for row in rows
        ]
        reference_phase = float(np.arctan2(rows[-1]["reference_y_m"], rows[-1]["reference_x_m"]))
        actual_phase = float(np.arctan2(rows[-1]["actual_y_m"], rows[-1]["actual_x_m"]))
        phase_error = (actual_phase - reference_phase + np.pi) % (2.0 * np.pi) - np.pi
        summary["circle_radius_rmse_m"] = float(np.sqrt(np.mean(np.square(radius_errors))))
        summary["final_phase_error_rad"] = float(phase_error)
        summary["lap_completed"] = bool(
            summary["sim_elapsed_time_s"] >= (2.0 * np.pi * radius / speed) - timestep
            and
            abs(phase_error) < 0.5 and summary["circle_radius_rmse_m"] < 0.2
        )
    return summary


def main():
    parser = argparse.ArgumentParser(description="Run MPC/MPPI speed range comparisons")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--speeds", default=",".join(str(value) for value in DEFAULT_SPEEDS))
    parser.add_argument("--controllers", default="mpc,mppi")
    parser.add_argument("--scenario", choices=("straight", "circle", "both"), default="straight")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--no-actuator", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=Path("experiments/real_vehicle/results/speed_sweep"))
    args = parser.parse_args()

    config, parameters, limits = load_real_config(args.config)
    if args.seed is not None:
        config["controller"]["seed"] = int(args.seed)
    if args.no_actuator:
        config["simulation"].setdefault("actuator", {})["enabled"] = False
    speeds = [float(value) for value in args.speeds.split(",") if value.strip()]
    controllers = [value.strip() for value in args.controllers.split(",") if value.strip()]
    scenarios = ("straight", "circle") if args.scenario == "both" else (args.scenario,)
    results = []
    for scenario in scenarios:
        for controller in controllers:
            for speed in speeds:
                summary = _run_case(
                    config,
                    parameters,
                    limits,
                    controller,
                    scenario,
                    speed,
                    duration=args.duration,
                )
                results.append(summary)
                print(
                    f"{scenario}: controller={controller}, speed={speed:.6f}, "
                    f"reached={summary['reached_goal']}, "
                    f"xy_rmse={summary['position_rmse_m']:.4f} m"
                    + (
                        f", lap={summary['lap_completed']}"
                        if scenario == "circle"
                        else ""
                    )
                )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.output_dir / "speed_sweep.csv"
    fields = sorted({key for result in results for key in result})
    with raw_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
    summary_path = args.output_dir / "speed_sweep.summary.json"
    summary_path.write_text(json.dumps({"results": results}, indent=2), encoding="utf-8")
    print(f"raw_csv: {raw_path}")
    print(f"summary_json: {summary_path}")


if __name__ == "__main__":
    main()
