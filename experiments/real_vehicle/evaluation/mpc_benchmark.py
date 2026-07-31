"""Compare full-input and control-blocked MPC solve times."""

import argparse
import json
from pathlib import Path

import numpy as np

from ..controllers import RealVehicleMPC
from .common import DEFAULT_CONFIG, load_real_config, straight_reference


def measure(controller, iterations, timestep_s):
    state = np.zeros(4, dtype=float)
    times_ms = []
    successes = 0
    deadline_exceeded = 0
    for index in range(iterations):
        reference = straight_reference(
            index * timestep_s,
            controller.horizon,
            timestep_s,
            0.30,
            4.0,
            y=0.0,
        )
        _control, info = controller.plan(state, reference)
        times_ms.append(1000.0 * float(info["solve_time_s"]))
        successes += int(info["success"])
        deadline_exceeded += int(info["deadline_exceeded"])
    return {
        "control_blocks": controller.control_blocks,
        "optimizer_variables": 2 * controller.control_blocks,
        "mean_ms": float(np.mean(times_ms)),
        "p95_ms": float(np.percentile(times_ms, 95)),
        "max_ms": float(np.max(times_ms)),
        "within_20ms_rate": float(np.mean(np.asarray(times_ms) <= 20.0)),
        "optimizer_success_rate": float(successes / iterations),
        "deadline_exceeded_rate": float(deadline_exceeded / iterations),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--iterations", type=int, default=50)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    config, parameters, limits = load_real_config(args.config)
    timestep_s = 1.0 / 50.0
    iterations = max(args.iterations, 1)
    common = dict(config["controller"], mpc_deadline_s=0.0)
    numerical_full_config = dict(
        common,
        mpc_control_blocks=config["controller"]["horizon"],
        mpc_use_analytic_gradient=False,
        mpc_max_function_evaluations=1000,
    )
    analytic_full_config = dict(
        common,
        mpc_control_blocks=config["controller"]["horizon"],
        mpc_use_analytic_gradient=True,
    )
    blocked_config = dict(common, mpc_use_analytic_gradient=True)
    projected_config = dict(
        blocked_config,
        mpc_solver="projected_gradient",
        max_iterations=8,
        mpc_max_function_evaluations=16,
    )
    result = {
        "iterations": iterations,
        "control_period_ms": 20.0,
        "numerical_full": measure(
            RealVehicleMPC(parameters, limits, timestep_s, numerical_full_config),
            iterations,
            timestep_s,
        ),
        "analytic_full": measure(
            RealVehicleMPC(parameters, limits, timestep_s, analytic_full_config),
            iterations,
            timestep_s,
        ),
        "analytic_blocked": measure(
            RealVehicleMPC(parameters, limits, timestep_s, blocked_config),
            iterations,
            timestep_s,
        ),
        "projected_gradient": measure(
            RealVehicleMPC(parameters, limits, timestep_s, projected_config),
            iterations,
            timestep_s,
        ),
    }
    result["speedup"] = (
        result["numerical_full"]["mean_ms"]
        / result["analytic_blocked"]["mean_ms"]
    )
    output = json.dumps(result, indent=2)
    print(output)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
