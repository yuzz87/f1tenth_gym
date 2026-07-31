"""真値姿勢で高速MPPIの速度とホライズンを比較する。"""

import argparse
import copy
import csv
from argparse import Namespace
from pathlib import Path
import sys
import time

import gym
import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.controllers import create_controller  # noqa: E402
from experiments.run_controller_experiment import (  # noqa: E402
    RunLogger,
    get_integrator,
    resolve_repo_path,
)


HORIZON_CONDITIONS = {
    "horizon_12": {"horizon": 12, "num_samples": 128},
    "horizon_16": {"horizon": 16, "num_samples": 128},
    "horizon_24": {"horizon": 24, "num_samples": 128},
}


def load_config(config_path):
    with open(config_path, encoding="utf-8") as file:
        config = yaml.safe_load(file)
    config["map_path"] = str(resolve_repo_path(config["map_path"]))
    config["wpt_path"] = str(resolve_repo_path(config["wpt_path"]))
    return config


def build_condition_config(base_config, horizon_name, horizon_overrides, speed):
    config = copy.deepcopy(base_config)
    config["run_name"] = (
        f"{base_config['run_name']}_truth_{horizon_name}_v{speed:.1f}"
    )
    config["controller_type"] = "mppi"
    config["controller"].update(horizon_overrides)
    config["controller"]["target_speed"] = float(speed)
    config["controller"]["vgain"] = float(speed)
    config.pop("localizer", None)
    config.pop("lidar", None)
    return config


def run_condition(
    base_config,
    horizon_name,
    horizon_overrides,
    speed,
    max_steps,
    lap_target,
    seed,
    output_dir,
    controller_overrides=None,
):
    config = build_condition_config(
        base_config, horizon_name, horizon_overrides, speed
    )
    if controller_overrides:
        config["controller"].update(controller_overrides)
    conf = Namespace(**config)
    controller = create_controller(conf, conf.car_params)
    env = gym.make(
        "f110_gym:f110-v0",
        map=conf.map_path,
        map_ext=conf.map_ext,
        num_agents=conf.num_agents,
        timestep=conf.timestep,
        integrator=get_integrator(conf.integrator),
        params=conf.car_params,
        lidar_dist=conf.lidar_dist,
        lidar_config=None,
        seed=seed,
    )

    try:
        obs, step_reward, done, info = env.reset(
            np.array([[conf.sx, conf.sy, conf.stheta]])
        )
        del step_reward, info
        condition_name = f"{horizon_name}_v{speed:.1f}"
        logger = RunLogger(
            controller,
            condition_name,
            conf.run_name,
            output_dir,
        )
        logger.record(0, 0.0, obs, 0.0, 0.0, done, plan_time_s=0.0)

        sim_elapsed_time = 0.0
        step_count = 0
        lap_target_reached = False
        start = time.perf_counter()
        while not done:
            plan_start = time.perf_counter()
            command_speed, steer = controller.plan(obs)
            plan_time_s = time.perf_counter() - plan_start
            obs, step_reward, done, info = env.step(
                np.array([[steer, command_speed]])
            )
            del info
            sim_elapsed_time += step_reward
            step_count += 1
            logger.record(
                step_count,
                sim_elapsed_time,
                obs,
                command_speed,
                steer,
                done,
                plan_time_s=plan_time_s,
            )

            if lap_target > 0 and int(obs["lap_counts"][0]) >= lap_target:
                lap_target_reached = True
                break
            if max_steps > 0 and step_count >= max_steps:
                break

        rows = logger.rows
        plan_times = np.asarray(
            [float(row["plan_time_s"]) for row in rows[1:]], dtype=float
        )
        if plan_times.size == 0:
            plan_times = np.zeros(1, dtype=float)
        cross_track = np.asarray(
            [float(row["cross_track_error"]) for row in rows], dtype=float
        )
        speeds = np.asarray(
            [float(row["linear_vel_x"]) for row in rows], dtype=float
        )
        collisions = np.asarray(
            [int(row["collision"]) for row in rows], dtype=int
        )
        log_path = logger.write()
        return {
            "horizon_condition": horizon_name,
            "horizon": int(controller.horizon),
            "num_samples": int(controller.num_samples),
            "target_speed_mps": float(speed),
            "controller_target_speed_mps": float(controller.speed_target),
            "seed": int(seed),
            "row_count": len(rows),
            "sim_time_s": float(rows[-1]["sim_time"]),
            "lap_target": int(lap_target),
            "final_lap_count": int(rows[-1]["lap_count"]),
            "lap_target_reached": int(lap_target_reached),
            "mean_speed_mps": float(np.mean(speeds)),
            "mean_abs_cross_track_m": float(np.mean(np.abs(cross_track))),
            "max_abs_cross_track_m": float(np.max(np.abs(cross_track))),
            "collision_count": int(np.sum(collisions)),
            "out_of_course_count": int(np.sum(np.abs(cross_track) > 0.5)),
            "mean_plan_time_ms": float(np.mean(plan_times) * 1000.0),
            "p95_plan_time_ms": float(np.percentile(plan_times, 95) * 1000.0),
            "max_plan_time_ms": float(np.max(plan_times) * 1000.0),
            "control_overrun_count": int(
                np.sum(plan_times > float(conf.timestep))
            ),
            "real_elapsed_time_s": float(time.perf_counter() - start),
            "log_csv": str(log_path),
        }
    finally:
        del env


def run_experiment(config_path, speeds, max_steps, lap_target, seed, output_dir):
    base_config = load_config(config_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for horizon_name, horizon_overrides in HORIZON_CONDITIONS.items():
        for speed in speeds:
            result = run_condition(
                base_config,
                horizon_name,
                horizon_overrides,
                speed,
                max_steps,
                lap_target,
                seed,
                output_dir,
            )
            results.append(result)
            print(
                f"{horizon_name}: speed={speed:.1f}, "
                f"steps={result['row_count']}, "
                f"laps={result['final_lap_count']}, "
                f"mean_plan={result['mean_plan_time_ms']:.3f} ms, "
                f"collision={result['collision_count']}"
            )

    summary_path = output_dir / "high_speed_mppi_truth_comparison.csv"
    fieldnames = list(results[0])
    with summary_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    print(f"summary_csv: {summary_path}")
    return results, summary_path


def parse_speeds(text):
    return [float(value.strip()) for value in text.split(",") if value.strip()]


def main():
    parser = argparse.ArgumentParser(description="高速真値姿勢MPPI比較")
    parser.add_argument(
        "--config",
        default="experiments/configs/homur_oval_localized_a1_mppi_step8_fast.yaml",
        help="MPPI設定のベースファイル",
    )
    parser.add_argument("--speeds", default="3.0,5.0,6.1")
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--lap-target", type=int, default=0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/high_speed_mppi_truth",
    )
    args = parser.parse_args()
    run_experiment(
        resolve_repo_path(args.config),
        parse_speeds(args.speeds),
        args.max_steps,
        args.lap_target,
        args.seed,
        resolve_repo_path(args.output_dir),
    )


if __name__ == "__main__":
    main()
