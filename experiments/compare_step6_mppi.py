"""Step 6の真値姿勢MPPIパラメータ比較と計算時間計測。"""

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


CONDITIONS = {
    "M0_baseline": {},
    "M1_short_horizon": {
        "horizon": 8,
        "num_samples": 64,
    },
    "M2_long_horizon": {
        "horizon": 16,
        "num_samples": 128,
    },
    "M3_more_samples": {
        "horizon": 12,
        "num_samples": 256,
    },
}


def load_config(config_path):
    with open(config_path, encoding="utf-8") as file:
        config = yaml.safe_load(file)
    config["map_path"] = str(resolve_repo_path(config["map_path"]))
    config["wpt_path"] = str(resolve_repo_path(config["wpt_path"]))
    return config


def build_condition_config(base_config, condition_name, overrides):
    config = copy.deepcopy(base_config)
    config["run_name"] = f"{base_config['run_name']}_{condition_name}"
    config["controller_type"] = "mppi"
    config["controller"].update(overrides)
    # Step 6はLiDAR/localizerを使わず、真値姿勢だけで評価する。
    config.pop("localizer", None)
    config.pop("lidar", None)
    return config


def summarize_rows(rows, condition_name, controller):
    if not rows:
        raise ValueError("condition produced no log rows")

    cross_track = np.asarray([float(row["cross_track_error"]) for row in rows])
    speed = np.asarray([float(row["linear_vel_x"]) for row in rows])
    steer = np.asarray([float(row["steer_cmd"]) for row in rows])
    plan_time = np.asarray([float(row["plan_time_s"]) for row in rows[1:]])
    if plan_time.size == 0:
        plan_time = np.zeros(1, dtype=float)
    steer_change = np.diff(steer)
    collisions = np.asarray([int(row["collision"]) for row in rows])
    timestep = float(controller.timestep)
    return {
        "condition": condition_name,
        "horizon": int(controller.horizon),
        "num_samples": int(controller.num_samples),
        "temperature": float(controller.temperature),
        "noise_sigma": float(controller.noise_sigma),
        "target_speed_mps": float(controller.speed_target),
        "rollout_model": getattr(controller, "rollout_model", "unknown"),
        "row_count": len(rows),
        "sim_time_s": float(rows[-1]["sim_time"]),
        "final_lap_count": int(rows[-1]["lap_count"]),
        "final_lap_time_s": float(rows[-1]["lap_time"]),
        "mean_abs_cross_track_m": float(np.mean(np.abs(cross_track))),
        "max_abs_cross_track_m": float(np.max(np.abs(cross_track))),
        "mean_speed_mps": float(np.mean(speed)),
        "total_abs_steer_change_rad": float(np.sum(np.abs(steer_change))),
        "collision_count": int(np.sum(collisions)),
        "out_of_course_count": int(np.sum(np.abs(cross_track) > 0.5)),
        "mean_plan_time_s": float(np.mean(plan_time)),
        "p95_plan_time_s": float(np.percentile(plan_time, 95)),
        "max_plan_time_s": float(np.max(plan_time)),
        "mean_plan_time_ratio": float(np.mean(plan_time) / timestep),
        "control_overrun_count": int(np.sum(plan_time > timestep)),
    }


def run_condition(base_config, condition_name, overrides, max_steps, lap_target, seed, output_dir):
    config = build_condition_config(base_config, condition_name, overrides)
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

    obs, step_reward, done, info = env.reset(
        np.array([[conf.sx, conf.sy, conf.stheta]])
    )
    del step_reward, info
    logger = RunLogger(controller, condition_name, conf.run_name, output_dir)
    logger.record(0, 0.0, obs, 0.0, 0.0, done, plan_time_s=0.0)

    sim_elapsed_time = 0.0
    step_count = 0
    lap_target_reached = False
    start = time.perf_counter()
    while not done:
        plan_start = time.perf_counter()
        speed, steer = controller.plan(obs)
        plan_time_s = time.perf_counter() - plan_start
        obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
        del info
        sim_elapsed_time += step_reward
        step_count += 1
        logger.record(
            step_count,
            sim_elapsed_time,
            obs,
            speed,
            steer,
            done,
            plan_time_s=plan_time_s,
        )

        if lap_target > 0 and int(obs["lap_counts"][0]) >= lap_target:
            lap_target_reached = True
            break
        if max_steps > 0 and step_count >= max_steps:
            break

    log_path = logger.write()
    metrics = summarize_rows(logger.rows, condition_name, controller)
    metrics.update(
        {
            "lap_target_reached": int(lap_target_reached),
            "log_csv": str(log_path),
            "real_elapsed_time_s": float(time.perf_counter() - start),
        }
    )
    print(
        f"{condition_name}: steps={step_count}, "
        f"laps={metrics['final_lap_count']}, "
        f"mean_plan={metrics['mean_plan_time_s'] * 1000.0:.3f} ms, "
        f"mean_abs_cross_track={metrics['mean_abs_cross_track_m']:.6f}"
    )
    return metrics


def run_experiment(config_path, max_steps, lap_target, seed, output_dir):
    base_config = load_config(config_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    results = [
        run_condition(
            base_config,
            condition_name,
            overrides,
            max_steps,
            lap_target,
            seed,
            output_dir,
        )
        for condition_name, overrides in CONDITIONS.items()
    ]

    summary_path = output_dir / "step6_mppi_comparison.csv"
    fieldnames = [
        "condition",
        "horizon",
        "num_samples",
        "temperature",
        "noise_sigma",
        "target_speed_mps",
        "rollout_model",
        "row_count",
        "sim_time_s",
        "final_lap_count",
        "final_lap_time_s",
        "lap_target_reached",
        "mean_abs_cross_track_m",
        "max_abs_cross_track_m",
        "mean_speed_mps",
        "total_abs_steer_change_rad",
        "collision_count",
        "out_of_course_count",
        "mean_plan_time_s",
        "p95_plan_time_s",
        "max_plan_time_s",
        "mean_plan_time_ratio",
        "control_overrun_count",
        "real_elapsed_time_s",
        "log_csv",
    ]
    with summary_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    print(f"summary_csv: {summary_path}")
    return results, summary_path


def main():
    parser = argparse.ArgumentParser(description="Step 6 真値姿勢MPPI比較")
    parser.add_argument(
        "--config",
        default="experiments/configs/homur_oval_mppi.yaml",
        help="共通で使うMPPI設定ファイル",
    )
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--lap-target", type=int, default=0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/step6_mppi",
    )
    args = parser.parse_args()
    run_experiment(
        resolve_repo_path(args.config),
        args.max_steps,
        args.lap_target,
        args.seed,
        resolve_repo_path(args.output_dir),
    )


if __name__ == "__main__":
    main()
