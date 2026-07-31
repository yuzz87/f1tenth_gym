"""LiDAR map localizerのビーム数・探索候補数を比較する。"""

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
from experiments.localization import create_localizer  # noqa: E402
from experiments.run_controller_experiment import (  # noqa: E402
    get_integrator,
    resolve_repo_path,
)
from experiments.run_localized_controller_experiment import (  # noqa: E402
    build_estimated_obs,
)


SWEEP_CONFIGS = {
    "baseline": {},
    "reduced_beams": {
        "scan_beams": 90,
        "preserve_scan_angles": False,
    },
    "reduced_candidates": {
        "xy_candidates": 3,
        "theta_candidates": 5,
    },
    "fast_combined": {
        "scan_beams": 90,
        "preserve_scan_angles": False,
        "xy_candidates": 3,
        "theta_candidates": 5,
    },
}


def load_config(config_path):
    with open(config_path, encoding="utf-8") as file:
        config = yaml.safe_load(file)
    config["map_path"] = str(resolve_repo_path(config["map_path"]))
    config["wpt_path"] = str(resolve_repo_path(config["wpt_path"]))
    return config


def normalize_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def run_condition(base_config, condition_name, overrides, max_steps, lap_target, seed):
    config = copy.deepcopy(base_config)
    config["run_name"] = f"{base_config['run_name']}_{condition_name}"
    localizer_config = config.setdefault("localizer", {})
    localizer_config.update(overrides)
    config["controller_type"] = "mppi"
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
        lidar_config=conf.lidar,
        seed=seed,
    )

    try:
        localizer = create_localizer(conf, env.sim.agents[0].scan_angles)
        obs, _, done, _ = env.reset(
            np.array([[conf.sx, conf.sy, conf.stheta]])
        )
        est_pose = localizer.initialize(
            np.array([conf.sx, conf.sy, conf.stheta], dtype=float)
        )
        control_obs = build_estimated_obs(obs, est_pose)
        localizer_times = []
        controller_times = []
        xy_errors = []
        theta_errors = []
        collisions = 0
        step_count = 0
        lap_target_reached = False

        while not done:
            plan_start = time.perf_counter()
            speed, steer = controller.plan(control_obs)
            controller_times.append(time.perf_counter() - plan_start)
            obs, _, done, _ = env.step(np.array([[steer, speed]]))
            step_count += 1

            localizer_start = time.perf_counter()
            est_pose = localizer.update(
                obs,
                control={"speed_cmd": speed, "steer_cmd": steer},
            )
            localizer_times.append(time.perf_counter() - localizer_start)
            control_obs = build_estimated_obs(obs, est_pose)

            true_pose = np.array(
                [obs["poses_x"][0], obs["poses_y"][0], obs["poses_theta"][0]],
                dtype=float,
            )
            xy_errors.append(float(np.linalg.norm(est_pose[:2] - true_pose[:2])))
            theta_errors.append(
                abs(float(normalize_angle(est_pose[2] - true_pose[2])))
            )
            collisions += int(obs["collisions"][0])

            if lap_target > 0 and int(obs["lap_counts"][0]) >= lap_target:
                lap_target_reached = True
                break
            if max_steps > 0 and step_count >= max_steps:
                break

        localizer_info = localizer.debug_info()
        localizer_times = np.asarray(localizer_times, dtype=float)
        controller_times = np.asarray(controller_times, dtype=float)
        xy_errors = np.asarray(xy_errors, dtype=float)
        theta_errors = np.asarray(theta_errors, dtype=float)
        return {
            "condition": condition_name,
            "scan_beams": int(localizer_info["scan_beams"]),
            "candidate_count": int(localizer_info["candidate_count"]),
            "preserve_scan_angles": int(localizer_info["preserve_scan_angles"]),
            "row_count": step_count,
            "sim_time_s": float(step_count * conf.timestep),
            "final_lap_count": int(obs["lap_counts"][0]),
            "lap_target_reached": int(lap_target_reached),
            "mean_localizer_time_ms": float(np.mean(localizer_times) * 1000.0),
            "p95_localizer_time_ms": float(np.percentile(localizer_times, 95) * 1000.0),
            "max_localizer_time_ms": float(np.max(localizer_times) * 1000.0),
            "localizer_overrun_count": int(
                np.sum(localizer_times > float(conf.timestep))
            ),
            "mean_controller_time_ms": float(np.mean(controller_times) * 1000.0),
            "mean_est_xy_error_m": float(np.mean(xy_errors)),
            "max_est_xy_error_m": float(np.max(xy_errors)),
            "mean_est_theta_error_rad": float(np.mean(theta_errors)),
            "collision_count": collisions,
        }
    finally:
        del env


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
        )
        for condition_name, overrides in SWEEP_CONFIGS.items()
    ]

    summary_path = output_dir / "localizer_timing_comparison.csv"
    fieldnames = list(results[0])
    with summary_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    for result in results:
        print(
            f"{result['condition']}: beams={result['scan_beams']}, "
            f"candidates={result['candidate_count']}, "
            f"mean={result['mean_localizer_time_ms']:.3f} ms, "
            f"p95={result['p95_localizer_time_ms']:.3f} ms, "
            f"xy_error={result['mean_est_xy_error_m']:.4f} m"
        )
    print(f"summary_csv: {summary_path}")
    return results, summary_path


def main():
    parser = argparse.ArgumentParser(description="LiDAR map localizer速度比較")
    parser.add_argument(
        "--config",
        default="experiments/configs/homur_oval_localized_a1_mppi_step7.yaml",
        help="比較のベース設定ファイル",
    )
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--lap-target", type=int, default=0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/localizer_timing",
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
