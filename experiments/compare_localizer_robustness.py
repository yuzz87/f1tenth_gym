"""高速localizerを複数seed・速度・LiDAR条件で再評価する。"""

import argparse
import copy
import csv
from argparse import Namespace
from collections import defaultdict
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


LIDAR_CONDITIONS = {
    "ideal": {
        "noise_std": 0.0,
        "noise_std_per_meter": 0.0,
        "dropout_probability": 0.0,
        "scan_rate_hz": 5.5,
        "scan_delay": 0.0,
    },
    "noise": {
        "noise_std": 0.01,
        "noise_std_per_meter": 0.002,
        "dropout_probability": 0.0,
        "scan_rate_hz": 5.5,
        "scan_delay": 0.0,
    },
    "noise_delay_dropout": {
        "noise_std": 0.01,
        "noise_std_per_meter": 0.002,
        "dropout_probability": 0.03,
        "scan_rate_hz": 5.5,
        "scan_delay": 0.05,
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


def run_condition(
    base_config,
    lidar_condition_name,
    lidar_overrides,
    speed_target,
    seed,
    max_steps,
    lap_target,
):
    config = copy.deepcopy(base_config)
    config["run_name"] = (
        f"{base_config['run_name']}_{lidar_condition_name}_"
        f"v{speed_target:.1f}_seed{seed}"
    )
    config["controller_type"] = "mppi"

    # 速度とseedを実験条件として明示する。
    config["controller"] = copy.deepcopy(config["controller"])
    config["controller"]["target_speed"] = float(speed_target)
    config["controller"]["vgain"] = float(speed_target)
    config["controller"]["seed"] = int(seed)

    config["lidar"] = copy.deepcopy(config.get("lidar") or {})
    config["lidar"].update(lidar_overrides)
    config["lidar"]["profile"] = "rplidar_a1m8_r6"
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
        speeds = []
        collision_events = 0
        previous_collision = 0
        step_count = 0
        lap_target_reached = False
        start = time.perf_counter()

        while not done:
            plan_start = time.perf_counter()
            speed, steer = controller.plan(control_obs)
            controller_times.append(time.perf_counter() - plan_start)
            speeds.append(float(speed))
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
            current_collision = int(obs["collisions"][0])
            if current_collision and not previous_collision:
                collision_events += 1
            previous_collision = current_collision

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
            "lidar_condition": lidar_condition_name,
            "speed_target_mps": float(speed_target),
            "seed": int(seed),
            "scan_beams": int(localizer_info["scan_beams"]),
            "candidate_count": int(localizer_info["candidate_count"]),
            "row_count": step_count,
            "sim_time_s": float(step_count * conf.timestep),
            "lap_target": int(lap_target),
            "final_lap_count": int(obs["lap_counts"][0]),
            "lap_target_reached": int(lap_target_reached),
            "mean_speed_mps": float(np.mean(speeds)),
            "mean_localizer_time_ms": float(np.mean(localizer_times) * 1000.0),
            "p95_localizer_time_ms": float(
                np.percentile(localizer_times, 95) * 1000.0
            ),
            "localizer_overrun_count": int(
                np.sum(localizer_times > float(conf.timestep))
            ),
            "mean_controller_time_ms": float(np.mean(controller_times) * 1000.0),
            "mean_est_xy_error_m": float(np.mean(xy_errors)),
            "max_est_xy_error_m": float(np.max(xy_errors)),
            "mean_est_theta_error_rad": float(np.mean(theta_errors)),
            "collision_count": collision_events,
            "real_elapsed_time_s": float(time.perf_counter() - start),
        }
    finally:
        del env


def aggregate_results(results):
    groups = defaultdict(list)
    for result in results:
        groups[
            (result["lidar_condition"], result["speed_target_mps"])
        ].append(result)

    aggregate_rows = []
    metrics = (
        "mean_localizer_time_ms",
        "p95_localizer_time_ms",
        "mean_speed_mps",
        "mean_est_xy_error_m",
        "max_est_xy_error_m",
        "mean_est_theta_error_rad",
        "localizer_overrun_count",
        "collision_count",
    )
    for (condition, speed), rows in groups.items():
        row = {
            "lidar_condition": condition,
            "speed_target_mps": speed,
            "seed_count": len(rows),
            "lap_target": int(rows[0]["lap_target"]),
            "lap_success_rate": "",
        }
        if row["lap_target"] > 0:
            row["lap_success_rate"] = float(
                np.mean([int(item["lap_target_reached"]) for item in rows])
            )
        for metric in metrics:
            values = np.asarray([float(item[metric]) for item in rows])
            row[f"{metric}_mean"] = float(np.mean(values))
            row[f"{metric}_std"] = float(np.std(values))
        aggregate_rows.append(row)
    return aggregate_rows


def run_experiment(config_path, seeds, speeds, max_steps, lap_target, output_dir):
    base_config = load_config(config_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for lidar_condition_name, lidar_overrides in LIDAR_CONDITIONS.items():
        for speed_target in speeds:
            for seed in seeds:
                result = run_condition(
                    base_config,
                    lidar_condition_name,
                    lidar_overrides,
                    speed_target,
                    seed,
                    max_steps,
                    lap_target,
                )
                results.append(result)
                print(
                    f"{lidar_condition_name}: speed={speed_target:.1f}, "
                    f"seed={seed}, steps={result['row_count']}, "
                    f"localizer={result['mean_localizer_time_ms']:.3f} ms, "
                    f"xy_error={result['mean_est_xy_error_m']:.4f} m"
                )

    raw_path = output_dir / "localizer_robustness_runs.csv"
    raw_fields = list(results[0])
    with raw_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=raw_fields)
        writer.writeheader()
        writer.writerows(results)

    aggregate_rows = aggregate_results(results)
    aggregate_path = output_dir / "localizer_robustness_summary.csv"
    aggregate_fields = list(aggregate_rows[0])
    with aggregate_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=aggregate_fields)
        writer.writeheader()
        writer.writerows(aggregate_rows)
    print(f"raw_csv: {raw_path}")
    print(f"summary_csv: {aggregate_path}")
    return results, aggregate_rows, raw_path, aggregate_path


def parse_csv_values(text, converter):
    return [converter(value.strip()) for value in text.split(",") if value.strip()]


def main():
    parser = argparse.ArgumentParser(description="LiDAR localizerの再現性比較")
    parser.add_argument(
        "--config",
        default="experiments/configs/homur_oval_localized_a1_mppi_step8_fast.yaml",
        help="高速localizer設定ファイル",
    )
    parser.add_argument("--seeds", default="123,456,789")
    parser.add_argument(
        "--speeds",
        default="1.0,2.0,3.0,4.0,5.0,5.5,6.1",
        help="カンマ区切りの目標速度[m/s]",
    )
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--lap-target", type=int, default=0)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/localizer_robustness",
    )
    args = parser.parse_args()
    run_experiment(
        resolve_repo_path(args.config),
        parse_csv_values(args.seeds, int),
        parse_csv_values(args.speeds, float),
        args.max_steps,
        args.lap_target,
        resolve_repo_path(args.output_dir),
    )


if __name__ == "__main__":
    main()
