"""Step 5のPure Pursuit姿勢入力とLiDAR条件を比較する。"""

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

from experiments.localization import create_localizer  # noqa: E402
from experiments.run_controller_experiment import (  # noqa: E402
    RunLogger,
    get_integrator,
    resolve_repo_path,
)
from experiments.run_localized_controller_experiment import (  # noqa: E402
    LocalizedRunLogger,
    build_estimated_obs,
)
from experiments.controllers import create_controller  # noqa: E402


CONDITIONS = {
    "A_truth_no_lidar": {
        "mode": "truth",
        "lidar": None,
    },
    "B_estimated_ideal": {
        "mode": "estimated",
        "lidar": {
            "profile": "rplidar_a1m8_r6",
            "noise_std": 0.0,
            "noise_std_per_meter": 0.0,
            "dropout_probability": 0.0,
            "scan_rate_hz": 5.5,
            "scan_delay": 0.0,
        },
    },
    "C_estimated_noise": {
        "mode": "estimated",
        "lidar": {
            "profile": "rplidar_a1m8_r6",
            "noise_std": 0.01,
            "noise_std_per_meter": 0.002,
            "dropout_probability": 0.0,
            "scan_rate_hz": 5.5,
            "scan_delay": 0.0,
        },
    },
    "D_estimated_noise_delay": {
        "mode": "estimated",
        "lidar": {
            "profile": "rplidar_a1m8_r6",
            "noise_std": 0.01,
            "noise_std_per_meter": 0.002,
            "dropout_probability": 0.0,
            "scan_rate_hz": 5.5,
            "scan_delay": 0.05,
        },
    },
}


def load_config(config_path):
    with open(config_path, encoding="utf-8") as file:
        config = yaml.safe_load(file)
    config["map_path"] = str(resolve_repo_path(config["map_path"]))
    config["wpt_path"] = str(resolve_repo_path(config["wpt_path"]))
    return config


def build_condition_config(base_config, condition_name, condition):
    config = copy.deepcopy(base_config)
    config["run_name"] = f"{base_config['run_name']}_{condition_name}"
    config["lidar"] = copy.deepcopy(condition["lidar"])
    return config


def make_metrics(rows, mode):
    if not rows:
        raise ValueError("condition produced no log rows")

    cross_track = np.asarray([float(row["cross_track_error"]) for row in rows])
    speed = np.asarray([float(row["linear_vel_x"]) for row in rows])
    steer = np.asarray([float(row["steer_cmd"]) for row in rows])
    steer_change = np.diff(steer)
    collisions = np.asarray([int(row["collision"]) for row in rows])
    result = {
        "mean_abs_cross_track_m": float(np.mean(np.abs(cross_track))),
        "max_abs_cross_track_m": float(np.max(np.abs(cross_track))),
        "mean_speed_mps": float(np.mean(speed)),
        "total_abs_steer_change_rad": float(np.sum(np.abs(steer_change))),
        "collision_count": int(np.sum(collisions)),
        "final_lap_count": int(rows[-1]["lap_count"]),
        "final_lap_time_s": float(rows[-1]["lap_time"]),
        "sim_time_s": float(rows[-1]["sim_time"]),
        "row_count": len(rows),
    }
    if mode == "estimated":
        xy_error = np.asarray(
            [
                np.hypot(float(row["est_error_x"]), float(row["est_error_y"]))
                for row in rows
            ]
        )
        theta_error = np.asarray(
            [abs(float(row["est_error_theta"])) for row in rows]
        )
        result.update(
            {
                "mean_est_xy_error_m": float(np.mean(xy_error)),
                "max_est_xy_error_m": float(np.max(xy_error)),
                "mean_est_theta_error_rad": float(np.mean(theta_error)),
                "max_est_theta_error_rad": float(np.max(theta_error)),
            }
        )
    else:
        result.update(
            {
                "mean_est_xy_error_m": 0.0,
                "max_est_xy_error_m": 0.0,
                "mean_est_theta_error_rad": 0.0,
                "max_est_theta_error_rad": 0.0,
            }
        )
    return result


def run_condition(base_config, condition_name, condition, max_steps, lap_target, seed, output_dir):
    config = build_condition_config(base_config, condition_name, condition)
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

    localizer = None
    if condition["mode"] == "estimated":
        scan_angles = env.sim.agents[0].scan_angles
        localizer = create_localizer(conf, scan_angles)

    obs, step_reward, done, info = env.reset(
        np.array([[conf.sx, conf.sy, conf.stheta]])
    )
    del step_reward, info
    control_obs = obs
    est_pose = None
    debug_info = {}
    if localizer is not None:
        est_pose = localizer.initialize(
            np.array([conf.sx, conf.sy, conf.stheta], dtype=float)
        )
        control_obs = build_estimated_obs(obs, est_pose)
        debug_info = localizer.debug_info()

    config_name = condition_name
    if localizer is None:
        logger = RunLogger(controller, config_name, conf.run_name, output_dir)
    else:
        logger = LocalizedRunLogger(controller, config_name, conf.run_name, output_dir)

    if localizer is None:
        logger.record(0, 0.0, obs, 0.0, 0.0, done)
    else:
        logger.record(0, 0.0, obs, est_pose, debug_info, 0.0, 0.0, done)

    sim_elapsed_time = 0.0
    step_count = 0
    lap_target_reached = False
    start = time.time()
    while not done:
        speed, steer = controller.plan(control_obs)
        obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
        del info
        sim_elapsed_time += step_reward
        step_count += 1

        if localizer is not None:
            est_pose = localizer.update(
                obs,
                control={"speed_cmd": speed, "steer_cmd": steer},
            )
            control_obs = build_estimated_obs(obs, est_pose)
            debug_info = localizer.debug_info()
            logger.record(
                step_count,
                sim_elapsed_time,
                obs,
                est_pose,
                debug_info,
                speed,
                steer,
                done,
            )
        else:
            control_obs = obs
            logger.record(step_count, sim_elapsed_time, obs, speed, steer, done)

        if lap_target > 0 and int(obs["lap_counts"][0]) >= lap_target:
            lap_target_reached = True
            break
        if max_steps > 0 and step_count >= max_steps:
            break

    log_path = logger.write()
    metrics = make_metrics(logger.rows, condition["mode"])
    metrics.update(
        {
            "condition": condition_name,
            "mode": condition["mode"],
            "lap_target_reached": int(lap_target_reached),
            "log_csv": str(log_path),
            "real_elapsed_time_s": float(time.time() - start),
        }
    )
    print(
        f"{condition_name}: steps={step_count}, "
        f"laps={metrics['final_lap_count']}, "
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
            condition,
            max_steps,
            lap_target,
            seed,
            output_dir,
        )
        for condition_name, condition in CONDITIONS.items()
    ]

    summary_path = output_dir / "step5_pure_pursuit_comparison.csv"
    fieldnames = [
        "condition",
        "mode",
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
        "mean_est_xy_error_m",
        "max_est_xy_error_m",
        "mean_est_theta_error_rad",
        "max_est_theta_error_rad",
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
    parser = argparse.ArgumentParser(description="Step 5 Pure Pursuit条件比較")
    parser.add_argument(
        "--config",
        default="experiments/configs/homur_oval_localized_a1_pure_pursuit.yaml",
        help="共通で使う設定ファイル",
    )
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--lap-target", type=int, default=0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/step5_pure_pursuit",
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
