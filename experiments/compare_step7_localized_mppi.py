"""Step 7のLiDAR推定姿勢MPPI比較。"""

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
    RunLogger,
    get_integrator,
    resolve_repo_path,
)
from experiments.run_localized_controller_experiment import (  # noqa: E402
    LocalizedRunLogger,
    build_estimated_obs,
)


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
    "E_estimated_noise_delay_dropout": {
        "mode": "estimated",
        "lidar": {
            "profile": "rplidar_a1m8_r6",
            "noise_std": 0.01,
            "noise_std_per_meter": 0.002,
            "dropout_probability": 0.03,
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
    config["controller_type"] = "mppi"
    config["lidar"] = copy.deepcopy(condition["lidar"])
    return config


def _row_time(rows, key):
    values = np.asarray([float(row[key]) for row in rows[1:]], dtype=float)
    return values if values.size else np.zeros(1, dtype=float)


def summarize_rows(rows, condition_name, mode, controller, timestep):
    if not rows:
        raise ValueError("condition produced no log rows")

    cross_track = np.asarray([float(row["cross_track_error"]) for row in rows])
    speed = np.asarray([float(row["linear_vel_x"]) for row in rows])
    steer = np.asarray([float(row["steer_cmd"]) for row in rows])
    plan_time = _row_time(
        rows,
        "plan_time_s" if mode == "truth" else "controller_plan_time_s",
    )
    localizer_time = (
        np.zeros(1, dtype=float)
        if mode == "truth"
        else _row_time(rows, "localizer_update_time_s")
    )
    if mode == "truth":
        xy_error = np.zeros(len(rows), dtype=float)
        theta_error = np.zeros(len(rows), dtype=float)
    else:
        xy_error = np.asarray(
            [
                np.hypot(float(row["est_error_x"]), float(row["est_error_y"]))
                for row in rows
            ]
        )
        theta_error = np.asarray(
            [abs(float(row["est_error_theta"])) for row in rows]
        )
    collisions = np.asarray([int(row["collision"]) for row in rows])
    return {
        "condition": condition_name,
        "mode": mode,
        "horizon": int(controller.horizon),
        "num_samples": int(controller.num_samples),
        "temperature": float(controller.temperature),
        "noise_sigma": float(controller.noise_sigma),
        "rollout_model": getattr(controller, "rollout_model", "unknown"),
        "row_count": len(rows),
        "sim_time_s": float(rows[-1]["sim_time"]),
        "final_lap_count": int(rows[-1]["lap_count"]),
        "final_lap_time_s": float(rows[-1]["lap_time"]),
        "mean_abs_cross_track_m": float(np.mean(np.abs(cross_track))),
        "max_abs_cross_track_m": float(np.max(np.abs(cross_track))),
        "mean_speed_mps": float(np.mean(speed)),
        "total_abs_steer_change_rad": float(np.sum(np.abs(np.diff(steer)))),
        "collision_count": int(np.sum(collisions)),
        "out_of_course_count": int(np.sum(np.abs(cross_track) > 0.5)),
        "mean_est_xy_error_m": float(np.mean(xy_error)),
        "max_est_xy_error_m": float(np.max(xy_error)),
        "mean_est_theta_error_rad": float(np.mean(theta_error)),
        "max_est_theta_error_rad": float(np.max(theta_error)),
        "mean_plan_time_s": float(np.mean(plan_time)),
        "p95_plan_time_s": float(np.percentile(plan_time, 95)),
        "max_plan_time_s": float(np.max(plan_time)),
        "controller_overrun_count": int(np.sum(plan_time > timestep)),
        "mean_localizer_time_s": float(np.mean(localizer_time)),
        "p95_localizer_time_s": float(np.percentile(localizer_time, 95)),
        "max_localizer_time_s": float(np.max(localizer_time)),
        "localizer_overrun_count": int(np.sum(localizer_time > timestep)),
    }


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
        localizer = create_localizer(conf, env.sim.agents[0].scan_angles)

    obs, step_reward, done, info = env.reset(
        np.array([[conf.sx, conf.sy, conf.stheta]])
    )
    del step_reward, info
    control_obs = obs
    est_pose = np.array([conf.sx, conf.sy, conf.stheta], dtype=float)
    debug_info = {}
    if localizer is not None:
        est_pose = localizer.initialize(est_pose)
        control_obs = build_estimated_obs(obs, est_pose)
        debug_info = localizer.debug_info()

    if localizer is None:
        logger = RunLogger(controller, condition_name, conf.run_name, output_dir)
        logger.record(0, 0.0, obs, 0.0, 0.0, done, plan_time_s=0.0)
    else:
        logger = LocalizedRunLogger(controller, condition_name, conf.run_name, output_dir)
        logger.record(
            0,
            0.0,
            obs,
            est_pose,
            debug_info,
            0.0,
            0.0,
            done,
            controller_plan_time_s=0.0,
            localizer_update_time_s=0.0,
        )

    sim_elapsed_time = 0.0
    step_count = 0
    lap_target_reached = False
    start = time.perf_counter()
    while not done:
        plan_start = time.perf_counter()
        speed, steer = controller.plan(control_obs)
        plan_time_s = time.perf_counter() - plan_start
        obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
        del info
        sim_elapsed_time += step_reward
        step_count += 1

        if localizer is None:
            control_obs = obs
            logger.record(
                step_count,
                sim_elapsed_time,
                obs,
                speed,
                steer,
                done,
                plan_time_s=plan_time_s,
            )
        else:
            localizer_start = time.perf_counter()
            est_pose = localizer.update(
                obs,
                control={"speed_cmd": speed, "steer_cmd": steer},
            )
            localizer_time_s = time.perf_counter() - localizer_start
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
                controller_plan_time_s=plan_time_s,
                localizer_update_time_s=localizer_time_s,
            )

        if lap_target > 0 and int(obs["lap_counts"][0]) >= lap_target:
            lap_target_reached = True
            break
        if max_steps > 0 and step_count >= max_steps:
            break

    log_path = logger.write()
    metrics = summarize_rows(
        logger.rows,
        condition_name,
        condition["mode"],
        controller,
        float(conf.timestep),
    )
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
        f"mean_localizer={metrics['mean_localizer_time_s'] * 1000.0:.3f} ms"
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

    summary_path = output_dir / "step7_localized_mppi_comparison.csv"
    fieldnames = [
        "condition",
        "mode",
        "horizon",
        "num_samples",
        "temperature",
        "noise_sigma",
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
        "mean_est_xy_error_m",
        "max_est_xy_error_m",
        "mean_est_theta_error_rad",
        "max_est_theta_error_rad",
        "mean_plan_time_s",
        "p95_plan_time_s",
        "max_plan_time_s",
        "controller_overrun_count",
        "mean_localizer_time_s",
        "p95_localizer_time_s",
        "max_localizer_time_s",
        "localizer_overrun_count",
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
    parser = argparse.ArgumentParser(description="Step 7 推定姿勢MPPI比較")
    parser.add_argument(
        "--config",
        default="experiments/configs/homur_oval_localized_a1_mppi_step7.yaml",
        help="共通で使うMPPI/localizer設定ファイル",
    )
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--lap-target", type=int, default=0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/step7_localized_mppi",
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
