"""高速Pure Pursuitで真値姿勢と推定姿勢を比較する。"""

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
    "truth": {"mode": "truth", "lidar": None},
    "ideal": {
        "mode": "estimated",
        "lidar": {
            "noise_std": 0.0,
            "noise_std_per_meter": 0.0,
            "dropout_probability": 0.0,
            "scan_rate_hz": 5.5,
            "scan_delay": 0.0,
        },
    },
    "noise": {
        "mode": "estimated",
        "lidar": {
            "noise_std": 0.01,
            "noise_std_per_meter": 0.002,
            "dropout_probability": 0.0,
            "scan_rate_hz": 5.5,
            "scan_delay": 0.0,
        },
    },
    "noise_delay_dropout": {
        "mode": "estimated",
        "lidar": {
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


def build_condition_config(base_config, condition_name, condition, speed):
    config = copy.deepcopy(base_config)
    config["run_name"] = (
        f"{base_config['run_name']}_pure_pursuit_{condition_name}_v{speed:.1f}"
    )
    config["controller_type"] = "pure_pursuit"
    config["controller"]["vgain"] = float(speed)
    config["lidar"] = copy.deepcopy(condition["lidar"])
    if config["lidar"] is not None:
        config["lidar"]["profile"] = "rplidar_a1m8_r6"
    return config


def collision_events(rows):
    events = 0
    previous = 0
    for row in rows:
        current = int(row["collision"])
        if current and not previous:
            events += 1
        previous = current
    return events


def run_condition(base_config, condition_name, condition, speed, max_steps, lap_target, seed, output_dir):
    config = build_condition_config(base_config, condition_name, condition, speed)
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
        localizer = None
        if condition["mode"] == "estimated":
            localizer = create_localizer(conf, env.sim.agents[0].scan_angles)
        obs, _, done, _ = env.reset(
            np.array([[conf.sx, conf.sy, conf.stheta]])
        )
        control_obs = obs
        est_pose = None
        debug_info = {}
        if localizer is not None:
            est_pose = localizer.initialize(
                np.array([conf.sx, conf.sy, conf.stheta], dtype=float)
            )
            control_obs = build_estimated_obs(obs, est_pose)
            debug_info = localizer.debug_info()

        condition_label = f"{condition_name}_v{speed:.1f}"
        if localizer is None:
            logger = RunLogger(controller, condition_label, conf.run_name, output_dir)
            logger.record(0, 0.0, obs, 0.0, 0.0, done, plan_time_s=0.0)
        else:
            logger = LocalizedRunLogger(
                controller, condition_label, conf.run_name, output_dir
            )
            logger.record(
                0, 0.0, obs, est_pose, debug_info, 0.0, 0.0, done,
                controller_plan_time_s=0.0,
                localizer_update_time_s=0.0,
            )

        sim_elapsed_time = 0.0
        step_count = 0
        lap_target_reached = False
        plan_times = []
        localizer_times = []
        start = time.perf_counter()
        while not done:
            plan_start = time.perf_counter()
            command_speed, steer = controller.plan(control_obs)
            plan_time_s = time.perf_counter() - plan_start
            plan_times.append(plan_time_s)
            obs, step_reward, done, _ = env.step(
                np.array([[steer, command_speed]])
            )
            sim_elapsed_time += step_reward
            step_count += 1

            if localizer is None:
                control_obs = obs
                logger.record(
                    step_count, sim_elapsed_time, obs, command_speed, steer, done,
                    plan_time_s=plan_time_s,
                )
            else:
                localizer_start = time.perf_counter()
                est_pose = localizer.update(
                    obs,
                    control={"speed_cmd": command_speed, "steer_cmd": steer},
                )
                localizer_time_s = time.perf_counter() - localizer_start
                localizer_times.append(localizer_time_s)
                control_obs = build_estimated_obs(obs, est_pose)
                logger.record(
                    step_count, sim_elapsed_time, obs, est_pose,
                    localizer.debug_info(), command_speed, steer, done,
                    controller_plan_time_s=plan_time_s,
                    localizer_update_time_s=localizer_time_s,
                )

            if lap_target > 0 and int(obs["lap_counts"][0]) >= lap_target:
                lap_target_reached = True
                break
            if max_steps > 0 and step_count >= max_steps:
                break

        rows = logger.rows
        cross_track = np.asarray(
            [float(row["cross_track_error"]) for row in rows], dtype=float
        )
        result = {
            "condition": condition_name,
            "mode": condition["mode"],
            "target_speed_mps": float(speed),
            "seed": int(seed),
            "row_count": len(rows),
            "sim_time_s": float(rows[-1]["sim_time"]),
            "lap_target": int(lap_target),
            "final_lap_count": int(rows[-1]["lap_count"]),
            "lap_target_reached": int(lap_target_reached),
            "mean_abs_cross_track_m": float(np.mean(np.abs(cross_track))),
            "max_abs_cross_track_m": float(np.max(np.abs(cross_track))),
            "collision_count": collision_events(rows),
            "mean_plan_time_ms": float(np.mean(plan_times) * 1000.0),
            "control_overrun_count": int(
                np.sum(np.asarray(plan_times) > float(conf.timestep))
            ),
            "mean_localizer_time_ms": "",
            "localizer_overrun_count": "",
            "mean_est_xy_error_m": "",
            "max_est_xy_error_m": "",
            "real_elapsed_time_s": float(time.perf_counter() - start),
        }
        if localizer is not None:
            xy_errors = np.asarray(
                [
                    np.hypot(float(row["est_error_x"]), float(row["est_error_y"]))
                    for row in rows
                ],
                dtype=float,
            )
            result["mean_localizer_time_ms"] = float(
                np.mean(localizer_times) * 1000.0
            )
            result["localizer_overrun_count"] = int(
                np.sum(np.asarray(localizer_times) > float(conf.timestep))
            )
            result["mean_est_xy_error_m"] = float(np.mean(xy_errors))
            result["max_est_xy_error_m"] = float(np.max(xy_errors))
        result["log_csv"] = str(logger.write())
        return result
    finally:
        del env


def run_experiment(config_path, speeds, max_steps, lap_target, seed, output_dir):
    base_config = load_config(config_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for condition_name, condition in CONDITIONS.items():
        for speed in speeds:
            result = run_condition(
                base_config, condition_name, condition, speed,
                max_steps, lap_target, seed, output_dir,
            )
            results.append(result)
            print(
                f"{condition_name}: speed={speed:.1f}, "
                f"steps={result['row_count']}, laps={result['final_lap_count']}, "
                f"cte={result['mean_abs_cross_track_m']:.4f} m, "
                f"collision={result['collision_count']}"
            )

    summary_path = output_dir / "high_speed_pure_pursuit_comparison.csv"
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
    parser = argparse.ArgumentParser(description="高速Pure Pursuit比較")
    parser.add_argument(
        "--config",
        default="experiments/configs/homur_oval_localized_a1_mppi_safe_3mps.yaml",
    )
    parser.add_argument("--speeds", default="3.0,5.0")
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--lap-target", type=int, default=0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/high_speed_pure_pursuit",
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
