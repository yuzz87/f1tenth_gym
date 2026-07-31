"""推定姿勢を使う高速MPPIをLiDAR条件別に比較する。"""

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
    LocalizedRunLogger,
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


def build_condition_config(
    base_config,
    lidar_condition,
    lidar_overrides,
    speed,
    controller_horizon=12,
    controller_samples=128,
):
    config = copy.deepcopy(base_config)
    config["run_name"] = (
        f"{base_config['run_name']}_localized_{lidar_condition}_v{speed:.1f}"
    )
    config["controller_type"] = "mppi"
    config["controller"]["horizon"] = int(controller_horizon)
    config["controller"]["num_samples"] = int(controller_samples)
    config["controller"]["target_speed"] = float(speed)
    config["controller"]["vgain"] = float(speed)
    config["lidar"] = copy.deepcopy(config.get("lidar") or {})
    config["lidar"].update(lidar_overrides)
    config["lidar"]["profile"] = "rplidar_a1m8_r6"
    return config


def count_collision_events(rows):
    events = 0
    previous = 0
    for row in rows:
        current = int(row["collision"])
        if current and not previous:
            events += 1
        previous = current
    return events


def run_condition(
    base_config,
    lidar_condition,
    lidar_overrides,
    speed,
    max_steps,
    lap_target,
    seed,
    output_dir,
    controller_horizon=12,
    controller_samples=128,
    localizer_scan_beams=None,
    localizer_xy_candidates=None,
    localizer_theta_candidates=None,
    localizer_scan_time_compensation=None,
    localizer_scan_motion_history_compensation=None,
    localizer_motion_history_seconds=None,
    localizer_update_mode="every_step",
    lidar_extrinsics=None,
    localizer_extrinsics=None,
    controller_target_speed=None,
    controller_vgain=None,
    controller_noise_sigma=None,
    controller_steer_limit=None,
):
    config = build_condition_config(
        base_config,
        lidar_condition,
        lidar_overrides,
        speed,
        controller_horizon=controller_horizon,
        controller_samples=controller_samples,
    )
    config["controller"]["seed"] = int(seed)
    if controller_target_speed is not None:
        config["controller"]["target_speed"] = float(controller_target_speed)
    if controller_vgain is not None:
        config["controller"]["vgain"] = float(controller_vgain)
    if controller_noise_sigma is not None:
        config["controller"]["noise_sigma"] = float(controller_noise_sigma)
    if controller_steer_limit is not None:
        config["controller"]["steer_limit"] = float(controller_steer_limit)
    config["localizer"] = copy.deepcopy(config.get("localizer") or {})
    if localizer_scan_beams is not None:
        config["localizer"]["scan_beams"] = int(localizer_scan_beams)
    if localizer_xy_candidates is not None:
        config["localizer"]["xy_candidates"] = int(localizer_xy_candidates)
    if localizer_theta_candidates is not None:
        config["localizer"]["theta_candidates"] = int(localizer_theta_candidates)
    if localizer_scan_time_compensation is not None:
        config["localizer"]["scan_time_compensation"] = bool(
            localizer_scan_time_compensation
        )
    if localizer_scan_motion_history_compensation is not None:
        config["localizer"]["scan_motion_history_compensation"] = bool(
            localizer_scan_motion_history_compensation
        )
    if localizer_motion_history_seconds is not None:
        config["localizer"]["motion_history_seconds"] = float(
            localizer_motion_history_seconds
        )
    if localizer_update_mode not in ("every_step", "on_scan_update"):
        raise ValueError(
            "localizer_update_mode must be 'every_step' or 'on_scan_update'"
        )
    if lidar_extrinsics:
        config["lidar"].update(lidar_extrinsics)
    if localizer_extrinsics:
        config["localizer"].update(localizer_extrinsics)
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
        condition_name = f"{lidar_condition}_v{speed:.1f}"
        logger = LocalizedRunLogger(
            controller,
            condition_name,
            conf.run_name,
            output_dir,
        )
        logger.record(
            0,
            0.0,
            obs,
            est_pose,
            localizer.debug_info(),
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
        localizer_match_count = 0
        while not done:
            plan_start = time.perf_counter()
            command_speed, steer = controller.plan(control_obs)
            plan_time_s = time.perf_counter() - plan_start
            obs, step_reward, done, _ = env.step(
                np.array([[steer, command_speed]])
            )
            sim_elapsed_time += step_reward
            step_count += 1

            localizer_start = time.perf_counter()
            should_match_scan = (
                localizer_update_mode == "every_step"
                or bool(obs["scan_updated"][0])
            )
            if should_match_scan:
                est_pose = localizer.update(
                    obs,
                    control={"speed_cmd": command_speed, "steer_cmd": steer},
                )
                localizer_match_count += 1
            else:
                est_pose = localizer.predict(
                    obs,
                    control={"speed_cmd": command_speed, "steer_cmd": steer},
                )
            localizer_time_s = time.perf_counter() - localizer_start
            control_obs = build_estimated_obs(obs, est_pose)
            logger.record(
                step_count,
                sim_elapsed_time,
                obs,
                est_pose,
                localizer.debug_info(),
                command_speed,
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

        rows = logger.rows
        plan_times = np.asarray(
            [float(row["controller_plan_time_s"]) for row in rows[1:]],
            dtype=float,
        )
        localizer_times = np.asarray(
            [float(row["localizer_update_time_s"]) for row in rows[1:]],
            dtype=float,
        )
        if plan_times.size == 0:
            plan_times = np.zeros(1, dtype=float)
            localizer_times = np.zeros(1, dtype=float)
        total_times = plan_times + localizer_times
        xy_errors = np.asarray(
            [
                np.hypot(float(row["est_error_x"]), float(row["est_error_y"]))
                for row in rows
            ],
            dtype=float,
        )
        cross_track = np.asarray(
            [float(row["cross_track_error"]) for row in rows], dtype=float
        )
        speeds = np.asarray(
            [float(row["linear_vel_x"]) for row in rows], dtype=float
        )
        log_path = logger.write()
        return {
            "lidar_condition": lidar_condition,
            "target_speed_mps": float(speed),
            "controller_target_speed_mps": float(controller.speed_target),
            "horizon": int(controller.horizon),
            "controller_samples": int(controller.num_samples),
            "steer_min_rad": float(controller.steer_min),
            "steer_max_rad": float(controller.steer_max),
            "localizer_scan_beams": int(localizer.debug_info()["scan_beams"]),
            "localizer_candidate_count": int(
                localizer.debug_info()["candidate_count"]
            ),
            "localizer_update_mode": localizer_update_mode,
            "localizer_match_count": int(localizer_match_count),
            "scan_time_compensation": int(
                localizer.debug_info().get("scan_time_compensation", False)
            ),
            "scan_motion_history_compensation": int(
                localizer.debug_info().get(
                    "scan_motion_history_compensation", False
                )
            ),
            "seed": int(seed),
            "row_count": len(rows),
            "sim_time_s": float(rows[-1]["sim_time"]),
            "lap_target": int(lap_target),
            "final_lap_count": int(rows[-1]["lap_count"]),
            "lap_target_reached": int(lap_target_reached),
            "mean_speed_mps": float(np.mean(speeds)),
            "mean_abs_cross_track_m": float(np.mean(np.abs(cross_track))),
            "max_abs_cross_track_m": float(np.max(np.abs(cross_track))),
            "mean_est_xy_error_m": float(np.mean(xy_errors)),
            "max_est_xy_error_m": float(np.max(xy_errors)),
            "collision_count": count_collision_events(rows),
            "mean_plan_time_ms": float(np.mean(plan_times) * 1000.0),
            "p95_plan_time_ms": float(np.percentile(plan_times, 95) * 1000.0),
            "control_overrun_count": int(
                np.sum(plan_times > float(conf.timestep))
            ),
            "mean_localizer_time_ms": float(np.mean(localizer_times) * 1000.0),
            "p95_localizer_time_ms": float(
                np.percentile(localizer_times, 95) * 1000.0
            ),
            "localizer_overrun_count": int(
                np.sum(localizer_times > float(conf.timestep))
            ),
            "mean_total_cycle_time_ms": float(np.mean(total_times) * 1000.0),
            "p95_total_cycle_time_ms": float(
                np.percentile(total_times, 95) * 1000.0
            ),
            "max_total_cycle_time_ms": float(np.max(total_times) * 1000.0),
            "total_cycle_overrun_count": int(
                np.sum(total_times > float(conf.timestep))
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
    for lidar_condition, lidar_overrides in LIDAR_CONDITIONS.items():
        for speed in speeds:
            result = run_condition(
                base_config,
                lidar_condition,
                lidar_overrides,
                speed,
                max_steps,
                lap_target,
                seed,
                output_dir,
            )
            results.append(result)
            print(
                f"{lidar_condition}: speed={speed:.1f}, "
                f"steps={result['row_count']}, "
                f"laps={result['final_lap_count']}, "
                f"plan={result['mean_plan_time_ms']:.3f} ms, "
                f"localizer={result['mean_localizer_time_ms']:.3f} ms, "
                f"collision={result['collision_count']}"
            )

    summary_path = output_dir / "high_speed_localized_mppi_comparison.csv"
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
    parser = argparse.ArgumentParser(description="高速推定姿勢MPPI比較")
    parser.add_argument(
        "--config",
        default="experiments/configs/homur_oval_localized_a1_mppi_step8_fast.yaml",
        help="高速localizer設定ファイル",
    )
    parser.add_argument("--speeds", default="3.0,5.0")
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--lap-target", type=int, default=0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/high_speed_localized_mppi",
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
