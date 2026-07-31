"""Step 3のLiDAR条件別データを保存する計測スクリプト。"""

import argparse
import csv
from pathlib import Path

import numpy as np

from f110_gym.envs.f110_env import F110Env


CONDITIONS = {
    "S0_ideal": {
        "noise_std": 0.0,
        "noise_std_per_meter": 0.0,
        "dropout_probability": 0.0,
        "scan_delay": 0.0,
    },
    "S1_noise": {
        "noise_std": 0.01,
        "noise_std_per_meter": 0.002,
        "dropout_probability": 0.0,
        "scan_delay": 0.0,
    },
    "S2_noise_rate": {
        "noise_std": 0.01,
        "noise_std_per_meter": 0.002,
        "dropout_probability": 0.0,
        "scan_delay": 0.0,
    },
    "S3_noise_rate_delay": {
        "noise_std": 0.01,
        "noise_std_per_meter": 0.002,
        "dropout_probability": 0.0,
        "scan_delay": 0.05,
    },
    "S4_noise_rate_dropout": {
        "noise_std": 0.01,
        "noise_std_per_meter": 0.002,
        "dropout_probability": 0.03,
        "scan_delay": 0.05,
    },
}


def run_condition(name, overrides, steps, seed):
    config = {
        "profile": "rplidar_a1m8_r6",
        "scan_rate_hz": 5.5 if name != "S0_ideal" and name != "S1_noise" else None,
    }
    config.update(overrides)

    env = F110Env(seed=seed, num_agents=1, timestep=0.01, lidar_config=config)
    obs, _, _, _ = env.reset(np.array([[0.0, 0.0, 0.0]]))
    scans = [np.asarray(obs["scans"][0], dtype=float)]
    metadata = [
        (
            0,
            0.0,
            bool(obs["scan_updated"][0]),
            float(obs["scan_times"][0]),
            float(obs["scan_ages"][0]),
        )
    ]

    for step in range(1, steps + 1):
        obs, _, _, _ = env.step(np.zeros((1, 2)))
        scans.append(np.asarray(obs["scans"][0], dtype=float))
        metadata.append(
            (
                step,
                step * 0.01,
                bool(obs["scan_updated"][0]),
                float(obs["scan_times"][0]),
                float(obs["scan_ages"][0]),
            )
        )

    return np.stack(scans), metadata


def main():
    parser = argparse.ArgumentParser(description="Step 3 LiDAR条件別計測")
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/lidar_step3",
        help="npzとCSVの出力先",
    )
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    for name, overrides in CONDITIONS.items():
        scans, metadata = run_condition(name, overrides, args.steps, args.seed)
        np.savez_compressed(
            output_dir / f"{name}.npz",
            scans=scans,
            step=np.asarray([row[0] for row in metadata]),
            sim_time=np.asarray([row[1] for row in metadata]),
            scan_updated=np.asarray([row[2] for row in metadata]),
            scan_time=np.asarray([row[3] for row in metadata]),
            scan_age=np.asarray([row[4] for row in metadata]),
        )

        summary_path = output_dir / f"{name}_summary.csv"
        with summary_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "step",
                    "sim_time",
                    "scan_updated",
                    "scan_time",
                    "scan_age",
                    "scan_min",
                    "scan_mean",
                    "scan_max",
                ]
            )
            for row, scan in zip(metadata, scans):
                writer.writerow(
                    [
                        *row,
                        float(np.min(scan)),
                        float(np.mean(scan)),
                        float(np.max(scan)),
                    ]
                )

        print(f"{name}: scans={scans.shape}, output={output_dir}")


if __name__ == "__main__":
    main()
