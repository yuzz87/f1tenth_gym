"""次段階のLiDAR・MPPI評価をまとめて実行する。"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_high_speed_localized_mppi import (  # noqa: E402
    LIDAR_CONDITIONS,
    load_config,
    run_condition,
)
from experiments.compare_high_speed_mppi import (  # noqa: E402
    HORIZON_CONDITIONS,
    run_condition as run_truth_condition,
)
from experiments.run_controller_experiment import resolve_repo_path  # noqa: E402


DEFAULT_CONFIG = (
    "experiments/configs/"
    "homur_oval_localized_a1_mppi_safe_3mps.yaml"
)


def write_rows(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return path
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return path


def aggregate_seed_results(rows):
    """seedごとの成功率と誤差・時間の平均/標準偏差を集計する。"""
    groups = defaultdict(list)
    for row in rows:
        groups[row["lidar_condition"]].append(row)

    metrics = (
        "mean_est_xy_error_m",
        "max_est_xy_error_m",
        "mean_plan_time_ms",
        "mean_localizer_time_ms",
        "mean_total_cycle_time_ms",
        "p95_total_cycle_time_ms",
        "max_total_cycle_time_ms",
        "collision_count",
    )
    summary = []
    for condition, condition_rows in groups.items():
        result = {
            "lidar_condition": condition,
            "seed_count": len(condition_rows),
            "lap_success_rate": float(
                np.mean([int(row["lap_target_reached"]) for row in condition_rows])
            ),
        }
        for metric in metrics:
            values = np.asarray(
                [float(row[metric]) for row in condition_rows], dtype=float
            )
            result[f"{metric}_mean"] = float(np.mean(values))
            result[f"{metric}_std"] = float(np.std(values))
        summary.append(result)
    return summary


def execute_reproducibility(
    base_config,
    output_dir,
    seeds,
    lap_target,
    label="reproducibility",
    controller_samples=128,
    localizer_xy_candidates=3,
    localizer_theta_candidates=5,
):
    """3 m/sを複数seed・3 LiDAR条件で評価する。"""
    rows = []
    for condition, overrides in LIDAR_CONDITIONS.items():
        for seed in seeds:
            print(f"reproducibility: condition={condition}, seed={seed}")
            rows.append(
                run_condition(
                    base_config,
                    condition,
                    overrides,
                    3.0,
                    0,
                    lap_target,
                    seed,
                    output_dir / f"{label}_logs",
                    controller_samples=controller_samples,
                    localizer_xy_candidates=localizer_xy_candidates,
                    localizer_theta_candidates=localizer_theta_candidates,
                )
            )
    raw_path = write_rows(output_dir / f"{label}_runs.csv", rows)
    summary = aggregate_seed_results(rows)
    summary_path = write_rows(output_dir / f"{label}_summary.csv", summary)
    return rows, summary, raw_path, summary_path


def execute_speed5_10hz_noise(base_config, output_dir, seed, max_steps, lap_target):
    """10 Hzでノイズ・遅延・欠損を含む5 m/s条件を評価する。"""
    overrides = dict(LIDAR_CONDITIONS["noise_delay_dropout"])
    overrides["scan_rate_hz"] = 10.0
    row = run_condition(
        base_config,
        "speed5_10hz_noise_delay_dropout",
        overrides,
        5.0,
        max_steps,
        lap_target,
        seed,
        output_dir / "speed5_10hz_noise_logs",
    )
    path = write_rows(output_dir / "speed5_10hz_noise_delay_dropout.csv", [row])
    return row, path


def execute_extrinsics(base_config, output_dir, seed, lap_target):
    """LiDAR取付位置の一致、不一致、補正済み条件を比較する。"""
    variants = (
        ("nominal_matched", {"x_offset": 0.0, "y_offset": 0.0, "yaw_offset": 0.0}, {}),
        ("x_10cm_mismatch", {"x_offset": 0.10}, {"x_offset": 0.0}),
        ("x_10cm_corrected", {"x_offset": 0.10}, {"x_offset": 0.10}),
        ("y_5cm_mismatch", {"y_offset": 0.05}, {"y_offset": 0.0}),
        ("y_5cm_corrected", {"y_offset": 0.05}, {"y_offset": 0.05}),
        ("yaw_5deg_mismatch", {"yaw_offset": 0.05}, {"yaw_offset": 0.0}),
        ("yaw_5deg_corrected", {"yaw_offset": 0.05}, {"yaw_offset": 0.05}),
    )
    rows = []
    for name, true_extrinsics, assumed_extrinsics in variants:
        print(f"extrinsics: variant={name}")
        row = run_condition(
            base_config,
            f"extrinsics_{name}",
            LIDAR_CONDITIONS["ideal"],
            3.0,
            0,
            lap_target,
            seed,
            output_dir / "extrinsics_logs",
            lidar_extrinsics=true_extrinsics,
            localizer_extrinsics=assumed_extrinsics,
        )
        row["variant"] = name
        row["true_extrinsics"] = str(true_extrinsics)
        row["assumed_extrinsics"] = str(assumed_extrinsics)
        rows.append(row)
    path = write_rows(output_dir / "extrinsics_comparison.csv", rows)
    return rows, path


def execute_speed5_pose_separation(
    base_config, output_dir, seed, max_steps, lap_target
):
    """5 m/sで真値姿勢と推定姿勢の制御性能を分離する。"""
    truth_row = run_truth_condition(
        base_config,
        "horizon_12",
        HORIZON_CONDITIONS["horizon_12"],
        5.0,
        max_steps,
        lap_target,
        seed,
        output_dir / "speed5_truth_logs",
    )
    truth_row["pose_source"] = "truth"
    truth_path = write_rows(output_dir / "speed5_truth_mppi.csv", [truth_row])

    estimated_rows = []
    for condition, overrides in LIDAR_CONDITIONS.items():
        print(f"speed5_pose_separation: condition={condition}")
        row = run_condition(
            base_config,
            f"speed5_estimated_{condition}",
            overrides,
            5.0,
            max_steps,
            lap_target,
            seed,
            output_dir / "speed5_estimated_logs",
        )
        row["pose_source"] = "estimated"
        estimated_rows.append(row)
    estimated_path = write_rows(
        output_dir / "speed5_estimated_mppi.csv", estimated_rows
    )
    return truth_row, estimated_rows, truth_path, estimated_path


def execute_timing(base_config, output_dir, seed, max_steps, lap_target=0):
    """MPPIサンプル数とlocalizer候補数を変えて合計周期を測定する。"""
    variants = (
        ("baseline", 12, 128, 90, 3, 5),
        ("mppi_96", 12, 96, 90, 3, 5),
        ("mppi_64", 12, 64, 90, 3, 5),
        ("localizer_27", 12, 128, 90, 3, 3),
        ("combined_64_27", 12, 64, 90, 3, 3),
    )
    rows = []
    for name, horizon, samples, beams, xy_candidates, theta_candidates in variants:
        print(f"timing: variant={name}")
        row = run_condition(
            base_config,
            f"timing_{name}",
            LIDAR_CONDITIONS["ideal"],
            3.0,
            max_steps,
            lap_target,
            seed,
            output_dir / "timing_logs",
            controller_horizon=horizon,
            controller_samples=samples,
            localizer_scan_beams=beams,
            localizer_xy_candidates=xy_candidates,
            localizer_theta_candidates=theta_candidates,
        )
        row["variant"] = name
        rows.append(row)
    path = write_rows(output_dir / "timing_variants.csv", rows)
    return rows, path


def execute_speed5(base_config, output_dir, seed, max_steps, lap_target):
    """5 m/sの失敗要因をLiDAR周期・遅延・探索幅・MPPIで比較する。"""
    variants = (
        ("baseline_5p5hz", 12, 128, 90, 3, 5, 5.5, 0.0, True),
        ("scan_10hz", 12, 128, 90, 3, 5, 10.0, 0.0, True),
        ("scan_20hz", 12, 128, 90, 3, 5, 20.0, 0.0, True),
        ("localizer_wide", 12, 128, 180, 5, 7, 5.5, 0.0, True),
        ("mppi_h16", 16, 128, 90, 3, 5, 5.5, 0.0, True),
        ("no_time_compensation", 12, 128, 90, 3, 5, 5.5, 0.0, False),
    )
    rows = []
    for (
        name,
        horizon,
        samples,
        beams,
        xy_candidates,
        theta_candidates,
        scan_rate,
        delay,
        compensate,
    ) in variants:
        print(f"speed5: variant={name}")
        overrides = dict(LIDAR_CONDITIONS["ideal"])
        overrides["scan_rate_hz"] = scan_rate
        overrides["scan_delay"] = delay
        row = run_condition(
            base_config,
            f"speed5_{name}",
            overrides,
            5.0,
            max_steps,
            lap_target,
            seed,
            output_dir / "speed5_logs",
            controller_horizon=horizon,
            controller_samples=samples,
            localizer_scan_beams=beams,
            localizer_xy_candidates=xy_candidates,
            localizer_theta_candidates=theta_candidates,
            localizer_scan_time_compensation=compensate,
        )
        row["variant"] = name
        row["scan_rate_hz"] = scan_rate
        row["scan_delay_s"] = delay
        row["scan_time_compensation"] = int(compensate)
        rows.append(row)
    path = write_rows(output_dir / "speed5_variants.csv", rows)
    return rows, path


def parse_ints(text):
    return [int(value.strip()) for value in text.split(",") if value.strip()]


def main():
    parser = argparse.ArgumentParser(description="LiDAR・MPPI次段階評価")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument(
        "--realtime-config",
        default=(
            "experiments/configs/"
            "homur_oval_localized_a1_mppi_realtime_candidate_3mps.yaml"
        ),
        help="短縮設定のseed再評価に使う設定ファイル",
    )
    parser.add_argument("--seeds", default="123,456,789")
    parser.add_argument("--lap-target", type=int, default=1)
    parser.add_argument("--timing-steps", type=int, default=100)
    parser.add_argument(
        "--timing-lap-target",
        type=int,
        default=0,
        help="処理時間比較でも指定周回数まで実行する。0ならstepsを使用する。",
    )
    parser.add_argument("--speed5-max-steps", type=int, default=0)
    parser.add_argument(
        "--output-dir", default="experiments/results/lidar_mppi_next"
    )
    args = parser.parse_args()

    config_path = resolve_repo_path(args.config)
    base_config = load_config(config_path)
    realtime_config = load_config(resolve_repo_path(args.realtime_config))
    output_dir = resolve_repo_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    seeds = parse_ints(args.seeds)
    if not seeds:
        raise ValueError("--seeds には少なくとも1つ指定してください")

    reproducibility = execute_reproducibility(
        realtime_config,
        output_dir,
        seeds,
        args.lap_target,
        label="realtime_reproducibility",
        controller_samples=64,
        localizer_xy_candidates=3,
        localizer_theta_candidates=3,
    )
    timing = execute_timing(
        base_config,
        output_dir,
        seeds[0],
        args.timing_steps,
        args.timing_lap_target,
    )
    speed5 = execute_speed5(
        base_config, output_dir, seeds[0], args.speed5_max_steps, args.lap_target
    )
    speed5_10hz_noise = execute_speed5_10hz_noise(
        base_config,
        output_dir,
        seeds[0],
        args.speed5_max_steps,
        args.lap_target,
    )
    extrinsics = execute_extrinsics(
        base_config, output_dir, seeds[0], args.lap_target
    )
    pose_separation = execute_speed5_pose_separation(
        base_config,
        output_dir,
        seeds[0],
        args.speed5_max_steps,
        args.lap_target,
    )

    print(f"realtime_reproducibility_raw_csv: {reproducibility[2]}")
    print(f"realtime_reproducibility_summary_csv: {reproducibility[3]}")
    print(f"timing_csv: {timing[1]}")
    print(f"speed5_csv: {speed5[1]}")
    print(f"speed5_10hz_noise_csv: {speed5_10hz_noise[1]}")
    print(f"extrinsics_csv: {extrinsics[1]}")
    print(f"speed5_truth_csv: {pose_separation[2]}")
    print(f"speed5_estimated_csv: {pose_separation[3]}")


if __name__ == "__main__":
    main()
