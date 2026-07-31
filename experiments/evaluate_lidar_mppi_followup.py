"""5 m/s LiDAR更新周期・外部パラメータ・localizer更新方式の追跡評価。"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path
import sys

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_high_speed_localized_mppi import (  # noqa: E402
    LIDAR_CONDITIONS,
    load_config,
    run_condition,
)
from experiments.run_controller_experiment import resolve_repo_path  # noqa: E402


DEFAULT_CONFIG = (
    "experiments/configs/"
    "homur_oval_localized_a1_mppi_safe_3mps.yaml"
)
DEFAULT_MOUNT = "experiments/configs/lidar_mount_nominal.yaml"


MPPI_TUNING_VARIANTS = {
    "baseline_h12_s64": {
        "controller_horizon": 12,
        "controller_samples": 64,
        "controller_target_speed": 5.0,
        "controller_vgain": 5.0,
        "controller_noise_sigma": 0.08,
    },
    "short_h8_s64": {
        "controller_horizon": 8,
        "controller_samples": 64,
        "controller_target_speed": 5.0,
        "controller_vgain": 5.0,
        "controller_noise_sigma": 0.08,
    },
    "lower_target_h12_s64": {
        "controller_horizon": 12,
        "controller_samples": 64,
        "controller_target_speed": 4.5,
        "controller_vgain": 4.5,
        "controller_noise_sigma": 0.08,
    },
    "limited_steer_h12_s64": {
        "controller_horizon": 12,
        "controller_samples": 64,
        "controller_target_speed": 5.0,
        "controller_vgain": 5.0,
        "controller_noise_sigma": 0.08,
        "controller_steer_limit": 0.36,
    },
    "balanced_h8_s64_limited": {
        "controller_horizon": 8,
        "controller_samples": 64,
        "controller_target_speed": 4.8,
        "controller_vgain": 4.8,
        "controller_noise_sigma": 0.06,
        "controller_steer_limit": 0.36,
    },
}


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


def aggregate(rows, group_key):
    groups = defaultdict(list)
    for row in rows:
        groups[row[group_key]].append(row)
    metrics = (
        "lap_target_reached",
        "collision_count",
        "mean_est_xy_error_m",
        "mean_abs_cross_track_m",
        "mean_localizer_time_ms",
        "mean_total_cycle_time_ms",
        "localizer_match_count",
    )
    summary = []
    for name, group in groups.items():
        result = {group_key: name, "seed_count": len(group)}
        for metric in metrics:
            values = np.asarray([float(row[metric]) for row in group])
            result[f"{metric}_mean"] = float(np.mean(values))
            result[f"{metric}_std"] = float(np.std(values))
        summary.append(result)
    return summary


def aggregate_by_keys(rows, group_keys, metrics=None):
    """複数の実験条件を分けたまま統計量を作る。"""

    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[key] for key in group_keys)].append(row)
    if metrics is None:
        metrics = (
            "lap_target_reached",
            "collision_count",
            "mean_est_xy_error_m",
            "mean_abs_cross_track_m",
            "mean_plan_time_ms",
            "mean_localizer_time_ms",
            "mean_total_cycle_time_ms",
        )
    summary = []
    for key_values, group in groups.items():
        result = dict(zip(group_keys, key_values))
        result["run_count"] = len(group)
        for metric in metrics:
            values = np.asarray([float(row[metric]) for row in group])
            result[f"{metric}_mean"] = float(np.mean(values))
            result[f"{metric}_std"] = float(np.std(values))
        summary.append(result)
    return summary


def build_10hz_conditions():
    conditions = {}
    for name, base in LIDAR_CONDITIONS.items():
        overrides = dict(base)
        overrides["scan_rate_hz"] = 10.0
        conditions[f"{name}_10hz"] = overrides

    # 遅延と欠損を個別に切り分ける。
    delay_only = dict(conditions["noise_10hz"])
    delay_only["scan_delay"] = 0.05
    dropout_only = dict(conditions["noise_10hz"])
    dropout_only["dropout_probability"] = 0.03
    conditions["delay_only_10hz"] = delay_only
    conditions["dropout_only_10hz"] = dropout_only
    return conditions


def evaluate_10hz(base_config, output_dir, seeds, lap_target, max_steps=0):
    rows = []
    for condition, overrides in build_10hz_conditions().items():
        for seed in seeds:
            print(f"10hz: condition={condition}, seed={seed}")
            row = run_condition(
                base_config,
                f"speed5_{condition}",
                overrides,
                5.0,
                max_steps,
                lap_target,
                seed,
                output_dir / "10hz_logs",
            )
            row["condition_group"] = condition
            rows.append(row)
    raw_path = write_rows(output_dir / "speed5_10hz_runs.csv", rows)
    summary = aggregate(rows, "condition_group")
    summary_path = write_rows(output_dir / "speed5_10hz_summary.csv", summary)
    return raw_path, summary_path


def evaluate_update_modes(
    base_config, output_dir, seed, lap_target, max_steps=0
):
    conditions = build_10hz_conditions()
    rows = []
    for condition in ("ideal_10hz", "noise_delay_dropout_10hz"):
        for mode, compensate in (
            ("every_step_compensated", True),
            ("on_scan_update_compensated", True),
            ("on_scan_update_no_compensation", False),
        ):
            print(f"update_mode: condition={condition}, mode={mode}")
            row = run_condition(
                base_config,
                f"speed5_{condition}_{mode}",
                conditions[condition],
                5.0,
                max_steps,
                lap_target,
                seed,
                output_dir / "update_mode_logs",
                localizer_scan_time_compensation=compensate,
                localizer_update_mode=(
                    "every_step"
                    if mode == "every_step_compensated"
                    else "on_scan_update"
                ),
            )
            row["condition_group"] = condition
            row["mode"] = mode
            rows.append(row)
    raw_path = write_rows(output_dir / "localizer_update_modes.csv", rows)
    return raw_path


def load_mount(path):
    with open(path, encoding="utf-8") as file:
        mount = yaml.safe_load(file) or {}
    return {
        key: float(mount.get(key, 0.0))
        for key in ("x_offset", "y_offset", "z_offset", "yaw_offset")
    }


def evaluate_mount(
    base_config, output_dir, seed, lap_target, mount, max_steps=0
):
    """基準取付値を使った一致条件と周辺誤差の感度を記録する。"""
    variants = (
        ("mount_matched", mount, mount),
        (
            "mount_x_plus_5cm",
            {**mount, "x_offset": mount["x_offset"] + 0.05},
            mount,
        ),
        (
            "mount_y_plus_5cm",
            {**mount, "y_offset": mount["y_offset"] + 0.05},
            mount,
        ),
        (
            "mount_yaw_plus_5deg",
            {**mount, "yaw_offset": mount["yaw_offset"] + 0.05},
            mount,
        ),
    )
    rows = []
    for name, true_mount, assumed_mount in variants:
        print(f"mount: variant={name}")
        row = run_condition(
            base_config,
            name,
            LIDAR_CONDITIONS["ideal"],
            3.0,
            max_steps,
            lap_target,
            seed,
            output_dir / "mount_logs",
            lidar_extrinsics=true_mount,
            localizer_extrinsics=assumed_mount,
        )
        row["variant"] = name
        row["mount_source"] = "configured_mount_with_sensitivity_test"
        rows.append(row)
    path = write_rows(output_dir / "mount_sensitivity.csv", rows)
    return path


def evaluate_delay_dropout_sweep(
    base_config, output_dir, seed, lap_target, max_steps=0
):
    """遅延と欠損率を独立に変えて5 m/sを評価する。"""
    rows = []
    for delay in (0.0, 0.02, 0.05, 0.10):
        overrides = dict(LIDAR_CONDITIONS["noise"])
        overrides["scan_rate_hz"] = 10.0
        overrides["scan_delay"] = delay
        print(f"delay_sweep: delay={delay:.2f}")
        row = run_condition(
            base_config,
            f"delay_{delay:.2f}",
            overrides,
            5.0,
            max_steps,
            lap_target,
            seed,
            output_dir / "delay_dropout_logs",
        )
        row["sweep_type"] = "delay"
        row["delay_s"] = delay
        row["dropout_probability"] = 0.0
        rows.append(row)
    for dropout in (0.0, 0.03, 0.05, 0.10):
        overrides = dict(LIDAR_CONDITIONS["noise"])
        overrides["scan_rate_hz"] = 10.0
        overrides["dropout_probability"] = dropout
        print(f"dropout_sweep: dropout={dropout:.2f}")
        row = run_condition(
            base_config,
            f"dropout_{dropout:.2f}",
            overrides,
            5.0,
            max_steps,
            lap_target,
            seed,
            output_dir / "delay_dropout_logs",
        )
        row["sweep_type"] = "dropout"
        row["delay_s"] = 0.0
        row["dropout_probability"] = dropout
        rows.append(row)
    path = write_rows(output_dir / "delay_dropout_sweep.csv", rows)
    return path


def evaluate_realtime_candidate(
    realtime_config, output_dir, seeds, lap_target, max_steps=0
):
    """欠損処理追加後の3.0 m/s短縮設定を正式候補として再評価する。"""
    rows = []
    for condition, overrides in LIDAR_CONDITIONS.items():
        for seed in seeds:
            print(f"realtime_candidate: condition={condition}, seed={seed}")
            row = run_condition(
                realtime_config,
                condition,
                overrides,
                3.0,
                max_steps,
                lap_target,
                seed,
                output_dir / "realtime_candidate_logs",
                controller_samples=64,
                localizer_xy_candidates=3,
                localizer_theta_candidates=3,
                localizer_update_mode="on_scan_update",
            )
            row["condition_group"] = condition
            rows.append(row)
    raw_path = write_rows(output_dir / "realtime_candidate_runs.csv", rows)
    summary = aggregate(rows, "condition_group")
    summary_path = write_rows(
        output_dir / "realtime_candidate_summary.csv", summary
    )
    return raw_path, summary_path


def evaluate_mppi_tuning(
    base_config,
    output_dir,
    seeds,
    lap_target,
    max_steps=0,
    variants=None,
):
    """5 m/s向けMPPI候補を同じLiDAR条件で比較する。"""

    rows = []
    conditions = build_10hz_conditions()
    selected_variants = MPPI_TUNING_VARIANTS
    if variants is not None:
        selected_variants = {
            name: MPPI_TUNING_VARIANTS[name]
            for name in variants
        }
    for variant, controller_overrides in selected_variants.items():
        for condition in (
            "ideal_10hz",
            "delay_only_10hz",
            "dropout_only_10hz",
            "noise_delay_dropout_10hz",
        ):
            for seed in seeds:
                print(
                    f"mppi_tuning: variant={variant}, "
                    f"condition={condition}, seed={seed}"
                )
                row = run_condition(
                    base_config,
                    f"{variant}_{condition}",
                    conditions[condition],
                    5.0,
                    max_steps,
                    lap_target,
                    seed,
                    output_dir / "mppi_tuning_logs",
                    localizer_scan_beams=90,
                    localizer_xy_candidates=3,
                    localizer_theta_candidates=3,
                    localizer_update_mode="on_scan_update",
                    localizer_scan_time_compensation=True,
                    localizer_scan_motion_history_compensation=True,
                    localizer_motion_history_seconds=1.0,
                    **controller_overrides,
                )
                row["variant"] = variant
                row["condition_group"] = condition
                rows.append(row)
    raw_path = write_rows(output_dir / "mppi_tuning_runs.csv", rows)
    summary = aggregate_by_keys(rows, ("variant", "condition_group"))
    summary_path = write_rows(output_dir / "mppi_tuning_summary.csv", summary)
    return raw_path, summary_path


def main():
    parser = argparse.ArgumentParser(description="LiDAR・MPPI追跡評価")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument(
        "--realtime-config",
        default=(
            "experiments/configs/"
            "homur_oval_localized_a1_mppi_realtime_candidate_3mps.yaml"
        ),
    )
    parser.add_argument("--mount-config", default=DEFAULT_MOUNT)
    parser.add_argument("--seeds", default="123,456,789")
    parser.add_argument("--lap-target", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument(
        "--tuning-seeds",
        default=None,
        help="MPPI調整評価だけに使うseed。未指定なら--seedsを使う。",
    )
    parser.add_argument(
        "--tuning-variants",
        default=None,
        help="MPPI候補名をカンマ区切りで限定する。",
    )
    parser.add_argument(
        "--output-dir", default="experiments/results/lidar_mppi_followup"
    )
    args = parser.parse_args()

    base_config = load_config(resolve_repo_path(args.config))
    realtime_config = load_config(resolve_repo_path(args.realtime_config))
    output_dir = resolve_repo_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    seeds = [int(value.strip()) for value in args.seeds.split(",") if value.strip()]
    if not seeds:
        raise ValueError("--seeds には少なくとも1つ指定してください")

    ten_hz = evaluate_10hz(
        base_config, output_dir, seeds, args.lap_target, args.max_steps
    )
    update_modes = evaluate_update_modes(
        base_config, output_dir, seeds[0], args.lap_target, args.max_steps
    )
    mount = evaluate_mount(
        base_config,
        output_dir,
        seeds[0],
        args.lap_target,
        load_mount(resolve_repo_path(args.mount_config)),
        args.max_steps,
    )
    delay_dropout = evaluate_delay_dropout_sweep(
        base_config, output_dir, seeds[0], args.lap_target, args.max_steps
    )
    realtime = evaluate_realtime_candidate(
        realtime_config,
        output_dir,
        seeds,
        args.lap_target,
        args.max_steps,
    )
    tuning_seed_text = args.tuning_seeds or args.seeds
    tuning_seeds = [
        int(value.strip())
        for value in tuning_seed_text.split(",")
        if value.strip()
    ]
    tuning_variant_text = args.tuning_variants
    tuning_variants = None
    if tuning_variant_text:
        tuning_variants = [
            value.strip()
            for value in tuning_variant_text.split(",")
            if value.strip()
        ]
        unknown = sorted(set(tuning_variants) - set(MPPI_TUNING_VARIANTS))
        if unknown:
            raise ValueError(
                "unknown --tuning-variants: " + ", ".join(unknown)
            )
    tuning = evaluate_mppi_tuning(
        base_config,
        output_dir,
        tuning_seeds,
        args.lap_target,
        args.max_steps,
        variants=tuning_variants,
    )
    print(f"speed5_10hz_raw_csv: {ten_hz[0]}")
    print(f"speed5_10hz_summary_csv: {ten_hz[1]}")
    print(f"localizer_update_modes_csv: {update_modes}")
    print(f"mount_sensitivity_csv: {mount}")
    print(f"delay_dropout_sweep_csv: {delay_dropout}")
    print(f"realtime_candidate_raw_csv: {realtime[0]}")
    print(f"realtime_candidate_summary_csv: {realtime[1]}")
    print(f"mppi_tuning_raw_csv: {tuning[0]}")
    print(f"mppi_tuning_summary_csv: {tuning[1]}")


if __name__ == "__main__":
    main()
