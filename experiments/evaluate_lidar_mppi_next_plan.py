"""実機なしで行うLiDAR・MPPI次段階評価。"""

import argparse
import copy
from pathlib import Path
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_high_speed_localized_mppi import (  # noqa: E402
    LIDAR_CONDITIONS,
    load_config as load_localized_config,
    run_condition as run_localized_condition,
)
from experiments.compare_high_speed_mppi import (  # noqa: E402
    load_config as load_truth_config,
    run_condition as run_truth_condition,
)
from experiments.evaluate_lidar_mppi_followup import (  # noqa: E402
    aggregate_by_keys,
    build_10hz_conditions,
    write_rows,
)
from experiments.run_controller_experiment import resolve_repo_path  # noqa: E402


THREE_MPS_CONFIG = (
    "experiments/configs/"
    "homur_oval_localized_a1_mppi_realtime_candidate_3mps.yaml"
)
FIVE_MPS_CONFIG = (
    "experiments/configs/"
    "homur_oval_localized_a1_mppi_high_speed_5mps.yaml"
)


TRUTH_6P1_VARIANTS = {
    "h8_s64_full": {
        "horizon": 8,
        "num_samples": 64,
        "target_speed": 6.1,
        "vgain": 6.1,
    },
    "h8_s128_full": {
        "horizon": 8,
        "num_samples": 128,
        "target_speed": 6.1,
        "vgain": 6.1,
    },
    "h12_s64_full": {
        "horizon": 12,
        "num_samples": 64,
        "target_speed": 6.1,
        "vgain": 6.1,
    },
    "h12_s128_full": {
        "horizon": 12,
        "num_samples": 128,
        "target_speed": 6.1,
        "vgain": 6.1,
    },
    "h12_s64_target5p5": {
        "horizon": 12,
        "num_samples": 64,
        "target_speed": 5.5,
        "vgain": 5.5,
    },
    "h12_s64_target5p5_limited": {
        "horizon": 12,
        "num_samples": 64,
        "target_speed": 5.5,
        "vgain": 5.5,
        "steer_limit": 0.36,
    },
}

TRUTH_SPEED_LADDER = (4.0, 4.5, 4.8, 5.0, 5.5, 6.1)


def _controller_shape(config):
    controller = config.get("controller") or {}
    return int(controller.get("horizon", 12)), int(
        controller.get("num_samples", 128)
    )


def _localizer_shape(config):
    localizer = config.get("localizer") or {}
    return {
        "localizer_scan_beams": int(localizer.get("scan_beams", 90)),
        "localizer_xy_candidates": int(localizer.get("xy_candidates", 3)),
        "localizer_theta_candidates": int(
            localizer.get("theta_candidates", 3)
        ),
    }


def _prepare_localized_kwargs(config):
    horizon, samples = _controller_shape(config)
    controller = config.get("controller") or {}
    kwargs = {
        "controller_horizon": horizon,
        "controller_samples": samples,
        "controller_target_speed": float(
            controller.get("target_speed", 1.0)
        ),
        "controller_vgain": float(controller.get("vgain", 1.0)),
        "controller_noise_sigma": float(
            controller.get("noise_sigma", 0.08)
        ),
        **_localizer_shape(config),
        "localizer_update_mode": "on_scan_update",
        "localizer_scan_time_compensation": True,
        "localizer_scan_motion_history_compensation": True,
        "localizer_motion_history_seconds": 1.0,
    }
    if "steer_limit" in controller:
        kwargs["controller_steer_limit"] = float(controller["steer_limit"])
    return kwargs


def build_robustness_conditions():
    """5 m/sの遅延・欠損・複合条件を作る。"""

    conditions = {}
    for delay in (0.0, 0.02, 0.05, 0.10, 0.15):
        overrides = dict(LIDAR_CONDITIONS["noise"])
        overrides.update(
            {
                "scan_rate_hz": 10.0,
                "scan_delay": delay,
                "dropout_probability": 0.0,
            }
        )
        conditions[f"delay_{delay:.2f}"] = overrides

    for dropout in (0.0, 0.03, 0.05, 0.10, 0.20):
        overrides = dict(LIDAR_CONDITIONS["noise"])
        overrides.update(
            {
                "scan_rate_hz": 10.0,
                "scan_delay": 0.0,
                "dropout_probability": dropout,
            }
        )
        conditions[f"dropout_{dropout:.2f}"] = overrides

    for delay in (0.0, 0.05, 0.10):
        for dropout in (0.03, 0.10, 0.20):
            overrides = dict(LIDAR_CONDITIONS["noise"])
            overrides.update(
                {
                    "scan_rate_hz": 10.0,
                    "scan_delay": delay,
                    "dropout_probability": dropout,
                }
            )
            conditions[f"combined_d{delay:.2f}_p{dropout:.2f}"] = overrides
    return conditions


def evaluate_long_runs(configs, seeds, lap_target, output_dir, max_steps=0):
    rows = []
    cases = (
        (
            "3mps",
            configs["3mps"],
            3.0,
            LIDAR_CONDITIONS,
        ),
        (
            "5mps",
            configs["5mps"],
            5.0,
            {
                key: value
                for key, value in build_10hz_conditions().items()
                if key in (
                    "ideal_10hz",
                    "noise_10hz",
                    "noise_delay_dropout_10hz",
                )
            },
        ),
    )
    for speed_group, config, speed, conditions in cases:
        kwargs = _prepare_localized_kwargs(config)
        for condition, lidar_overrides in conditions.items():
            for seed in seeds:
                print(
                    f"long: speed={speed_group}, condition={condition}, "
                    f"seed={seed}"
                )
                row = run_localized_condition(
                    config,
                    condition,
                    lidar_overrides,
                    speed,
                    max_steps,
                    lap_target,
                    seed,
                    output_dir / "long_logs",
                    **kwargs,
                )
                row["speed_group"] = speed_group
                row["condition_group"] = condition
                row["evaluation"] = "long_run"
                rows.append(row)
    raw_path = write_rows(output_dir / "long_runs.csv", rows)
    summary = aggregate_by_keys(rows, ("speed_group", "condition_group"))
    summary_path = write_rows(output_dir / "long_summary.csv", summary)
    return raw_path, summary_path


def evaluate_robustness(config, seeds, lap_target, output_dir, max_steps=0):
    rows = []
    kwargs = _prepare_localized_kwargs(config)
    for condition, lidar_overrides in build_robustness_conditions().items():
        for seed in seeds:
            print(f"robustness: condition={condition}, seed={seed}")
            row = run_localized_condition(
                config,
                condition,
                lidar_overrides,
                5.0,
                max_steps,
                lap_target,
                seed,
                output_dir / "robustness_logs",
                **kwargs,
            )
            row["condition_group"] = condition
            row["delay_s"] = float(lidar_overrides["scan_delay"])
            row["dropout_probability"] = float(
                lidar_overrides["dropout_probability"]
            )
            row["evaluation"] = "robustness"
            rows.append(row)
    raw_path = write_rows(output_dir / "robustness_runs.csv", rows)
    summary = aggregate_by_keys(rows, ("condition_group",))
    summary_path = write_rows(output_dir / "robustness_summary.csv", summary)
    return raw_path, summary_path


def evaluate_truth_6p1(config, seeds, lap_target, output_dir, max_steps=0):
    rows = []
    for variant, settings in TRUTH_6P1_VARIANTS.items():
        horizon_settings = {
            "horizon": settings["horizon"],
            "num_samples": settings["num_samples"],
        }
        controller_settings = {
            key: value
            for key, value in settings.items()
            if key not in ("horizon", "num_samples")
        }
        for seed in seeds:
            print(f"truth_6p1: variant={variant}, seed={seed}")
            row = run_truth_condition(
                config,
                variant,
                horizon_settings,
                6.1,
                max_steps,
                lap_target,
                seed,
                output_dir / "truth_6p1_logs",
                controller_overrides=controller_settings,
            )
            row["variant"] = variant
            row["evaluation"] = "truth_pose_6p1"
            rows.append(row)
    raw_path = write_rows(output_dir / "truth_6p1_runs.csv", rows)
    summary = aggregate_by_keys(
        rows,
        ("variant",),
        metrics=(
            "lap_target_reached",
            "collision_count",
            "mean_speed_mps",
            "mean_abs_cross_track_m",
            "mean_plan_time_ms",
        ),
    )
    summary_path = write_rows(output_dir / "truth_6p1_summary.csv", summary)
    return raw_path, summary_path


def evaluate_truth_speed_ladder(config, seeds, lap_target, output_dir, max_steps=0):
    """真値姿勢でMPPIの速度上限を測る。"""

    rows = []
    for speed in TRUTH_SPEED_LADDER:
        for seed in seeds:
            print(f"truth_speed_ladder: speed={speed:.1f}, seed={seed}")
            row = run_truth_condition(
                config,
                f"speed_{speed:.1f}",
                {"horizon": 8, "num_samples": 64},
                speed,
                max_steps,
                lap_target,
                seed,
                output_dir / "truth_speed_ladder_logs",
                controller_overrides={
                    "target_speed": speed,
                    "vgain": speed,
                },
            )
            row["speed_mps"] = float(speed)
            row["evaluation"] = "truth_speed_ladder"
            rows.append(row)
    raw_path = write_rows(output_dir / "truth_speed_ladder_runs.csv", rows)
    summary = aggregate_by_keys(
        rows,
        ("speed_mps",),
        metrics=(
            "lap_target_reached",
            "collision_count",
            "mean_speed_mps",
            "mean_abs_cross_track_m",
            "mean_plan_time_ms",
        ),
    )
    summary_path = write_rows(
        output_dir / "truth_speed_ladder_summary.csv", summary
    )
    return raw_path, summary_path


def write_manifest(output_dir, args):
    manifest = {
        "hardware_experiment": False,
        "standard_configs": {
            "3mps": THREE_MPS_CONFIG,
            "5mps": FIVE_MPS_CONFIG,
        },
        "seeds": args.seeds,
        "long_laps": args.long_laps,
        "robustness_laps": args.robustness_laps,
        "truth_6p1_laps": args.truth_6p1_laps,
        "truth_speed_ladder_mps": list(TRUTH_SPEED_LADDER),
        "robustness_delays_s": [0.0, 0.02, 0.05, 0.10, 0.15],
        "robustness_dropout_probabilities": [0.0, 0.03, 0.05, 0.10, 0.20],
        "truth_6p1_variants": sorted(TRUTH_6P1_VARIANTS),
    }
    path = output_dir / "evaluation_manifest.yaml"
    with path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(manifest, file, allow_unicode=True, sort_keys=False)
    return path


def parse_seeds(text):
    seeds = [int(value.strip()) for value in text.split(",") if value.strip()]
    if not seeds:
        raise ValueError("seedを少なくとも1つ指定してください")
    return seeds


def main():
    parser = argparse.ArgumentParser(description="LiDAR・MPPI次段階評価")
    parser.add_argument("--seeds", default="123,456,789")
    parser.add_argument("--long-laps", type=int, default=2)
    parser.add_argument("--robustness-laps", type=int, default=1)
    parser.add_argument("--truth-6p1-laps", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument(
        "--output-dir",
        default="experiments/results/lidar_mppi_next_plan",
    )
    parser.add_argument("--skip-long", action="store_true")
    parser.add_argument("--skip-robustness", action="store_true")
    parser.add_argument("--skip-truth-6p1", action="store_true")
    args = parser.parse_args()

    output_dir = resolve_repo_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    seeds = parse_seeds(args.seeds)
    configs = {
        "3mps": load_localized_config(resolve_repo_path(THREE_MPS_CONFIG)),
        "5mps": load_localized_config(resolve_repo_path(FIVE_MPS_CONFIG)),
    }
    truth_config = load_truth_config(resolve_repo_path(FIVE_MPS_CONFIG))
    manifest_path = write_manifest(output_dir, args)
    print(f"manifest: {manifest_path}")

    if not args.skip_long:
        long_paths = evaluate_long_runs(
            configs,
            seeds,
            args.long_laps,
            output_dir,
            args.max_steps,
        )
        print(f"long_raw_csv: {long_paths[0]}")
        print(f"long_summary_csv: {long_paths[1]}")
    if not args.skip_robustness:
        robust_paths = evaluate_robustness(
            configs["5mps"],
            seeds,
            args.robustness_laps,
            output_dir,
            args.max_steps,
        )
        print(f"robustness_raw_csv: {robust_paths[0]}")
        print(f"robustness_summary_csv: {robust_paths[1]}")
    if not args.skip_truth_6p1:
        truth_paths = evaluate_truth_6p1(
            truth_config,
            seeds,
            args.truth_6p1_laps,
            output_dir,
            args.max_steps,
        )
        print(f"truth_6p1_raw_csv: {truth_paths[0]}")
        print(f"truth_6p1_summary_csv: {truth_paths[1]}")
        ladder_paths = evaluate_truth_speed_ladder(
            truth_config,
            seeds,
            args.truth_6p1_laps,
            output_dir,
            args.max_steps,
        )
        print(f"truth_speed_ladder_raw_csv: {ladder_paths[0]}")
        print(f"truth_speed_ladder_summary_csv: {ladder_paths[1]}")


if __name__ == "__main__":
    main()
