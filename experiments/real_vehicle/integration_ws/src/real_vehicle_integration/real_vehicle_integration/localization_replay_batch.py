"""Replay multiple localization trials and select a deployment preset."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import statistics

from .localization_replay import REPLAY_PRESETS, compare_trial


AGGREGATE_FIELDS = (
    "preset",
    "trial_type",
    "status",
    "trial_count",
    "passed_trials",
    "pass_rate",
    "pose_odom_difference_mean_m",
    "pose_odom_difference_max_m",
    "expected_distance_error_mean_m",
    "expected_distance_error_max_m",
    "expected_yaw_error_mean_deg",
    "expected_yaw_error_max_deg",
    "position_drift_mean_m",
    "position_drift_max_m",
    "processing_time_mean_ms",
    "processing_time_max_ms",
    "correction_events_total",
    "correction_events_mean",
    "acceptance_failures",
)


def _finite_values(summaries, key):
    values = []
    for summary in summaries:
        value = summary.get(key)
        if value is None:
            continue
        value = float(value)
        if math.isfinite(value):
            values.append(value)
    return values


def _aggregate_preset(
    preset,
    summaries,
    trial_type,
    minimum_pass_rate,
    maximum_mean_pose_odom_difference_m,
    maximum_trial_pose_odom_difference_m,
    maximum_mean_processing_time_ms,
    maximum_mean_yaw_error_rad,
    maximum_trial_yaw_error_rad,
    maximum_mean_position_drift_m,
    maximum_trial_position_drift_m,
):
    trial_count = len(summaries)
    passed_trials = sum(
        summary.get("status") == "passed" for summary in summaries
    )
    pass_rate = passed_trials / trial_count if trial_count else 0.0
    differences = _finite_values(
        summaries,
        "pose_odom_distance_difference_m",
    )
    expected_errors = _finite_values(
        summaries,
        "localizer_expected_distance_error_m",
    )
    yaw_errors = _finite_values(
        summaries,
        "expected_yaw_error_rad",
    )
    position_drifts = _finite_values(
        summaries,
        "position_drift_m",
    )
    processing_means = _finite_values(
        summaries,
        "processing_time_ms_mean",
    )
    processing_maxima = _finite_values(
        summaries,
        "processing_time_ms_max",
    )
    correction_events = [
        int(summary.get("correction_events", 0))
        for summary in summaries
    ]
    reasons = Counter()
    for summary in summaries:
        reasons.update(summary.get("selection_reason_counts", {}))

    failures = []
    if trial_count == 0:
        failures.append("no trials were evaluated")
    if pass_rate + 1e-12 < minimum_pass_rate:
        failures.append(
            f"pass rate {pass_rate:.6f} is below "
            f"{minimum_pass_rate:.6f}"
        )
    if trial_type == "manual_rotation":
        difference_mean = None
        difference_max = None
    else:
        if len(differences) != trial_count:
            failures.append("pose/odometry difference is unavailable")
            difference_mean = None
            difference_max = None
        else:
            difference_mean = statistics.mean(differences)
            difference_max = max(differences)
            if (
                difference_mean
                > maximum_mean_pose_odom_difference_m + 1e-12
            ):
                failures.append(
                    f"mean pose/odometry difference "
                    f"{difference_mean:.6f} m exceeds "
                    f"{maximum_mean_pose_odom_difference_m:.6f} m"
                )
            if (
                difference_max
                > maximum_trial_pose_odom_difference_m + 1e-12
            ):
                failures.append(
                    f"maximum pose/odometry difference "
                    f"{difference_max:.6f} m exceeds "
                    f"{maximum_trial_pose_odom_difference_m:.6f} m"
                )

    expected_mean = (
        statistics.mean(expected_errors)
        if len(expected_errors) == trial_count
        else None
    )
    expected_max = (
        max(expected_errors)
        if len(expected_errors) == trial_count
        else None
    )
    if trial_type == "manual_rotation":
        if len(yaw_errors) != trial_count:
            failures.append("expected-yaw error is unavailable")
            yaw_error_mean = None
            yaw_error_max = None
        else:
            yaw_error_mean = statistics.mean(yaw_errors)
            yaw_error_max = max(yaw_errors)
            if yaw_error_mean > maximum_mean_yaw_error_rad + 1e-12:
                failures.append(
                    f"mean expected-yaw error "
                    f"{math.degrees(yaw_error_mean):.6f} deg exceeds "
                    f"{math.degrees(maximum_mean_yaw_error_rad):.6f} deg"
                )
            if yaw_error_max > maximum_trial_yaw_error_rad + 1e-12:
                failures.append(
                    f"maximum expected-yaw error "
                    f"{math.degrees(yaw_error_max):.6f} deg exceeds "
                    f"{math.degrees(maximum_trial_yaw_error_rad):.6f} deg"
                )
        if len(position_drifts) != trial_count:
            failures.append("position drift is unavailable")
            position_drift_mean = None
            position_drift_max = None
        else:
            position_drift_mean = statistics.mean(position_drifts)
            position_drift_max = max(position_drifts)
            if (
                position_drift_mean
                > maximum_mean_position_drift_m + 1e-12
            ):
                failures.append(
                    f"mean position drift "
                    f"{position_drift_mean:.6f} m exceeds "
                    f"{maximum_mean_position_drift_m:.6f} m"
                )
            if (
                position_drift_max
                > maximum_trial_position_drift_m + 1e-12
            ):
                failures.append(
                    f"maximum position drift "
                    f"{position_drift_max:.6f} m exceeds "
                    f"{maximum_trial_position_drift_m:.6f} m"
                )
    else:
        yaw_error_mean = None
        yaw_error_max = None
        position_drift_mean = None
        position_drift_max = None
    if len(processing_means) != trial_count:
        failures.append("mean processing time is unavailable")
        processing_mean = None
    else:
        processing_mean = statistics.mean(processing_means)
        if processing_mean > maximum_mean_processing_time_ms + 1e-12:
            failures.append(
                f"mean processing time {processing_mean:.6f} ms exceeds "
                f"{maximum_mean_processing_time_ms:.6f} ms"
            )

    return {
        "preset": preset,
        "trial_type": trial_type,
        "status": "failed" if failures else "passed",
        "acceptance_failures": failures,
        "trial_count": trial_count,
        "passed_trials": passed_trials,
        "pass_rate": pass_rate,
        "pose_odom_difference_mean_m": difference_mean,
        "pose_odom_difference_max_m": difference_max,
        "expected_distance_error_mean_m": expected_mean,
        "expected_distance_error_max_m": expected_max,
        "expected_yaw_error_mean_rad": yaw_error_mean,
        "expected_yaw_error_max_rad": yaw_error_max,
        "expected_yaw_error_mean_deg": (
            math.degrees(yaw_error_mean)
            if yaw_error_mean is not None
            else None
        ),
        "expected_yaw_error_max_deg": (
            math.degrees(yaw_error_max)
            if yaw_error_max is not None
            else None
        ),
        "position_drift_mean_m": position_drift_mean,
        "position_drift_max_m": position_drift_max,
        "processing_time_mean_ms": processing_mean,
        "processing_time_max_ms": (
            max(processing_maxima)
            if len(processing_maxima) == trial_count
            else None
        ),
        "correction_events_total": sum(correction_events),
        "correction_events_mean": (
            statistics.mean(correction_events)
            if correction_events else 0.0
        ),
        "selection_reason_counts": dict(sorted(reasons.items())),
    }


def _validate_trial(trial_dir):
    trial_dir = Path(trial_dir).expanduser().resolve()
    summary_path = trial_dir / "summary.json"
    if not summary_path.is_file():
        raise FileNotFoundError(f"trial has no summary.json: {trial_dir}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if bool(summary.get("interrupted", False)):
        raise ValueError(f"interrupted trial cannot be batched: {trial_dir}")
    required = ("scan.csv", "odom.csv", "pose.csv")
    missing = [name for name in required if not (trial_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(
            f"trial is missing replay data ({', '.join(missing)}): "
            f"{trial_dir}"
        )
    return trial_dir


def _write_aggregate_csv(path, aggregates):
    with Path(path).open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=AGGREGATE_FIELDS)
        writer.writeheader()
        for aggregate in aggregates.values():
            row = {
                key: aggregate.get(key)
                for key in AGGREGATE_FIELDS
            }
            row["acceptance_failures"] = json.dumps(
                aggregate["acceptance_failures"],
                separators=(",", ":"),
            )
            writer.writerow(row)


def compare_trials(
    trial_dirs,
    output_dir,
    presets=("baseline", "improved", "guarded"),
    deployment_candidates=("guarded",),
    map_yaml=None,
    minimum_pass_rate=1.0,
    maximum_mean_pose_odom_difference_m=0.03,
    maximum_trial_pose_odom_difference_m=0.05,
    maximum_mean_processing_time_ms=10.0,
    maximum_mean_yaw_error_deg=8.0,
    maximum_trial_yaw_error_deg=10.0,
    maximum_mean_position_drift_m=0.15,
    maximum_trial_position_drift_m=0.20,
):
    """Replay all trials and write aggregate deployment criteria."""

    trials = [_validate_trial(path) for path in trial_dirs]
    if not trials:
        raise ValueError("at least one --trial-dir is required")
    trial_names = [path.name for path in trials]
    if len(set(trial_names)) != len(trial_names):
        raise ValueError("trial directory names must be unique")
    trial_types = set()
    for trial in trials:
        summary = json.loads(
            (trial / "summary.json").read_text(encoding="utf-8")
        )
        trial_types.add(str(
            summary.get("trial_type", "manual_motion")
        ))
    if len(trial_types) != 1:
        raise ValueError(
            "motion and rotation trials cannot be mixed in one batch"
        )
    trial_type = trial_types.pop()
    if trial_type not in ("manual_motion", "manual_rotation"):
        raise ValueError(f"unsupported trial type: {trial_type}")
    unknown = [name for name in presets if name not in REPLAY_PRESETS]
    if unknown:
        raise ValueError(f"unknown replay presets: {', '.join(unknown)}")
    unknown_candidates = [
        name for name in deployment_candidates if name not in presets
    ]
    if unknown_candidates:
        raise ValueError(
            "deployment candidates are not selected presets: "
            + ", ".join(unknown_candidates)
        )

    output = Path(output_dir).expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(
            f"batch replay output directory is not empty: {output}"
        )
    output.mkdir(parents=True, exist_ok=True)

    trial_results = {}
    summaries_by_preset = {name: [] for name in presets}
    for trial in trials:
        trial_output = output / "trials" / trial.name
        comparison = compare_trial(
            trial,
            trial_output,
            map_yaml=map_yaml,
            presets=presets,
        )
        trial_results[trial.name] = {
            "trial_dir": str(trial),
            "comparison_json": str(trial_output / "comparison.json"),
            "preferred_preset": comparison["preferred_preset"],
            "preferred_metric": comparison["preferred_metric"],
            "preferred_by_pose_odom_distance": comparison[
                "preferred_by_pose_odom_distance"
            ],
            "preferred_by_rotation_error": comparison[
                "preferred_by_rotation_error"
            ],
        }
        for preset in presets:
            summaries_by_preset[preset].append(
                comparison["summaries"][preset]
            )

    aggregates = {
        preset: _aggregate_preset(
            preset,
            summaries_by_preset[preset],
            trial_type,
            float(minimum_pass_rate),
            float(maximum_mean_pose_odom_difference_m),
            float(maximum_trial_pose_odom_difference_m),
            float(maximum_mean_processing_time_ms),
            math.radians(float(maximum_mean_yaw_error_deg)),
            math.radians(float(maximum_trial_yaw_error_deg)),
            float(maximum_mean_position_drift_m),
            float(maximum_trial_position_drift_m),
        )
        for preset in presets
    }
    accepted = [
        name
        for name, aggregate in aggregates.items()
        if aggregate["status"] == "passed"
    ]
    accepted_candidates = [
        name for name in deployment_candidates if name in accepted
    ]

    def recommendation_key(name):
        if trial_type == "manual_rotation":
            return (
                aggregates[name]["expected_yaw_error_mean_rad"],
                aggregates[name]["position_drift_mean_m"],
                aggregates[name]["processing_time_mean_ms"],
            )
        return (
            aggregates[name]["pose_odom_difference_mean_m"],
            aggregates[name]["processing_time_mean_ms"],
        )

    recommended = (
        min(accepted_candidates, key=recommendation_key)
        if accepted_candidates else None
    )
    thresholds = {
        "minimum_pass_rate": float(minimum_pass_rate),
        "maximum_mean_pose_odom_difference_m": float(
            maximum_mean_pose_odom_difference_m
        ),
        "maximum_trial_pose_odom_difference_m": float(
            maximum_trial_pose_odom_difference_m
        ),
        "maximum_mean_processing_time_ms": float(
            maximum_mean_processing_time_ms
        ),
        "maximum_mean_yaw_error_deg": float(
            maximum_mean_yaw_error_deg
        ),
        "maximum_trial_yaw_error_deg": float(
            maximum_trial_yaw_error_deg
        ),
        "maximum_mean_position_drift_m": float(
            maximum_mean_position_drift_m
        ),
        "maximum_trial_position_drift_m": float(
            maximum_trial_position_drift_m
        ),
    }
    result = {
        "status": "passed" if recommended is not None else "failed",
        "trial_type": trial_type,
        "trial_count": len(trials),
        "trials": trial_results,
        "presets": list(presets),
        "deployment_candidates": list(deployment_candidates),
        "accepted_presets": accepted,
        "recommended_preset": recommended,
        "thresholds": thresholds,
        "aggregates": aggregates,
    }
    (output / "aggregate.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _write_aggregate_csv(output / "aggregate.csv", aggregates)
    return result


def _parse_list(value):
    return tuple(item.strip() for item in value.split(",") if item.strip())


def _parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trial-dir",
        type=Path,
        action="append",
        required=True,
        help="Repeat once for each complete recorded trial.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument(
        "--presets",
        default="baseline,improved,guarded",
    )
    parser.add_argument(
        "--deployment-candidates",
        default="guarded",
    )
    parser.add_argument("--minimum-pass-rate", type=float, default=1.0)
    parser.add_argument(
        "--maximum-mean-pose-odom-difference-m",
        type=float,
        default=0.03,
    )
    parser.add_argument(
        "--maximum-trial-pose-odom-difference-m",
        type=float,
        default=0.05,
    )
    parser.add_argument(
        "--maximum-mean-processing-time-ms",
        type=float,
        default=10.0,
    )
    parser.add_argument(
        "--maximum-mean-yaw-error-deg",
        type=float,
        default=8.0,
    )
    parser.add_argument(
        "--maximum-trial-yaw-error-deg",
        type=float,
        default=10.0,
    )
    parser.add_argument(
        "--maximum-mean-position-drift-m",
        type=float,
        default=0.15,
    )
    parser.add_argument(
        "--maximum-trial-position-drift-m",
        type=float,
        default=0.20,
    )
    return parser.parse_args(args)


def main(args=None):
    options = _parse_args(args)
    try:
        result = compare_trials(
            options.trial_dir,
            options.output_dir,
            presets=_parse_list(options.presets),
            deployment_candidates=_parse_list(
                options.deployment_candidates
            ),
            map_yaml=options.map_yaml,
            minimum_pass_rate=options.minimum_pass_rate,
            maximum_mean_pose_odom_difference_m=(
                options.maximum_mean_pose_odom_difference_m
            ),
            maximum_trial_pose_odom_difference_m=(
                options.maximum_trial_pose_odom_difference_m
            ),
            maximum_mean_processing_time_ms=(
                options.maximum_mean_processing_time_ms
            ),
            maximum_mean_yaw_error_deg=(
                options.maximum_mean_yaw_error_deg
            ),
            maximum_trial_yaw_error_deg=(
                options.maximum_trial_yaw_error_deg
            ),
            maximum_mean_position_drift_m=(
                options.maximum_mean_position_drift_m
            ),
            maximum_trial_position_drift_m=(
                options.maximum_trial_position_drift_m
            ),
        )
    except (
        FileExistsError,
        FileNotFoundError,
        json.JSONDecodeError,
        ValueError,
    ) as exc:
        print(f"batch replay failed: {exc}")
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    print(
        f"output_dir: {Path(options.output_dir).expanduser().resolve()}"
    )
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "AGGREGATE_FIELDS",
    "compare_trials",
]
