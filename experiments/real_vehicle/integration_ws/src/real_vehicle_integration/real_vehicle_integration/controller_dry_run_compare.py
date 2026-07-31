"""Compare saved MPC and MPPI real-sensor dry-run summaries."""

import argparse
from collections import defaultdict
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import statistics

from .repository import find_repository_root


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _mean(values):
    finite = [value for value in values if value is not None]
    return statistics.mean(finite) if finite else None


def load_trial_summary(path):
    """Load one trial directory or summary JSON."""

    path = Path(path).expanduser().resolve()
    summary_path = path / "summary.json" if path.is_dir() else path
    if not summary_path.is_file():
        raise FileNotFoundError(f"summary.json not found: {summary_path}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if int(summary.get("recording_format_version", 0)) < 1:
        raise ValueError(f"not a Phase 6 summary: {summary_path}")
    return summary_path, summary


def compare_summaries(entries):
    """Aggregate saved trials by controller and fault mode."""

    rows = []
    groups = defaultdict(list)
    for summary_path, summary in entries:
        row = {
            "summary_path": str(summary_path),
            "trial_name": summary_path.parent.name,
            "controller_type": str(summary.get("controller_type", "")),
            "fault_mode": str(summary.get("fault_mode", "")),
            "status": str(summary.get("status", "")),
            "duration_s": _finite(summary.get("actual_duration_s")),
            "solve_mean_ms": _finite(
                summary.get("controller_solve_time_ms_mean")
            ),
            "solve_p95_ms": _finite(
                summary.get("controller_solve_time_ms_p95")
            ),
            "solve_max_ms": _finite(
                summary.get("controller_solve_time_ms_max")
            ),
            "overrun_rate": _finite(
                summary.get("controller_overrun_rate")
            ),
            "deadline_rate": _finite(
                summary.get("controller_deadline_rate")
            ),
            "localized_rate": _finite(summary.get("localized_rate")),
            "hardware_path_absent": bool(
                summary.get("hardware_path_absent", False)
            ),
            "fault_response_s": _finite(
                summary.get("fault_response_s")
            ),
        }
        rows.append(row)
        groups[(row["controller_type"], row["fault_mode"])].append(row)

    aggregate = []
    for (controller_type, fault_mode), trials in sorted(groups.items()):
        passed = sum(row["status"] == "passed" for row in trials)
        aggregate.append({
            "controller_type": controller_type,
            "fault_mode": fault_mode,
            "trials": len(trials),
            "passed": passed,
            "pass_rate": passed / len(trials),
            "solve_mean_ms": _mean([
                row["solve_mean_ms"] for row in trials
            ]),
            "solve_p95_ms_mean": _mean([
                row["solve_p95_ms"] for row in trials
            ]),
            "solve_max_ms": max(
                (
                    row["solve_max_ms"]
                    for row in trials
                    if row["solve_max_ms"] is not None
                ),
                default=None,
            ),
            "overrun_rate_mean": _mean([
                row["overrun_rate"] for row in trials
            ]),
            "deadline_rate_mean": _mean([
                row["deadline_rate"] for row in trials
            ]),
            "localized_rate_mean": _mean([
                row["localized_rate"] for row in trials
            ]),
            "hardware_isolation_pass_rate": (
                sum(row["hardware_path_absent"] for row in trials)
                / len(trials)
            ),
            "fault_response_s_mean": _mean([
                row["fault_response_s"] for row in trials
            ]),
        })

    baseline = [
        row for row in aggregate if row["fault_mode"] == "none"
    ]
    eligible = [
        row for row in baseline
        if row["pass_rate"] == 1.0
        and row["hardware_isolation_pass_rate"] == 1.0
    ]
    recommended = ""
    recommendation_reason = (
        "No controller has a complete passing baseline."
    )
    if eligible:
        selected = min(
            eligible,
            key=lambda row: (
                float("inf")
                if row["solve_p95_ms_mean"] is None
                else row["solve_p95_ms_mean"],
                float("inf")
                if row["solve_max_ms"] is None
                else row["solve_max_ms"],
                row["controller_type"],
            ),
        )
        recommended = selected["controller_type"]
        recommendation_reason = (
            "Selected from controllers with 100% baseline and hardware "
            "isolation pass rates using the lowest mean p95 solve time."
        )

    return {
        "status": (
            "passed"
            if rows and all(row["status"] == "passed" for row in rows)
            else "incomplete_or_failed"
        ),
        "trial_count": len(rows),
        "trials": rows,
        "aggregate": aggregate,
        "recommended_controller": recommended,
        "recommendation_reason": recommendation_reason,
    }


def _write_csv(path, rows):
    fields = (
        "summary_path",
        "trial_name",
        "controller_type",
        "fault_mode",
        "status",
        "duration_s",
        "solve_mean_ms",
        "solve_p95_ms",
        "solve_max_ms",
        "overrun_rate",
        "deadline_rate",
        "localized_rate",
        "hardware_path_absent",
        "fault_response_s",
    )
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _parse_args(args=None):
    root = find_repository_root()
    default_root = (
        root
        / "experiments/real_vehicle/results/"
        "phase6_controller_dry_run/trials"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trial-dir",
        action="append",
        type=Path,
        default=[],
    )
    parser.add_argument(
        "--results-root",
        type=Path,
        default=default_root,
    )
    parser.add_argument("--include-failed", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args(args)


def main(args=None):
    options = _parse_args(args)
    paths = list(options.trial_dir)
    if not paths:
        paths = sorted(
            path.parent
            for path in options.results_root.expanduser().glob(
                "*/summary.json"
            )
        )
    entries = [load_trial_summary(path) for path in paths]
    if not options.include_failed:
        entries = [
            entry for entry in entries
            if entry[1].get("status") == "passed"
        ]
    if not entries:
        raise SystemExit("no Phase 6 trial summaries were selected")

    result = compare_summaries(entries)
    output = options.output_dir
    if output is None:
        output = (
            find_repository_root()
            / "experiments/real_vehicle/results/"
            "phase6_controller_dry_run/comparisons/"
            / datetime.now().strftime("%Y%m%d_%H%M%S")
        )
    output = output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / "comparison.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _write_csv(output / "trials.csv", result["trials"])
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"output_dir: {output}")
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
