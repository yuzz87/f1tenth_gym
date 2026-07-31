"""Analyze Phase 3 encoder measurements without driving hardware."""

import argparse
import csv
import json
import math
from pathlib import Path


DEFAULT_WHEEL_DIAMETER_M = 0.066
DEFAULT_COUNTS_PER_REVOLUTION = 144.0


def load_encoder_csv(path):
    """Load numeric encoder rows from a Phase 1 receiver CSV."""

    rows = []
    with Path(path).open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            rows.append({
                "sequence": int(row["sequence"]),
                "cumulative_count": int(row["cumulative_count"]),
                "delta_count": int(row["delta_count"]),
                "direction": int(row["direction"]),
                "invalid_transition_count": int(
                    row["invalid_transition_count"]
                ),
                "pulse_age_s": float(row["pulse_age_s"]),
            })
    if not rows:
        raise ValueError("encoder CSV contains no rows")
    return rows


def summarize_rows(rows, wheel_diameter_m=DEFAULT_WHEEL_DIAMETER_M,
                   counts_per_revolution=DEFAULT_COUNTS_PER_REVOLUTION):
    counts = [row["cumulative_count"] for row in rows]
    directions = {direction: 0 for direction in (-1, 0, 1)}
    for row in rows:
        directions[row["direction"]] = directions.get(row["direction"], 0) + 1
    distance_per_count = math.pi * float(wheel_diameter_m) / float(
        counts_per_revolution
    )
    return {
        "rows": len(rows),
        "first_sequence": rows[0]["sequence"],
        "last_sequence": rows[-1]["sequence"],
        "first_count": counts[0],
        "last_count": counts[-1],
        "min_count": min(counts),
        "max_count": max(counts),
        "direction_samples": directions,
        "max_invalid_transition_count": max(
            row["invalid_transition_count"] for row in rows
        ),
        "max_pulse_age_s": max(row["pulse_age_s"] for row in rows),
        "provisional_distance_per_count_m": distance_per_count,
        "provisional_count_per_meter": 1.0 / distance_per_count,
    }


def wheel_turn_calibration(start_count, end_count, turns,
                           wheel_diameter_m=DEFAULT_WHEEL_DIAMETER_M):
    turns = float(turns)
    if turns <= 0.0:
        raise ValueError("turns must be positive")
    wheel_diameter_m = float(wheel_diameter_m)
    if wheel_diameter_m <= 0.0:
        raise ValueError("wheel_diameter_m must be positive")
    count_delta = abs(int(end_count) - int(start_count))
    if count_delta <= 0:
        raise ValueError("start_count and end_count must differ")
    counts_per_revolution = count_delta / turns
    distance_per_count_m = math.pi * wheel_diameter_m / counts_per_revolution
    return {
        "start_count": int(start_count),
        "end_count": int(end_count),
        "count_delta": count_delta,
        "turns": turns,
        "counts_per_revolution": counts_per_revolution,
        "distance_per_count_m": distance_per_count_m,
        "effective_wheel_diameter_m": wheel_diameter_m,
        "calibration_status": "wheel_turn_measurement",
    }


def known_distance_calibration(start_count, end_count, distance_m,
                               counts_per_revolution=DEFAULT_COUNTS_PER_REVOLUTION):
    distance_m = float(distance_m)
    counts_per_revolution = float(counts_per_revolution)
    if distance_m <= 0.0:
        raise ValueError("distance_m must be positive")
    if counts_per_revolution <= 0.0:
        raise ValueError("counts_per_revolution must be positive")
    count_delta = abs(int(end_count) - int(start_count))
    if count_delta <= 0:
        raise ValueError("start_count and end_count must differ")
    distance_per_count_m = distance_m / count_delta
    effective_diameter_m = (
        distance_per_count_m * counts_per_revolution / math.pi
    )
    return {
        "start_count": int(start_count),
        "end_count": int(end_count),
        "count_delta": count_delta,
        "known_distance_m": distance_m,
        "counts_per_revolution_assumed": counts_per_revolution,
        "distance_per_count_m": distance_per_count_m,
        "count_per_meter": 1.0 / distance_per_count_m,
        "effective_wheel_diameter_m": effective_diameter_m,
        "calibration_status": "known_distance_measurement",
    }


def _add_count_arguments(parser):
    parser.add_argument("--start-count", type=int, required=True)
    parser.add_argument("--end-count", type=int, required=True)


def _write_output(result, output):
    print(json.dumps(result, indent=2, sort_keys=True))
    if output is not None:
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"output_json: {output}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command")

    summarize = subparsers.add_parser("summarize")
    summarize.add_argument("--csv", type=Path, required=True)
    summarize.add_argument("--wheel-diameter-m", type=float, default=DEFAULT_WHEEL_DIAMETER_M)
    summarize.add_argument("--counts-per-revolution", type=float, default=DEFAULT_COUNTS_PER_REVOLUTION)
    summarize.add_argument("--output", type=Path, default=None)

    wheel_turns = subparsers.add_parser("wheel-turns")
    _add_count_arguments(wheel_turns)
    wheel_turns.add_argument("--turns", type=float, required=True)
    wheel_turns.add_argument("--wheel-diameter-m", type=float, default=DEFAULT_WHEEL_DIAMETER_M)
    wheel_turns.add_argument("--output", type=Path, default=None)

    known_distance = subparsers.add_parser("known-distance")
    _add_count_arguments(known_distance)
    known_distance.add_argument("--distance-m", type=float, required=True)
    known_distance.add_argument("--counts-per-revolution", type=float, default=DEFAULT_COUNTS_PER_REVOLUTION)
    known_distance.add_argument("--output", type=Path, default=None)

    args = parser.parse_args(argv)
    if args.command == "summarize":
        _write_output(
            summarize_rows(
                load_encoder_csv(args.csv),
                args.wheel_diameter_m,
                args.counts_per_revolution,
            ),
            args.output,
        )
        return 0
    if args.command == "wheel-turns":
        _write_output(
            wheel_turn_calibration(
                args.start_count,
                args.end_count,
                args.turns,
                args.wheel_diameter_m,
            ),
            args.output,
        )
        return 0
    if args.command == "known-distance":
        _write_output(
            known_distance_calibration(
                args.start_count,
                args.end_count,
                args.distance_m,
                args.counts_per_revolution,
            ),
            args.output,
        )
        return 0
    parser.error("choose one of: summarize, wheel-turns, known-distance")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
