"""保存した実機LaserScanとシミュレーションscanの契約・分布を比較する。"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_recording(path):
    data = np.load(path)
    ranges = np.asarray(data["ranges"], dtype=float)
    beam_counts = np.asarray(data["beam_counts"], dtype=int)
    return ranges, beam_counts


def finite_values(ranges, beam_counts):
    values = []
    for scan, beam_count in zip(ranges, beam_counts):
        values.append(scan[:beam_count][np.isfinite(scan[:beam_count])])
    if not values:
        return np.empty(0, dtype=float)
    return np.concatenate(values)


def distribution_summary(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        raise ValueError("recording contains no finite range values")
    return {
        "count": int(values.size),
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "min": float(np.min(values)),
        "p10": float(np.percentile(values, 10)),
        "p50": float(np.percentile(values, 50)),
        "p90": float(np.percentile(values, 90)),
        "max": float(np.max(values)),
    }


def compare_recordings(real_npz, simulation_npz):
    real_ranges, real_beams = load_recording(real_npz)
    sim_data = np.load(simulation_npz)
    sim_ranges = np.asarray(sim_data["scans"], dtype=float)
    sim_beams = np.full(sim_ranges.shape[0], sim_ranges.shape[1], dtype=int)
    real_values = finite_values(real_ranges, real_beams)
    sim_values = finite_values(sim_ranges, sim_beams)
    real_summary = distribution_summary(real_values)
    sim_summary = distribution_summary(sim_values)
    return {
        "real": real_summary,
        "simulation": sim_summary,
        "mean_difference": real_summary["mean"] - sim_summary["mean"],
        "std_difference": real_summary["std"] - sim_summary["std"],
        "beam_count_match": sorted(set(real_beams.tolist())) == [sim_ranges.shape[1]],
    }


def check_geometry(summary, expected):
    errors = []
    if expected.get("beams") is not None and summary["beam_counts"] != [expected["beams"]]:
        errors.append("beam count mismatch")
    for key in ("range_min", "range_max", "angle_increment"):
        expected_value = expected.get(key)
        if expected_value is not None and not np.isclose(summary[key], expected_value, atol=1e-6):
            errors.append(f"{key} mismatch")
    return errors


def main():
    parser = argparse.ArgumentParser(description="LaserScan recording comparison")
    parser.add_argument("--recording-dir", required=True)
    parser.add_argument("--simulation-npz")
    parser.add_argument("--expected-beams", type=int, default=360)
    parser.add_argument("--expected-range-min", type=float, default=0.15)
    parser.add_argument("--expected-range-max", type=float, default=12.0)
    parser.add_argument("--output", default="laserscan_comparison.json")
    args = parser.parse_args()

    recording_dir = Path(args.recording_dir)
    summary = json.loads(
        (recording_dir / "laserscan_summary.json").read_text(encoding="utf-8")
    )
    expected = {
        "beams": args.expected_beams,
        "range_min": args.expected_range_min,
        "range_max": args.expected_range_max,
    }
    result = {
        "geometry": summary,
        "geometry_errors": check_geometry(summary, expected),
    }
    if args.simulation_npz:
        result["distribution"] = compare_recordings(
            recording_dir / "laserscan_recording.npz",
            args.simulation_npz,
        )
    output_path = Path(args.output)
    output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"geometry_errors: {len(result['geometry_errors'])}")
    print(f"comparison_json: {output_path}")


if __name__ == "__main__":
    main()
