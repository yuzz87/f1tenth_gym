from argparse import ArgumentParser
import csv
import glob
import math
from pathlib import Path


def load_rows(csv_path):
    with open(csv_path, newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def summarize_csv(csv_path):
    rows = load_rows(csv_path)
    if not rows:
        raise ValueError(f"CSV has no data rows: {csv_path}")

    last = rows[-1]
    cross_track = [abs(float(row["cross_track_error"])) for row in rows]
    heading = [abs(float(row["heading_error"])) for row in rows]
    xy_error = [math.hypot(float(row["est_error_x"]), float(row["est_error_y"])) for row in rows]
    theta_error = [abs(float(row["est_error_theta"])) for row in rows]

    return {
        "file": Path(csv_path).name,
        "final_lap_count": int(float(last.get("lap_count", 0))),
        "final_lap_time": float(last.get("lap_time", 0.0)),
        "mean_xy_error": sum(xy_error) / len(xy_error),
        "max_xy_error": max(xy_error),
        "mean_theta_error": sum(theta_error) / len(theta_error),
        "max_theta_error": max(theta_error),
        "mean_cross_track": sum(cross_track) / len(cross_track),
        "max_cross_track": max(cross_track),
        "mean_heading": sum(heading) / len(heading),
        "max_heading": max(heading),
    }


def latest_match(results_dir, pattern):
    matches = sorted(glob.glob(str(Path(results_dir) / pattern)))
    return matches[-1] if matches else None


def format_float(value):
    return f"{float(value):.6f}"


def build_markdown_report(results):
    lines = [
        "# Feature-Localized Controller Comparison",
        "",
        "| controller | source_csv | lap_count | lap_time_s | mean_xy_error_m | mean_theta_error_rad | mean_cross_track_m | max_cross_track_m |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for controller_name in ["pure_pursuit", "mpc", "mppi"]:
        result = results.get(controller_name)
        if result is None:
            lines.append(f"| {controller_name} | missing | - | - | - | - | - | - |")
            continue
        lines.append(
            "| {controller} | `{file}` | {lap_count} | {lap_time} | {mean_xy} | {mean_theta} | {mean_cte} | {max_cte} |".format(
                controller=controller_name,
                file=result["file"],
                lap_count=result["final_lap_count"],
                lap_time=format_float(result["final_lap_time"]),
                mean_xy=format_float(result["mean_xy_error"]),
                mean_theta=format_float(result["mean_theta_error"]),
                mean_cte=format_float(result["mean_cross_track"]),
                max_cte=format_float(result["max_cross_track"]),
            )
        )
    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- `feature_localizer` の比較結果。",
            "- `mean_xy_error_m` は localization の位置誤差。",
            "- `mean_cross_track_m` は controller の追従誤差。",
            "",
        ]
    )
    return "\n".join(lines)


def main():
    parser = ArgumentParser(description="feature-localized Pure Pursuit / MPC / MPPI の比較レポートを作る。")
    parser.add_argument("--results-dir", default="experiments/results")
    parser.add_argument("--pure-pursuit-csv")
    parser.add_argument("--mpc-csv")
    parser.add_argument("--mppi-csv")
    parser.add_argument(
        "--output", default="experiments/results/reports/feature_controller_comparison.md"
    )
    args = parser.parse_args()

    selected = {
        "pure_pursuit": args.pure_pursuit_csv or latest_match(args.results_dir, "*feature_pure_pursuit.csv"),
        "mpc": args.mpc_csv or latest_match(args.results_dir, "*feature_mpc.csv"),
        "mppi": args.mppi_csv or latest_match(args.results_dir, "*feature_mppi.csv"),
    }

    results = {}
    for key, value in selected.items():
        results[key] = summarize_csv(value) if value is not None else None

    output_path = Path(args.output)
    output_path.write_text(build_markdown_report(results), encoding="utf-8")
    print(f"comparison_md: {output_path}")
    for key, value in selected.items():
        print(f"{key}_csv: {value}")


if __name__ == "__main__":
    main()
