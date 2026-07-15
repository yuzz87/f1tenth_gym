from argparse import ArgumentParser
import csv
import glob
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

    return {
        "file": Path(csv_path).name,
        "rows": len(rows),
        "final_lap_count": int(float(last.get("lap_count", 0))),
        "final_lap_time": float(last.get("lap_time", 0.0)),
        "sim_time_final": float(last["sim_time"]),
        "max_abs_cross_track": max(cross_track),
        "mean_abs_cross_track": sum(cross_track) / len(cross_track),
        "max_abs_heading": max(heading),
        "mean_abs_heading": sum(heading) / len(heading),
    }


def latest_match(results_dir, pattern):
    matches = sorted(glob.glob(str(Path(results_dir) / pattern)))
    if not matches:
        return None
    return matches[-1]


def find_default_inputs(results_dir):
    return {
        "pure_pursuit": latest_match(results_dir, "*pure_pursuit.csv")
        or latest_match(results_dir, "*homur_oval.csv"),
        "mpc": latest_match(results_dir, "*mpc.csv"),
        "mppi": latest_match(results_dir, "*mppi.csv"),
    }


def format_float(value):
    return f"{float(value):.6f}"


def build_markdown_report(results):
    lines = [
        "# Controller Comparison",
        "",
        "| controller | source_csv | lap_count | lap_time_s | max_abs_cross_track_m | mean_abs_cross_track_m | max_abs_heading_rad | mean_abs_heading_rad |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]

    order = ["pure_pursuit", "mpc", "mppi"]
    for controller_name in order:
        result = results.get(controller_name)
        if result is None:
            lines.append(
                f"| {controller_name} | missing | - | - | - | - | - | - |"
            )
            continue
        lines.append(
            "| {controller} | `{file}` | {lap_count} | {lap_time} | {max_cte} | {mean_cte} | {max_heading} | {mean_heading} |".format(
                controller=controller_name,
                file=result["file"],
                lap_count=result["final_lap_count"],
                lap_time=format_float(result["final_lap_time"]),
                max_cte=format_float(result["max_abs_cross_track"]),
                mean_cte=format_float(result["mean_abs_cross_track"]),
                max_heading=format_float(result["max_abs_heading"]),
                mean_heading=format_float(result["mean_abs_heading"]),
            )
        )

    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- `lap_count = 1` なら 1 周完了。",
            "- `max_abs_cross_track_m` と `mean_abs_cross_track_m` が小さいほど中心線追従が安定。",
            "- `max_abs_heading_rad` と `mean_abs_heading_rad` は姿勢のずれの目安。",
            "",
        ]
    )
    return "\n".join(lines)


def main():
    parser = ArgumentParser(description="Pure Pursuit / MPC / MPPI の最新ログを比較する。")
    parser.add_argument(
        "--results-dir",
        default="experiments/results",
        help="比較対象の CSV が入っているフォルダ。",
    )
    parser.add_argument("--pure-pursuit-csv", help="Pure Pursuit の CSV を明示指定する。")
    parser.add_argument("--mpc-csv", help="MPC の CSV を明示指定する。")
    parser.add_argument("--mppi-csv", help="MPPI の CSV を明示指定する。")
    parser.add_argument(
        "--output",
        default="experiments/results/reports/controller_comparison.md",
        help="出力する Markdown レポートのパス。",
    )
    args = parser.parse_args()

    defaults = find_default_inputs(args.results_dir)
    selected = {
        "pure_pursuit": args.pure_pursuit_csv or defaults["pure_pursuit"],
        "mpc": args.mpc_csv or defaults["mpc"],
        "mppi": args.mppi_csv or defaults["mppi"],
    }

    summaries = {}
    for controller_name, csv_path in selected.items():
        if csv_path is None:
            summaries[controller_name] = None
            continue
        summaries[controller_name] = summarize_csv(csv_path)

    output_path = Path(args.output)
    report = build_markdown_report(summaries)
    output_path.write_text(report, encoding="utf-8")
    print(f"comparison_md: {output_path}")
    for controller_name, csv_path in selected.items():
        print(f"{controller_name}_csv: {csv_path}")


if __name__ == "__main__":
    main()
