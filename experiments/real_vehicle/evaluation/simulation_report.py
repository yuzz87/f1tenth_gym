"""Aggregate simulation campaign CSV and create figures plus Markdown."""

import argparse
import csv
from html import escape
import json
from pathlib import Path

import numpy as np


GROUP_FIELDS = (
    "controller",
    "scenario",
    "pose_source",
    "plant_profile",
    "lidar_profile",
    "network_profile",
)


def _load_rows(path):
    with Path(path).open("r", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _aggregate(rows):
    grouped = {}
    for row in rows:
        key = tuple(row[field] for field in GROUP_FIELDS)
        grouped.setdefault(key, []).append(row)
    summaries = []
    for key, values in sorted(grouped.items()):
        def numeric(field):
            return np.array([float(row[field]) for row in values])
        summary = dict(zip(GROUP_FIELDS, key))
        summary.update({
            "runs": len(values),
            "completion_rate": float(np.mean(numeric("completed"))),
            "position_rmse_mean_m": float(np.mean(numeric("position_rmse_m"))),
            "localization_xy_rmse_mean_m": float(
                np.mean(numeric("localization_xy_rmse_m"))
            ),
            "controller_mean_ms": float(np.mean(numeric("controller_mean_ms"))),
            "controller_p95_ms": float(np.mean(numeric("controller_p95_ms"))),
            "controller_max_ms": float(np.max(numeric("controller_max_ms"))),
            "overrun_rate": float(np.mean(numeric("control_overrun_rate"))),
            "safety_stop_rate": float(np.mean(numeric("safety_stop_count") > 0)),
            "lidar_valid_ratio": float(np.mean(numeric("lidar_valid_ratio"))),
            "course_departure_rate": float(np.mean(numeric("course_departure"))),
        })
        summaries.append(summary)
    return summaries


def _write_csv(path, rows):
    if not rows:
        return
    with Path(path).open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _create_plots(summaries, output_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return _create_svg_plots(summaries, output_dir)
    labels = [
        f"{row['controller']}\n{row['network_profile']}\n{row['plant_profile']}"
        for row in summaries
    ]
    plots = []
    definitions = (
        ("completion_rate", "Completion rate", "completion_rate.png"),
        ("position_rmse_mean_m", "Position RMSE [m]", "position_rmse.png"),
        ("controller_p95_ms", "Controller p95 [ms]", "controller_timing.png"),
        ("safety_stop_rate", "Safety stop rate", "safety_stops.png"),
    )
    for field, title, filename in definitions:
        figure, axis = plt.subplots(figsize=(max(8, len(labels) * 0.5), 4.5))
        axis.bar(np.arange(len(labels)), [row[field] for row in summaries])
        axis.set_title(title)
        axis.set_xticks(np.arange(len(labels)))
        axis.set_xticklabels(labels, rotation=45, ha="right", fontsize=7)
        axis.grid(axis="y", alpha=0.3)
        figure.tight_layout()
        path = Path(output_dir) / filename
        figure.savefig(path, dpi=140)
        plt.close(figure)
        plots.append(path)
    return plots


def _create_svg_plots(summaries, output_dir):
    definitions = (
        ("completion_rate", "Completion rate", "completion_rate.svg"),
        ("position_rmse_mean_m", "Position RMSE [m]", "position_rmse.svg"),
        ("controller_p95_ms", "Controller p95 [ms]", "controller_timing.svg"),
        ("safety_stop_rate", "Safety stop rate", "safety_stops.svg"),
    )
    width = max(800, 90 * len(summaries))
    height = 460
    plots = []
    for field, title, filename in definitions:
        values = [float(row[field]) for row in summaries]
        maximum = max(values or [1.0]) or 1.0
        bars = []
        for index, (row, value) in enumerate(zip(summaries, values)):
            x = 55 + index * 85
            bar_height = 300.0 * value / maximum
            y = 350.0 - bar_height
            label = f"{row['controller']}/{row['network_profile']}"
            bars.extend([
                f'<rect x="{x}" y="{y:.2f}" width="55" height="{bar_height:.2f}" '
                'fill="#187b8d"/>',
                f'<text x="{x + 27.5}" y="{y - 5:.2f}" text-anchor="middle" '
                f'font-size="11">{value:.3f}</text>',
                f'<text x="{x + 27.5}" y="370" text-anchor="middle" font-size="9" '
                f'transform="rotate(35 {x + 27.5} 370)">{escape(label)}</text>',
            ])
        svg = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
            '<rect width="100%" height="100%" fill="white"/>',
            f'<text x="20" y="28" font-size="18" font-family="sans-serif">{escape(title)}</text>',
            '<line x1="45" y1="350" x2="98%" y2="350" stroke="#333"/>',
            *bars,
            '</svg>',
        ]
        path = Path(output_dir) / filename
        path.write_text("\n".join(svg), encoding="utf-8")
        plots.append(path)
    return plots


def generate_report(runs_path, output_dir):
    output_dir = Path(output_dir)
    rows = _load_rows(runs_path)
    summaries = _aggregate(rows)
    _write_csv(output_dir / "summary.csv", summaries)
    (output_dir / "summary.json").write_text(
        json.dumps({"groups": summaries}, indent=2) + "\n",
        encoding="utf-8",
    )
    plots = _create_plots(summaries, output_dir)
    overall_completion = float(np.mean([float(row["completed"]) for row in rows]))
    overall_stops = int(sum(int(row["safety_stop_count"]) for row in rows))
    lines = [
        "# 実機RCカー シミュレーション自動評価結果",
        "",
        "## 概要",
        "",
        f"- 実行数: {len(rows)}",
        f"- 完了率: {overall_completion:.3f}",
        f"- Safety停止回数: {overall_stops}",
        "",
        "## 条件別結果",
        "",
        "| Controller | Scenario | Pose | Plant | LiDAR | Network | Runs | Completion | "
        "Position RMSE [m] | p95 [ms] | Safety stop |",
        "| --- | --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summaries:
        lines.append(
            f"| {row['controller']} | {row['scenario']} | {row['pose_source']} | "
            f"{row['plant_profile']} | {row['lidar_profile']} | "
            f"{row['network_profile']} | {row['runs']} | "
            f"{row['completion_rate']:.3f} | {row['position_rmse_mean_m']:.4f} | "
            f"{row['controller_p95_ms']:.3f} | {row['safety_stop_rate']:.3f} |"
        )
    if plots:
        lines.extend(["", "## グラフ", ""])
        for path in plots:
            lines.append(f"![{path.stem}]({path.name})")
    lines.extend([
        "",
        "## 注意",
        "",
        "この結果はPC上の仮想車両、仮想LiDAR、仮想通信を使った結果であり、",
        "Raspberry Pi、実LiDAR、実PWMの性能を保証しない。",
        "",
    ])
    report_path = output_dir / "report.md"
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return report_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs_csv", type=Path)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()
    output_dir = args.output_dir or args.runs_csv.parent
    print(generate_report(args.runs_csv, output_dir))


if __name__ == "__main__":
    main()
