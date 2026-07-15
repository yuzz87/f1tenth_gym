from argparse import ArgumentParser
import csv
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def load_csv(csv_path):
    """CSV ログを列単位の numpy 配列として読み込む。"""
    columns = {}
    with open(csv_path, newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            for key, value in row.items():
                columns.setdefault(key, []).append(float(value))
    return {key: np.asarray(values, dtype=float) for key, values in columns.items()}


def data_bounds(values, pad_ratio=0.08, minimum_half_span=1e-3):
    """描画用の最小最大を返す。"""
    low = float(np.min(values))
    high = float(np.max(values))
    if np.isclose(low, high):
        center = 0.5 * (low + high)
        return center - minimum_half_span, center + minimum_half_span
    span = high - low
    pad = span * pad_ratio
    return low - pad, high + pad


def polyline_points(xs, ys, x_min, x_max, y_min, y_max, left, top, width, height):
    """データ列を SVG polyline 用の座標文字列へ変換する。"""
    x_span = max(x_max - x_min, 1e-9)
    y_span = max(y_max - y_min, 1e-9)
    svg_points = []
    for x_value, y_value in zip(xs, ys):
        px = left + (float(x_value) - x_min) / x_span * width
        py = top + height - (float(y_value) - y_min) / y_span * height
        svg_points.append(f"{px:.2f},{py:.2f}")
    return " ".join(svg_points)


def rect_panel(title, left, top, width, height):
    """パネル枠とタイトルを返す。"""
    return [
        f'<rect x="{left}" y="{top}" width="{width}" height="{height}" fill="#ffffff" stroke="#d0d7de" stroke-width="1"/>',
        f'<text x="{left + 12}" y="{top + 24}" font-size="16" font-family="monospace" fill="#24292f">{title}</text>',
    ]


def draw_xy_panel(rows, data):
    """軌跡と中心線の重ね描きパネルを生成する。"""
    left, top, width, height = rows
    margin_left = 60
    margin_right = 30
    margin_top = 36
    margin_bottom = 40
    plot_left = left + margin_left
    plot_top = top + margin_top
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    xs = np.concatenate((data["pose_x"], data["ref_x"]))
    ys = np.concatenate((data["pose_y"], data["ref_y"]))
    x_min, x_max = data_bounds(xs)
    y_min, y_max = data_bounds(ys)

    parts = rect_panel("Trajectory vs Centerline", left, top, width, height)
    parts.append(
        f'<rect x="{plot_left}" y="{plot_top}" width="{plot_width}" height="{plot_height}" fill="#f6f8fa" stroke="#d0d7de" stroke-width="1"/>'
    )
    parts.append(
        f'<polyline fill="none" stroke="#1f883d" stroke-width="2" points="{polyline_points(data["ref_x"], data["ref_y"], x_min, x_max, y_min, y_max, plot_left, plot_top, plot_width, plot_height)}"/>'
    )
    parts.append(
        f'<polyline fill="none" stroke="#0969da" stroke-width="2" points="{polyline_points(data["pose_x"], data["pose_y"], x_min, x_max, y_min, y_max, plot_left, plot_top, plot_width, plot_height)}"/>'
    )
    parts.append(
        f'<text x="{plot_left}" y="{top + height - 12}" font-size="12" font-family="monospace" fill="#57606a">x [{x_min:.2f}, {x_max:.2f}] m</text>'
    )
    parts.append(
        f'<text x="{left + 8}" y="{plot_top + 12}" font-size="12" font-family="monospace" fill="#57606a">y [{y_min:.2f}, {y_max:.2f}] m</text>'
    )
    return parts


def draw_time_series_panel(title, left, top, width, height, times, series_specs):
    """複数系列の時系列パネルを生成する。"""
    margin_left = 60
    margin_right = 30
    margin_top = 36
    margin_bottom = 40
    plot_left = left + margin_left
    plot_top = top + margin_top
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    y_values = np.concatenate([spec["values"] for spec in series_specs])
    x_min, x_max = data_bounds(times, pad_ratio=0.0, minimum_half_span=0.5)
    y_min, y_max = data_bounds(y_values)

    parts = rect_panel(title, left, top, width, height)
    parts.append(
        f'<rect x="{plot_left}" y="{plot_top}" width="{plot_width}" height="{plot_height}" fill="#f6f8fa" stroke="#d0d7de" stroke-width="1"/>'
    )

    if y_min <= 0.0 <= y_max:
        zero_y = plot_top + plot_height - (0.0 - y_min) / max(y_max - y_min, 1e-9) * plot_height
        parts.append(
            f'<line x1="{plot_left}" y1="{zero_y:.2f}" x2="{plot_left + plot_width}" y2="{zero_y:.2f}" stroke="#8c959f" stroke-width="1" stroke-dasharray="4 4"/>'
        )

    for spec in series_specs:
        parts.append(
            f'<polyline fill="none" stroke="{spec["color"]}" stroke-width="2" points="{polyline_points(times, spec["values"], x_min, x_max, y_min, y_max, plot_left, plot_top, plot_width, plot_height)}"/>'
        )

    legend_x = plot_left + 4
    legend_y = plot_top + 16
    for index, spec in enumerate(series_specs):
        y_pos = legend_y + index * 18
        parts.append(
            f'<line x1="{legend_x}" y1="{y_pos - 4}" x2="{legend_x + 18}" y2="{y_pos - 4}" stroke="{spec["color"]}" stroke-width="3"/>'
        )
        parts.append(
            f'<text x="{legend_x + 24}" y="{y_pos}" font-size="12" font-family="monospace" fill="#24292f">{spec["label"]}</text>'
        )

    parts.append(
        f'<text x="{plot_left}" y="{top + height - 12}" font-size="12" font-family="monospace" fill="#57606a">time [{x_min:.2f}, {x_max:.2f}] s</text>'
    )
    parts.append(
        f'<text x="{left + 8}" y="{plot_top + 12}" font-size="12" font-family="monospace" fill="#57606a">y [{y_min:.3f}, {y_max:.3f}]</text>'
    )
    return parts


def build_summary_svg(data):
    """走行ログの概要 SVG を構築する。"""
    width = 1100
    height = 1180
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f3f4f6"/>',
        '<text x="40" y="42" font-size="24" font-family="monospace" fill="#24292f">F1TENTH baseline run summary</text>',
    ]

    parts.extend(draw_xy_panel((40, 70, 1020, 360), data))
    parts.extend(
        draw_time_series_panel(
            "Cross-track and Heading Error",
            40,
            460,
            1020,
            280,
            data["sim_time"],
            [
                {"label": "cross_track_error [m]", "values": data["cross_track_error"], "color": "#d1242f"},
                {"label": "heading_error [rad]", "values": data["heading_error"], "color": "#8250df"},
            ],
        )
    )
    parts.extend(
        draw_time_series_panel(
            "Control and Velocity",
            40,
            770,
            1020,
            280,
            data["sim_time"],
            [
                {"label": "speed_cmd [m/s]", "values": data["speed_cmd"], "color": "#1a7f37"},
                {"label": "steer_cmd [rad]", "values": data["steer_cmd"], "color": "#0969da"},
                {"label": "linear_vel_x [m/s]", "values": data["linear_vel_x"], "color": "#bf8700"},
            ],
        )
    )

    parts.append("</svg>")
    return "\n".join(parts)


def format_metric(value):
    """Markdown レポート用の数値整形。"""
    return f"{float(value):.6f}"


def build_report_markdown(csv_path, svg_path, data):
    """Zed のプレビュー向け Markdown レポートを構築する。"""
    cross_track_abs = np.abs(data["cross_track_error"])
    heading_abs = np.abs(data["heading_error"])
    speed_cmd_abs = np.abs(data["speed_cmd"])
    final_lap_count = int(data["lap_count"][-1]) if "lap_count" in data else 0
    final_lap_time = float(data["lap_time"][-1]) if "lap_time" in data else 0.0
    completed_lap = final_lap_count >= 1

    summary_lines = [
        "# Run Summary",
        "",
        f"- source_csv: `{csv_path.name}`",
        f"- sim_time_final_s: `{format_metric(data['sim_time'][-1])}`",
        f"- total_steps: `{int(data['step'][-1])}`",
        f"- completed_lap_1: `{str(completed_lap).lower()}`",
        f"- final_lap_count: `{final_lap_count}`",
        f"- final_lap_time_s: `{format_metric(final_lap_time)}`",
        f"- max_abs_cross_track_m: `{format_metric(np.max(cross_track_abs))}`",
        f"- mean_abs_cross_track_m: `{format_metric(np.mean(cross_track_abs))}`",
        f"- max_abs_heading_rad: `{format_metric(np.max(heading_abs))}`",
        f"- mean_abs_heading_rad: `{format_metric(np.mean(heading_abs))}`",
        f"- max_speed_cmd_mps: `{format_metric(np.max(speed_cmd_abs))}`",
        "",
        f"![run summary]({svg_path.name})",
        "",
        "Zed ではこの Markdown を Preview で開くと、上のグラフを画像として見られる。",
        "",
    ]
    return "\n".join(summary_lines)


def main():
    parser = ArgumentParser(description="走行ログ CSV から概要グラフ SVG と Markdown レポートを出力する。")
    parser.add_argument("csv_path", help="入力する CSV ログのパス。")
    parser.add_argument(
        "--output",
        help="出力する SVG パス。省略時は CSV と同じ場所に *_summary.svg を出力する。",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    output_path = Path(args.output) if args.output else csv_path.with_name(f"{csv_path.stem}_sum.svg")
    data = load_csv(csv_path)
    summary_svg = build_summary_svg(data)
    output_path.write_text(summary_svg, encoding="utf-8")
    report_path = csv_path.with_name(f"{csv_path.stem}_view.md")
    report_markdown = build_report_markdown(csv_path, output_path, data)
    report_path.write_text(report_markdown, encoding="utf-8")
    print(f"summary_svg: {output_path}")
    print(f"report_md: {report_path}")


if __name__ == "__main__":
    main()
