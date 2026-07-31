"""Run every hardware-free real-vehicle experiment and build presentation data."""

import argparse
import csv
from datetime import datetime, timezone
from html import escape
import json
from pathlib import Path
import platform
import shutil
import subprocess
import time

import numpy as np

from .common import write_rows
from .map_simulation import (
    CONTROLLER_RATES_HZ,
    DEFAULT_MAP,
    LIDAR_CONDITIONS,
    run_map_case,
)
from ..integration_ws.src.real_vehicle_integration.real_vehicle_integration.localization_replay import (
    compare_trial,
)
from ..integration_ws.src.real_vehicle_integration.real_vehicle_integration.real_map_localizer import (
    OccupancyMap,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "experiments/real_vehicle/results/offline_complete_pipeline"
)
DEFAULT_TRIAL_ROOT = (
    REPO_ROOT
    / "experiments/real_vehicle/results/phase5_localization/manual_motion"
)
CONTROLLERS = ("pure_pursuit", "mpc", "mppi")
COLORS = {
    "pure_pursuit": "#6a4c93",
    "mpc": "#087f8c",
    "mppi": "#d1495b",
    "reference": "#30343b",
}
FONT = "Noto Sans CJK JP, Yu Gothic, Hiragino Sans, sans-serif"


def _parse_csv(raw, cast=str):
    values = tuple(cast(value.strip()) for value in str(raw).split(",") if value.strip())
    if not values:
        raise ValueError("at least one value is required")
    return values


def _prepare_output(path, overwrite):
    path = Path(path).expanduser().resolve()
    if path.exists() and any(path.iterdir()):
        if not overwrite:
            raise FileExistsError(
                f"output directory is not empty: {path}; use --overwrite"
            )
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)
    return path


def _aggregate_map_runs(runs):
    summaries = []
    for condition in sorted({row["lidar_condition"] for row in runs}):
        for controller in CONTROLLERS:
            selected = [
                row for row in runs
                if row["controller"] == controller
                and row["lidar_condition"] == condition
            ]
            if not selected:
                continue

            def values(field):
                return np.asarray([float(row[field]) for row in selected])

            summaries.append({
                "controller": controller,
                "lidar_condition": condition,
                "run_count": len(selected),
                "completion_rate": float(np.mean(values("completed"))),
                "position_rmse_mean_m": float(
                    np.mean(values("position_rmse_m"))
                ),
                "position_rmse_std_m": float(
                    np.std(values("position_rmse_m"))
                ),
                "localization_xy_rmse_mean_m": float(
                    np.mean(values("localization_xy_rmse_m"))
                ),
                "controller_mean_ms": float(
                    np.mean(values("controller_mean_ms"))
                ),
                "controller_p95_mean_ms": float(
                    np.mean(values("controller_p95_ms"))
                ),
                "controller_p95_std_ms": float(
                    np.std(values("controller_p95_ms"))
                ),
                "localizer_failure_rate": float(
                    np.mean(values("localizer_failure_rate"))
                ),
                "minimum_map_clearance_m": float(
                    np.min(values("minimum_map_clearance_m"))
                ),
            })
    return summaries


def _read_csv(path):
    with Path(path).open("r", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _svg(width, height, body):
    return "\n".join([
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<g font-family="{FONT}" fill="#20242a">',
        *body,
        "</g>",
        "</svg>",
        "",
    ])


def _world_cell_centers(occupancy_map):
    rows, columns = np.where(occupancy_map.occupied)
    local_x = (columns + 0.5) * occupancy_map.resolution_m
    local_y = (
        occupancy_map.height - 1 - rows + 0.5
    ) * occupancy_map.resolution_m
    cosine = np.cos(occupancy_map.origin_yaw_rad)
    sine = np.sin(occupancy_map.origin_yaw_rad)
    x = occupancy_map.origin_x_m + cosine * local_x - sine * local_y
    y = occupancy_map.origin_y_m + sine * local_x + cosine * local_y
    return x, y


def _trajectory_svg(
    map_yaml,
    trajectory_paths,
    output_path,
    language,
):
    width, height = 1000, 620
    left, top, plot_width, plot_height = 90, 90, 700, 440
    x_min, x_max = -0.55, 0.65
    y_min, y_max = -0.65, 0.65

    def transform(x_value, y_value):
        x = left + (x_value - x_min) / (x_max - x_min) * plot_width
        y = top + plot_height - (y_value - y_min) / (y_max - y_min) * plot_height
        return x, y

    occupancy_map = OccupancyMap(map_yaml)
    occupied_x, occupied_y = _world_cell_centers(occupancy_map)
    visible = (
        (occupied_x >= x_min)
        & (occupied_x <= x_max)
        & (occupied_y >= y_min)
        & (occupied_y <= y_max)
    )
    title = (
        "保存した2D地図上での制御器比較"
        if language == "ja"
        else "Controller Comparison on the Saved 2D Map"
    )
    subtitle = (
        "実機地図 small_test_area_03・仮想LiDAR・同一初期状態"
        if language == "ja"
        else "Real map small_test_area_03, virtual LiDAR, matched initial state"
    )
    body = [
        f'<text x="40" y="40" font-size="25" font-weight="700">{title}</text>',
        f'<text x="40" y="67" font-size="14" fill="#5b626b">{subtitle}</text>',
        (
            f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" '
            'fill="#f6f7f8" stroke="#aeb4ba"/>'
        ),
    ]
    cell_width = max(
        occupancy_map.resolution_m / (x_max - x_min) * plot_width,
        1.0,
    )
    cell_height = max(
        occupancy_map.resolution_m / (y_max - y_min) * plot_height,
        1.0,
    )
    for x_value, y_value in zip(occupied_x[visible], occupied_y[visible]):
        x, y = transform(float(x_value), float(y_value))
        body.append(
            f'<rect x="{x - cell_width / 2:.2f}" y="{y - cell_height / 2:.2f}" '
            f'width="{cell_width:.2f}" height="{cell_height:.2f}" fill="#25282b"/>'
        )

    representative_rows = None
    for controller in CONTROLLERS:
        path = trajectory_paths.get(controller)
        if path is None:
            continue
        rows = _read_csv(path)
        representative_rows = representative_rows or rows
        points = " ".join(
            f"{x:.2f},{y:.2f}"
            for x, y in (
                transform(float(row["x_m"]), float(row["y_m"]))
                for row in rows[::max(len(rows) // 800, 1)]
            )
        )
        body.append(
            f'<polyline points="{points}" fill="none" '
            f'stroke="{COLORS[controller]}" stroke-width="4"/>'
        )
    if representative_rows:
        points = " ".join(
            f"{x:.2f},{y:.2f}"
            for x, y in (
                transform(
                    float(row["reference_x_m"]),
                    float(row["reference_y_m"]),
                )
                for row in representative_rows[
                    ::max(len(representative_rows) // 800, 1)
                ]
            )
        )
        body.append(
            f'<polyline points="{points}" fill="none" '
            f'stroke="{COLORS["reference"]}" stroke-width="2" '
            'stroke-dasharray="7 5"/>'
        )
    body.extend([
        (
            f'<text x="{left + plot_width / 2}" y="{top + plot_height + 45}" '
            'text-anchor="middle" font-size="15">x [m]</text>'
        ),
        (
            f'<text x="35" y="{top + plot_height / 2}" text-anchor="middle" '
            f'font-size="15" transform="rotate(-90 35 {top + plot_height / 2})">'
            'y [m]</text>'
        ),
    ])
    for index, controller in enumerate(CONTROLLERS):
        y = 165 + 42 * index
        body.extend([
            (
                f'<line x1="825" y1="{y}" x2="875" y2="{y}" '
                f'stroke="{COLORS[controller]}" stroke-width="4"/>'
            ),
            (
                f'<text x="890" y="{y + 5}" font-size="14">'
                f'{escape(controller.replace("_", " ").title())}</text>'
            ),
        ])
    body.extend([
        '<rect x="825" y="310" width="22" height="18" fill="#25282b"/>',
        (
            '<text x="860" y="325" font-size="14">'
            + ("地図の障害物" if language == "ja" else "Mapped obstacle")
            + "</text>"
        ),
    ])
    Path(output_path).write_text(_svg(width, height, body), encoding="utf-8")


def _metrics_svg(summaries, output_path, language):
    selected = [
        row for row in summaries if row["lidar_condition"] == "ideal"
    ]
    width, height = 1040, 540
    title = (
        "3つの制御器の精度・計算時間比較"
        if language == "ja"
        else "Accuracy and Computation Time of Three Controllers"
    )
    body = [
        f'<text x="40" y="42" font-size="25" font-weight="700">{title}</text>',
    ]
    panels = (
        (
            "position_rmse_mean_m",
            "position_rmse_std_m",
            "位置RMSE [m]" if language == "ja" else "Position RMSE [m]",
            60,
        ),
        (
            "controller_p95_mean_ms",
            "controller_p95_std_ms",
            "p95計算時間 [ms]" if language == "ja" else "p95 computation [ms]",
            550,
        ),
    )
    for field, _error_field, label, panel_left in panels:
        values = [float(row[field]) for row in selected]
        maximum = max(values or [1.0]) * 1.25 or 1.0
        plot_top, plot_height = 105, 330
        body.extend([
            f'<text x="{panel_left}" y="88" font-size="17" font-weight="600">{label}</text>',
            (
                f'<line x1="{panel_left}" y1="{plot_top + plot_height}" '
                f'x2="{panel_left + 420}" y2="{plot_top + plot_height}" '
                'stroke="#4b5158"/>'
            ),
        ])
        for index, row in enumerate(selected):
            value = float(row[field])
            center = panel_left + 85 + index * 125
            bar_height = value / maximum * plot_height
            y = plot_top + plot_height - bar_height
            controller = row["controller"]
            body.extend([
                (
                    f'<rect x="{center - 35}" y="{y:.2f}" width="70" '
                    f'height="{bar_height:.2f}" fill="{COLORS[controller]}"/>'
                ),
                (
                    f'<text x="{center}" y="{y - 9:.2f}" text-anchor="middle" '
                    f'font-size="13">{value:.4f}</text>'
                ),
                (
                    f'<text x="{center}" y="{plot_top + plot_height + 28}" '
                    f'text-anchor="middle" font-size="13">'
                    f'{escape(controller.replace("_", " ").title())}</text>'
                ),
            ])
    Path(output_path).write_text(_svg(width, height, body), encoding="utf-8")


def _replay_svg(replay_summaries, output_path, language):
    width, height = 1040, 540
    title = (
        "取得済み実LiDARデータのオフライン再生"
        if language == "ja"
        else "Offline Replay of Recorded Real LiDAR Data"
    )
    subtitle = (
        "保存したscan・odometryをguarded自己位置推定へ再入力"
        if language == "ja"
        else "Recorded scans and odometry replayed through guarded localization"
    )
    body = [
        f'<text x="40" y="42" font-size="25" font-weight="700">{title}</text>',
        f'<text x="40" y="68" font-size="14" fill="#5b626b">{subtitle}</text>',
        '<rect x="55" y="105" width="930" height="340" fill="#f7f8f9" stroke="#c1c6ca"/>',
        (
            '<text x="80" y="140" font-size="15" font-weight="600">'
            + ("試行" if language == "ja" else "Trial")
            + "</text>"
        ),
        (
            '<text x="590" y="140" font-size="15" font-weight="600">'
            + ("位置差 [m]" if language == "ja" else "Pose/odom difference [m]")
            + "</text>"
        ),
        (
            '<text x="790" y="140" font-size="15" font-weight="600">'
            + ("平均計算時間 [ms]" if language == "ja" else "Mean time [ms]")
            + "</text>"
        ),
    ]
    for index, summary in enumerate(replay_summaries[:7]):
        y = 180 + index * 38
        difference = summary.get("pose_odom_distance_difference_m")
        processing = summary.get("processing_time_ms_mean")
        difference_text = (
            f"{float(difference):.4f}" if difference is not None else "N/A"
        )
        processing_text = (
            f"{float(processing):.3f}" if processing is not None else "N/A"
        )
        body.extend([
            f'<text x="80" y="{y}" font-size="13">{escape(summary["trial"])}</text>',
            f'<text x="650" y="{y}" text-anchor="middle" font-size="13">'
            f'{difference_text}</text>',
            f'<text x="855" y="{y}" text-anchor="middle" font-size="13">'
            f'{processing_text}</text>',
        ])
    Path(output_path).write_text(_svg(width, height, body), encoding="utf-8")


def _rasterize(svg_path, png_path):
    browser = next(
        (
            executable
            for name in ("google-chrome", "chromium", "chromium-browser")
            if (executable := shutil.which(name)) is not None
        ),
        None,
    )
    if browser is None:
        return False
    subprocess.run(
        [
            browser,
            "--headless",
            "--disable-gpu",
            "--no-sandbox",
            "--hide-scrollbars",
            "--force-device-scale-factor=2",
            "--window-size=1100,700",
            f"--screenshot={Path(png_path).resolve()}",
            Path(svg_path).resolve().as_uri(),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return True


def _create_figures(
    output_dir,
    map_yaml,
    summaries,
    replay_summaries,
    representative_paths,
):
    japanese = (
        ("map_trajectory_comparison", _trajectory_svg),
        ("accuracy_and_timing", _metrics_svg),
        ("real_data_replay", _replay_svg),
    )
    for stem, function in japanese:
        svg_path = output_dir / f"{stem}.svg"
        if stem == "map_trajectory_comparison":
            function(map_yaml, representative_paths, svg_path, "ja")
        elif stem == "accuracy_and_timing":
            function(summaries, svg_path, "ja")
        else:
            function(replay_summaries, svg_path, "ja")
        _rasterize(svg_path, output_dir / f"{stem}.png")

    english_dir = output_dir / "english"
    english_dir.mkdir()
    temporary = output_dir / ".english_svg"
    temporary.mkdir()
    for stem, function in japanese:
        svg_path = temporary / f"{stem}.svg"
        if stem == "map_trajectory_comparison":
            function(map_yaml, representative_paths, svg_path, "en")
        elif stem == "accuracy_and_timing":
            function(summaries, svg_path, "en")
        else:
            function(replay_summaries, svg_path, "en")
        _rasterize(svg_path, english_dir / f"{stem}.png")
    shutil.rmtree(temporary)


def _write_report(output_dir, runs, summaries, replay_summaries, metadata):
    ideal = {
        row["controller"]: row
        for row in summaries
        if row["lidar_condition"] == "ideal"
    }
    replay_passed = sum(
        summary.get("status") == "passed" for summary in replay_summaries
    )
    replay_failed = [
        summary for summary in replay_summaries
        if summary.get("status") != "passed"
    ]
    lines = [
        "# 実機なしで継続する自律走行研究 実行結果",
        "",
        "## 今回実行したこと",
        "",
        "- 保存済み2D地図 `small_test_area_03` をPC上で読み込んだ。",
        "- 地図に対して仮想LiDARを照射し、保存地図ベースの自己位置推定を動かした。",
        "- Pure Pursuit、MPC、MPPIを同じ車体・経路・初期状態で比較した。",
        "- ノイズ、遅延、欠損を含むLiDAR条件を評価した。",
        "- 取得済みの実LiDAR・odometryデータをオフライン再生した。",
        "- CSV、JSON、日本語PNG、英語PNGを生成した。",
        "",
        "## 理想LiDAR条件の比較",
        "",
        "| 制御器 | 完了率 | 位置RMSE [m] | p95計算時間 [ms] | 自己位置RMSE [m] |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for controller in CONTROLLERS:
        row = ideal.get(controller)
        if row is None:
            continue
        lines.append(
            f"| {controller} | {row['completion_rate']:.3f} | "
            f"{row['position_rmse_mean_m']:.4f} | "
            f"{row['controller_p95_mean_ms']:.3f} | "
            f"{row['localization_xy_rmse_mean_m']:.4f} |"
        )
    lines.extend([
        "",
        "## 実データ再生",
        "",
        f"- 再生した試行数: {len(replay_summaries)}",
        f"- 合格: {replay_passed}、不合格または評価不能: {len(replay_failed)}",
        "- 実機を再接続しなくても、保存済みscanとodometryからlocalizerを再評価できる。",
    ])
    for summary in replay_failed:
        failures = "; ".join(str(value) for value in summary.get("failures", []))
        lines.append(
            f"- `{summary['trial']}`: {failures or '評価条件を満たさなかった'}"
        )
    lines.extend([
        "",
        "## 発表用グラフ",
        "",
        "![地図上の軌跡](map_trajectory_comparison.png)",
        "",
        "![精度と計算時間](accuracy_and_timing.png)",
        "",
        "![実データ再生](real_data_replay.png)",
        "",
        "英語版PNGは `english/` にある。",
        "",
        "## データの読み方",
        "",
        "- `map_runs.csv`: すべてのシミュレーション走行の集計前データ。",
        "- `map_summary.csv`: 制御器・LiDAR条件ごとの平均値。",
        "- `trajectories/*.csv`: 0.01秒ごとの位置、目標位置、推定位置、指令。",
        "- `replay/*`: 実LiDAR記録を再計算した結果。",
        "- `metadata.json`: 実行条件と環境。",
        "",
        "## 重要な制限",
        "",
        "この結果の地図と再生データは実機由来だが、今回の新しい走行はPC上の仮想車両である。",
        "モーター、操舵サーボ、実Wi-Fi、衝突安全性を新たに検証した結果ではない。",
        "",
        f"実行数: {len(runs)}、開始UTC: {metadata['started_at_utc']}",
        "",
    ])
    report = output_dir / "report_ja.md"
    report.write_text("\n".join(lines), encoding="utf-8")
    return report


def run_pipeline(
    output_dir=DEFAULT_OUTPUT,
    map_yaml=DEFAULT_MAP,
    seeds=(123, 456, 789),
    conditions=tuple(LIDAR_CONDITIONS),
    overwrite=False,
    smoke=False,
):
    """Run map simulation, real-data replay, aggregation, and figures."""

    output_dir = _prepare_output(output_dir, overwrite)
    map_yaml = Path(map_yaml).expanduser().resolve()
    if smoke:
        seeds = (int(seeds[0]),)
        conditions = ("ideal",)
        duration_s = 0.30
        distance_m = 0.03
    else:
        duration_s = 4.0
        distance_m = 0.45
    started = time.perf_counter()
    started_at = datetime.now(timezone.utc).isoformat()

    runs = []
    representative_paths = {}
    cases = [
        (controller, condition, int(seed))
        for condition in conditions
        for controller in CONTROLLERS
        for seed in seeds
    ]
    for index, (controller, condition, seed) in enumerate(cases, 1):
        summary, _rows = run_map_case(
            controller,
            condition,
            seed,
            map_yaml=map_yaml,
            output_dir=output_dir,
            duration_s=duration_s,
            distance_m=distance_m,
        )
        runs.append(summary)
        run_id = f"{controller}_{condition}_localized_seed{seed}.csv"
        if condition == conditions[-1] and seed == seeds[0]:
            representative_paths[controller] = (
                output_dir / "trajectories" / run_id
            )
        print(
            f"[map {index}/{len(cases)}] {controller}/{condition}/seed{seed}: "
            f"completed={summary['completed']} "
            f"rmse={summary['position_rmse_m']:.4f}m"
        )
    write_rows(output_dir / "map_runs.csv", runs)
    summaries = _aggregate_map_runs(runs)
    write_rows(output_dir / "map_summary.csv", summaries)
    (output_dir / "map_summary.json").write_text(
        json.dumps({"groups": summaries}, indent=2) + "\n",
        encoding="utf-8",
    )

    replay_root = output_dir / "replay"
    trial_dirs = sorted(
        path for path in DEFAULT_TRIAL_ROOT.iterdir()
        if path.is_dir()
        and (path / "scan.csv").is_file()
        and (path / "odom.csv").is_file()
        and (path / "pose.csv").is_file()
        and (path / "summary.json").is_file()
    )
    if smoke:
        trial_dirs = trial_dirs[:1]
    replay_summaries = []
    for index, trial_dir in enumerate(trial_dirs, 1):
        comparison = compare_trial(
            trial_dir,
            replay_root / trial_dir.name,
            map_yaml=map_yaml,
            presets=("guarded",),
        )
        summary = dict(comparison["summaries"]["guarded"])
        summary["trial"] = trial_dir.name
        replay_summaries.append(summary)
        print(
            f"[replay {index}/{len(trial_dirs)}] {trial_dir.name}: "
            f"status={summary['status']}"
        )
    if replay_summaries:
        write_rows(output_dir / "replay_summary.csv", replay_summaries)

    _create_figures(
        output_dir,
        map_yaml,
        summaries,
        replay_summaries,
        representative_paths,
    )
    metadata = {
        "started_at_utc": started_at,
        "wall_time_s": time.perf_counter() - started,
        "map_yaml": str(map_yaml),
        "controllers": list(CONTROLLERS),
        "conditions": list(conditions),
        "seeds": list(seeds),
        "map_run_count": len(runs),
        "real_replay_count": len(replay_summaries),
        "smoke": bool(smoke),
        "hardware_output_enabled": False,
        "python": platform.python_version(),
        "platform": platform.platform(),
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )
    report = _write_report(
        output_dir,
        runs,
        summaries,
        replay_summaries,
        metadata,
    )
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--map-yaml", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--seeds", default="123,456,789")
    parser.add_argument(
        "--conditions",
        default=",".join(LIDAR_CONDITIONS),
    )
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--smoke", action="store_true")
    options = parser.parse_args(args)
    try:
        report = run_pipeline(
            output_dir=options.output_dir,
            map_yaml=options.map_yaml,
            seeds=_parse_csv(options.seeds, int),
            conditions=_parse_csv(options.conditions),
            overwrite=options.overwrite,
            smoke=options.smoke,
        )
    except (FileExistsError, FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"offline pipeline failed: {exc}")
        return 2
    print(f"report: {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
