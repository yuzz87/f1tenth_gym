"""Generate a matched MPC/MPPI data set and presentation-ready SVG figures."""

import argparse
import copy
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile
import time

import numpy as np

from .common import DEFAULT_CONFIG, load_real_config, write_rows
from .simulation_campaign import DEFAULT_CAMPAIGN, load_campaign, run_case


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "experiments/real_vehicle/results/2026-07-21_presentation_controller_comparison"
)
CONTROLLERS = ("mpc", "mppi")
COLORS = {"mpc": "#087f8c", "mppi": "#d1495b", "reference": "#30343b"}
SVG_FONT_FAMILY = "Noto Sans CJK JP, Yu Gothic, Hiragino Sans, sans-serif"


def _parse_seeds(raw):
    seeds = tuple(int(value.strip()) for value in str(raw).split(",") if value.strip())
    if not seeds:
        raise ValueError("at least one seed is required")
    return seeds


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _git_revision():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _git_worktree_dirty():
    try:
        output = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=REPO_ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        )
        return bool(output.strip())
    except (OSError, subprocess.CalledProcessError):
        return None


def _source_hashes():
    relative_paths = [
        "experiments/real_vehicle/evaluation/common.py",
        "experiments/real_vehicle/evaluation/presentation_experiment.py",
        "experiments/real_vehicle/evaluation/simulation_campaign.py",
        "experiments/real_vehicle/controllers/common.py",
        "experiments/real_vehicle/controllers/real_vehicle_mpc.py",
        "experiments/real_vehicle/controllers/real_vehicle_mppi.py",
        "experiments/real_vehicle/localization/virtual_lidar.py",
        "experiments/real_vehicle/models/actuator_model.py",
        "experiments/real_vehicle/models/bicycle_model.py",
    ]
    return {
        relative_path: _sha256(REPO_ROOT / relative_path)
        for relative_path in relative_paths
    }


def _prepare_output(output_dir, overwrite):
    output_dir = Path(output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        if not overwrite:
            raise FileExistsError(
                f"output directory is not empty: {output_dir}; use --overwrite"
            )
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def _case(controller, seed):
    return {
        "seed": int(seed),
        "controller": controller,
        "scenario": "circle",
        "pose_source": "ground_truth",
        "plant_profile": "nominal",
        "lidar_profile": "ideal",
        "network_profile": "normal",
    }


def aggregate_results(rows):
    """Aggregate matched runs without hiding seed-level raw data."""

    summaries = []
    for controller in CONTROLLERS:
        values = [row for row in rows if row["controller"] == controller]
        if not values:
            continue

        def numeric(field):
            return np.asarray([float(row[field]) for row in values], dtype=float)

        summaries.append({
            "controller": controller,
            "run_count": len(values),
            "completion_rate": float(np.mean(numeric("completed"))),
            "position_rmse_mean_m": float(np.mean(numeric("position_rmse_m"))),
            "position_rmse_std_m": float(np.std(numeric("position_rmse_m"))),
            "controller_mean_ms": float(np.mean(numeric("controller_mean_ms"))),
            "controller_p95_mean_ms": float(np.mean(numeric("controller_p95_ms"))),
            "controller_p95_std_ms": float(np.std(numeric("controller_p95_ms"))),
            "controller_max_ms": float(np.max(numeric("controller_max_ms"))),
            "deadline_exceeded_total": int(np.sum(numeric("deadline_exceeded_count"))),
            "circle_radius_final_error_mean_m": float(
                np.mean(numeric("circle_radius_final_error_m"))
            ),
            "circle_phase_abs_error_mean_rad": float(
                np.mean(np.abs(numeric("circle_phase_error_rad")))
            ),
        })
    return summaries


def _read_trajectory(path):
    with Path(path).open("r", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _svg_document(width, height, body):
    return "\n".join([
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<g font-family="{SVG_FONT_FAMILY}" fill="#20242a">',
        *body,
        "</g>",
        "</svg>",
        "",
    ])


def _polyline(rows, x_field, y_field, transform, color, width, dash=""):
    stride = max(len(rows) // 1200, 1)
    points = []
    for row in rows[::stride]:
        x, y = transform(float(row[x_field]), float(row[y_field]))
        points.append(f"{x:.2f},{y:.2f}")
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" '
        f'stroke-width="{width}" stroke-linejoin="round"{dash_attr}/>'
    )


def create_trajectory_figure(trajectories, output_path, seed, language="ja"):
    width, height = 900, 680
    left, top, plot_size = 120, 90, 520
    extent = 2.35

    def transform(x_value, y_value):
        x = left + (x_value + extent) / (2.0 * extent) * plot_size
        y = top + plot_size - (y_value + extent) / (2.0 * extent) * plot_size
        return x, y

    if language == "ja":
        title = "MPCとMPPIによる円形軌道追従"
        subtitle = f"同一条件・代表シード {seed}・目標速度0.30 m/s・直径4 m"
        reference_label = "目標軌道"
    elif language == "en":
        title = "Circular Trajectory Tracking with MPC and MPPI"
        subtitle = f"Matched conditions, representative seed {seed}, target speed 0.30 m/s, diameter 4 m"
        reference_label = "Reference"
    else:
        raise ValueError("language must be 'ja' or 'en'")
    body = [
        f'<text x="40" y="42" font-size="24" font-weight="700">{title}</text>',
        f'<text x="40" y="68" font-size="14" fill="#5b626b">{subtitle}</text>',
        f'<rect x="{left}" y="{top}" width="{plot_size}" height="{plot_size}" fill="#fafbfc" stroke="#aeb4ba"/>',
    ]
    for tick in range(-2, 3):
        x, _ = transform(tick, 0.0)
        _, y = transform(0.0, tick)
        body.extend([
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_size}" stroke="#e3e6e8"/>',
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_size}" y2="{y:.2f}" stroke="#e3e6e8"/>',
            f'<text x="{x:.2f}" y="{top + plot_size + 23}" text-anchor="middle" font-size="12">{tick}</text>',
            f'<text x="{left - 16}" y="{y + 4:.2f}" text-anchor="end" font-size="12">{tick}</text>',
        ])
    representative = trajectories[CONTROLLERS[0]]
    body.append(_polyline(
        representative,
        "reference_x_m",
        "reference_y_m",
        transform,
        COLORS["reference"],
        2.0,
        "7 5",
    ))
    for controller in CONTROLLERS:
        body.append(_polyline(
            trajectories[controller],
            "x_m",
            "y_m",
            transform,
            COLORS[controller],
            3.0,
        ))
    body.extend([
        f'<text x="{left + plot_size / 2}" y="{top + plot_size + 54}" text-anchor="middle" font-size="15">x [m]</text>',
        f'<text x="45" y="{top + plot_size / 2}" text-anchor="middle" font-size="15" transform="rotate(-90 45 {top + plot_size / 2})">y [m]</text>',
        '<line x1="690" y1="150" x2="742" y2="150" stroke="#30343b" stroke-width="2" stroke-dasharray="7 5"/>',
        f'<text x="755" y="155" font-size="14">{reference_label}</text>',
        f'<line x1="690" y1="190" x2="742" y2="190" stroke="{COLORS["mpc"]}" stroke-width="3"/>',
        '<text x="755" y="195" font-size="14">MPC</text>',
        f'<line x1="690" y1="230" x2="742" y2="230" stroke="{COLORS["mppi"]}" stroke-width="3"/>',
        '<text x="755" y="235" font-size="14">MPPI</text>',
    ])
    Path(output_path).write_text(_svg_document(width, height, body), encoding="utf-8")


def create_position_error_figure(trajectories, output_path, seed, language="ja"):
    width, height = 1000, 520
    left, top, plot_width, plot_height = 90, 90, 820, 330
    duration = max(float(rows[-1]["timestamp_s"]) for rows in trajectories.values())
    maximum = max(
        float(row["position_error_m"])
        for rows in trajectories.values()
        for row in rows
    )
    y_max = max(0.05, np.ceil(maximum * 20.0) / 20.0)

    def transform(x_value, y_value):
        x = left + x_value / max(duration, 1e-9) * plot_width
        y = top + plot_height - y_value / y_max * plot_height
        return x, y

    if language == "ja":
        title = "1周走行中の位置誤差"
        subtitle = f"代表シード {seed}（値が小さいほど目標位置に近い）"
        x_label = "時間 [s]"
        y_label = "位置誤差 [m]"
    elif language == "en":
        title = "Position Error over One Lap"
        subtitle = f"Representative seed {seed} (lower is better)"
        x_label = "Time [s]"
        y_label = "Position error [m]"
    else:
        raise ValueError("language must be 'ja' or 'en'")
    body = [
        f'<text x="40" y="42" font-size="24" font-weight="700">{title}</text>',
        f'<text x="40" y="68" font-size="14" fill="#5b626b">{subtitle}</text>',
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" fill="#fafbfc" stroke="#aeb4ba"/>',
    ]
    for index in range(6):
        time_value = duration * index / 5.0
        x, _ = transform(time_value, 0.0)
        body.extend([
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}" stroke="#e3e6e8"/>',
            f'<text x="{x:.2f}" y="{top + plot_height + 23}" text-anchor="middle" font-size="12">{time_value:.0f}</text>',
        ])
    for index in range(5):
        error_value = y_max * index / 4.0
        _, y = transform(0.0, error_value)
        body.extend([
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" stroke="#e3e6e8"/>',
            f'<text x="{left - 14}" y="{y + 4:.2f}" text-anchor="end" font-size="12">{error_value:.2f}</text>',
        ])
    for controller in CONTROLLERS:
        body.append(_polyline(
            trajectories[controller],
            "timestamp_s",
            "position_error_m",
            transform,
            COLORS[controller],
            2.5,
        ))
    body.extend([
        f'<text x="{left + plot_width / 2}" y="{top + plot_height + 58}" text-anchor="middle" font-size="15">{x_label}</text>',
        f'<text x="28" y="{top + plot_height / 2}" text-anchor="middle" font-size="15" transform="rotate(-90 28 {top + plot_height / 2})">{y_label}</text>',
        f'<line x1="730" y1="115" x2="775" y2="115" stroke="{COLORS["mpc"]}" stroke-width="3"/><text x="785" y="120" font-size="13">MPC</text>',
        f'<line x1="730" y1="145" x2="775" y2="145" stroke="{COLORS["mppi"]}" stroke-width="3"/><text x="785" y="150" font-size="13">MPPI</text>',
    ])
    Path(output_path).write_text(_svg_document(width, height, body), encoding="utf-8")


def create_metric_figure(summaries, output_path, language="ja"):
    width, height = 1040, 540
    if language == "ja":
        title = "MPCとMPPIの精度・計算時間比較"
        subtitle = "同一条件・3シードの平均（誤差線：標準偏差）"
        accuracy_title = "位置RMSE [m]（小さいほど高精度）"
        timing_title = "制御計算時間 p95 [ms]（小さいほど高速）"
        deadlines = ((10.0, "MPPI周期", "#59636e"), (20.0, "MPC周期", "#8a5d00"))
    elif language == "en":
        title = "MPC vs. MPPI: Accuracy and Computation Time"
        subtitle = "Mean across three matched seeds (error bars: one standard deviation)"
        accuracy_title = "Position RMSE [m] (lower is more accurate)"
        timing_title = "Control computation p95 [ms] (lower is faster)"
        deadlines = ((10.0, "MPPI period", "#59636e"), (20.0, "MPC period", "#8a5d00"))
    else:
        raise ValueError("language must be 'ja' or 'en'")
    body = [
        f'<text x="40" y="42" font-size="24" font-weight="700">{title}</text>',
        f'<text x="40" y="68" font-size="14" fill="#5b626b">{subtitle}</text>',
    ]
    panels = (
        (70, "position_rmse_mean_m", "position_rmse_std_m", accuracy_title, None),
        (560, "controller_p95_mean_ms", "controller_p95_std_ms", timing_title, deadlines),
    )
    for panel_left, value_field, std_field, title, deadlines in panels:
        plot_top, plot_width, plot_height = 125, 400, 300
        values = [float(row[value_field]) for row in summaries]
        errors = [float(row[std_field]) for row in summaries]
        maximum = max(value + error for value, error in zip(values, errors))
        if deadlines is not None:
            maximum = max(maximum, max(value for value, _label, _color in deadlines))
        y_max = max(maximum * 1.2, 1e-6)
        body.extend([
            f'<text x="{panel_left}" y="105" font-size="17" font-weight="700">{title}</text>',
            f'<rect x="{panel_left}" y="{plot_top}" width="{plot_width}" height="{plot_height}" fill="#fafbfc" stroke="#aeb4ba"/>',
        ])
        for index in range(5):
            value = y_max * index / 4.0
            y = plot_top + plot_height - value / y_max * plot_height
            body.extend([
                f'<line x1="{panel_left}" y1="{y:.2f}" x2="{panel_left + plot_width}" y2="{y:.2f}" stroke="#e3e6e8"/>',
                f'<text x="{panel_left - 10}" y="{y + 4:.2f}" text-anchor="end" font-size="11">{value:.2f}</text>',
            ])
        if deadlines is not None:
            for deadline, label, color in deadlines:
                y = plot_top + plot_height - deadline / y_max * plot_height
                body.extend([
                    f'<line x1="{panel_left}" y1="{y:.2f}" x2="{panel_left + plot_width}" y2="{y:.2f}" stroke="{color}" stroke-width="2" stroke-dasharray="6 4"/>',
                    f'<text x="{panel_left + plot_width - 4}" y="{y - 7:.2f}" text-anchor="end" font-size="12" fill="{color}">{label} {deadline:.0f} ms</text>',
                ])
        for index, (row, value, error) in enumerate(zip(summaries, values, errors)):
            center = panel_left + 120 + index * 170
            bar_width = 75
            bar_height = value / y_max * plot_height
            y = plot_top + plot_height - bar_height
            err_top = plot_top + plot_height - min(value + error, y_max) / y_max * plot_height
            err_bottom = plot_top + plot_height - max(value - error, 0.0) / y_max * plot_height
            body.extend([
                f'<rect x="{center - bar_width / 2}" y="{y:.2f}" width="{bar_width}" height="{bar_height:.2f}" fill="{COLORS[row["controller"]]}"/>',
                f'<line x1="{center}" y1="{err_top:.2f}" x2="{center}" y2="{err_bottom:.2f}" stroke="#20242a"/>',
                f'<line x1="{center - 8}" y1="{err_top:.2f}" x2="{center + 8}" y2="{err_top:.2f}" stroke="#20242a"/>',
                f'<line x1="{center - 8}" y1="{err_bottom:.2f}" x2="{center + 8}" y2="{err_bottom:.2f}" stroke="#20242a"/>',
                f'<text x="{center}" y="{y - 10:.2f}" text-anchor="middle" font-size="13">{value:.3f}</text>',
                f'<text x="{center}" y="{plot_top + plot_height + 28}" text-anchor="middle" font-size="14">{row["controller"].upper()}</text>',
            ])
    Path(output_path).write_text(_svg_document(width, height, body), encoding="utf-8")


def create_png_exports(output_dir):
    """Rasterize SVG figures when a supported local Chromium browser exists."""

    browser = next(
        (
            path
            for executable in ("google-chrome", "chromium", "chromium-browser")
            if (path := shutil.which(executable)) is not None
        ),
        None,
    )
    if browser is None:
        return []
    specs = (
        ("trajectory_comparison", 900, 680),
        ("position_error_timeseries", 1000, 520),
        ("accuracy_and_timing", 1040, 540),
    )
    exports = []
    for stem, width, height in specs:
        svg_path = (Path(output_dir) / f"{stem}.svg").resolve()
        png_path = (Path(output_dir) / f"{stem}.png").resolve()
        try:
            subprocess.run(
                [
                    browser,
                    "--headless",
                    "--disable-gpu",
                    "--no-sandbox",
                    "--hide-scrollbars",
                    "--force-device-scale-factor=2",
                    f"--window-size={width},{height}",
                    f"--screenshot={png_path}",
                    svg_path.as_uri(),
                ],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.CalledProcessError):
            continue
        exports.append(png_path.name)
    return exports


def create_english_png_exports(output_dir, trajectories, summaries, seed):
    """Create an English-only PNG folder without retaining intermediate SVGs."""

    english_dir = Path(output_dir) / "english"
    if english_dir.exists():
        shutil.rmtree(english_dir)
    english_dir.mkdir(parents=True)
    with tempfile.TemporaryDirectory() as directory:
        temporary = Path(directory)
        create_trajectory_figure(
            trajectories,
            temporary / "trajectory_comparison.svg",
            seed,
            language="en",
        )
        create_position_error_figure(
            trajectories,
            temporary / "position_error_timeseries.svg",
            seed,
            language="en",
        )
        create_metric_figure(
            summaries,
            temporary / "accuracy_and_timing.svg",
            language="en",
        )
        exported = create_png_exports(temporary)
        for filename in exported:
            shutil.move(str(temporary / filename), english_dir / filename)
    if not any(english_dir.iterdir()):
        english_dir.rmdir()
        return []
    return [f"english/{filename}" for filename in sorted(exported)]


def _write_report(output_dir, summaries, metadata, representative_seed, full_lap):
    by_controller = {row["controller"]: row for row in summaries}
    mpc = by_controller.get("mpc")
    mppi = by_controller.get("mppi")
    lines = [
        "# 発表用 MPC・MPPI 同条件比較",
        "",
        "## 実験条件",
        "",
        "- コース: 直径4 mの円形",
        "- 目標速度: 0.30 m/s",
        "- 車体: nominal actuator profile",
        "- 姿勢入力: ground truth",
        "- LiDAR: ideal",
        "- 通信: normal",
        f"- seed: {', '.join(str(seed) for seed in metadata['seeds'])}",
        f"- 評価長: {'1周' if full_lap else str(metadata['max_steps']) + ' steps'}",
        "- 実GPIO・実PWM・実LiDAR: 使用していない",
        "",
        "## 結果",
        "",
        "| Controller | Runs | Completion | Position RMSE [m] | p95 [ms] | Max [ms] | Deadline count |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summaries:
        lines.append(
            f"| {row['controller'].upper()} | {row['run_count']} | "
            f"{row['completion_rate']:.3f} | "
            f"{row['position_rmse_mean_m']:.4f} +/- {row['position_rmse_std_m']:.4f} | "
            f"{row['controller_p95_mean_ms']:.3f} +/- "
            f"{row['controller_p95_std_ms']:.3f} | "
            f"{row['controller_max_ms']:.3f} | "
            f"{row['deadline_exceeded_total']} |"
        )
    if mpc is not None and mppi is not None:
        accuracy_ratio = mppi["position_rmse_mean_m"] / max(
            mpc["position_rmse_mean_m"], 1e-12
        )
        timing_ratio = mpc["controller_p95_mean_ms"] / max(
            mppi["controller_p95_mean_ms"], 1e-12
        )
        lines.extend([
            "",
            "## 結果の読み方",
            "",
            f"- 両Controllerとも完了率は{mpc['completion_rate']:.0%}。",
            f"- MPCの位置RMSEはMPPIより約{accuracy_ratio:.1f}倍小さい。",
            f"- MPPIのp95計算時間はMPCより約{timing_ratio:.1f}倍短い。",
            f"- MPCの最大計算時間は{mpc['controller_max_ms']:.2f} msで、"
            f"20 ms deadline超過は合計{mpc['deadline_exceeded_total']}回。",
            f"- 最終位相の平均絶対誤差はMPC {mpc['circle_phase_abs_error_mean_rad']:.4f} rad、"
            f"MPPI {mppi['circle_phase_abs_error_mean_rad']:.4f} rad。",
            "",
            "軌跡形状だけを見ると両者とも円に近いが、位置RMSEは時刻ごとの参照位置との距離を",
            "含む。MPPIは主に進行方向の位相遅れが蓄積し、MPCより位置RMSEが大きくなった。",
            "したがって結論は「MPCは高精度、MPPIは高速」であり、どちらかが全面的に優れる",
            "という意味ではない。",
        ])
    lines.extend([
        "",
        "## グラフ",
        "",
        f"代表軌跡はseed {representative_seed}を使用し、集計値は全seedを使用する。",
        "",
        "![trajectory](trajectory_comparison.svg)",
        "",
        "![position_error](position_error_timeseries.svg)",
        "",
        "![metrics](accuracy_and_timing.svg)",
        "",
        "## 成果物",
        "",
        "- `runs.csv`: seedごとの集計前データ",
        "- `summary.csv`: controllerごとの平均・標準偏差",
        "- `trajectories/`: 各runの時系列データ",
        "- `metadata.json`: 設定hash、実行環境、実行時間",
        "- `*.svg`: 拡大しても劣化しない発表用グラフ",
        "- `*.png`: Chromiumが利用可能な場合に生成する高解像度グラフ",
        "- `english/*.png`: 英語版の高解像度グラフ3枚",
        "",
        "## 解釈上の注意",
        "",
        "この結果はPC上の仮想車両による比較であり、Raspberry Piの実行時間、実LiDARの",
        "誤差、実タイヤ、実PWM、実車の走行性能を保証しない。姿勢入力をground truthへ固定",
        "しているため、この比較はlocalizerではなくcontrollerとnominal車体モデルの比較である。",
        "",
    ])
    report_path = Path(output_dir) / "report.md"
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return report_path


def run_presentation_experiment(
    vehicle_config=DEFAULT_CONFIG,
    campaign_config=DEFAULT_CAMPAIGN,
    output_dir=DEFAULT_OUTPUT,
    seeds=(123, 456, 789),
    max_steps=0,
    overwrite=False,
):
    output_dir = _prepare_output(output_dir, overwrite)
    config, parameters, limits = load_real_config(vehicle_config)
    if bool(config["simulation"].get("hardware_output_enabled", False)):
        raise RuntimeError("presentation experiment requires hardware_output_enabled=false")
    campaign = copy.deepcopy(load_campaign(campaign_config))
    campaign["campaign"]["save_trajectories"] = True
    seeds = tuple(int(seed) for seed in seeds)
    results = []
    started = time.perf_counter()
    cases = [_case(controller, seed) for seed in seeds for controller in CONTROLLERS]
    for index, case in enumerate(cases, 1):
        summary = run_case(
            case,
            config,
            campaign,
            parameters,
            limits,
            output_dir=output_dir,
            max_steps=max_steps,
        )
        results.append(summary)
        print(
            f"[{index}/{len(cases)}] {summary['run_id']}: "
            f"completed={summary['completed']}, "
            f"rmse={summary['position_rmse_m']:.4f} m, "
            f"p95={summary['controller_p95_ms']:.3f} ms"
        )
    write_rows(output_dir / "runs.csv", results)
    summaries = aggregate_results(results)
    write_rows(output_dir / "summary.csv", summaries)
    (output_dir / "summary.json").write_text(
        json.dumps({"controllers": summaries}, indent=2) + "\n",
        encoding="utf-8",
    )

    representative_seed = seeds[0]
    trajectories = {}
    for controller in CONTROLLERS:
        run_id = _case(controller, representative_seed)
        run_id = "_".join(str(run_id[key]) for key in (
            "controller",
            "scenario",
            "pose_source",
            "lidar_profile",
            "plant_profile",
            "network_profile",
            "seed",
        ))
        trajectories[controller] = _read_trajectory(
            output_dir / "trajectories" / f"{run_id}.csv"
        )
    create_trajectory_figure(
        trajectories,
        output_dir / "trajectory_comparison.svg",
        representative_seed,
    )
    create_position_error_figure(
        trajectories,
        output_dir / "position_error_timeseries.svg",
        representative_seed,
    )
    create_metric_figure(summaries, output_dir / "accuracy_and_timing.svg")
    png_exports = create_png_exports(output_dir)
    english_png_exports = create_english_png_exports(
        output_dir,
        trajectories,
        summaries,
        representative_seed,
    )

    try:
        import scipy
        scipy_version = scipy.__version__
    except ImportError:
        scipy_version = "unavailable"
    metadata = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "git_revision": _git_revision(),
        "git_worktree_dirty": _git_worktree_dirty(),
        "python_version": platform.python_version(),
        "numpy_version": np.__version__,
        "scipy_version": scipy_version,
        "vehicle_config": str(Path(vehicle_config)),
        "vehicle_config_sha256": _sha256(vehicle_config),
        "campaign_config": str(Path(campaign_config)),
        "campaign_config_sha256": _sha256(campaign_config),
        "source_sha256": _source_hashes(),
        "seeds": list(seeds),
        "run_count": len(results),
        "max_steps": int(max_steps),
        "wall_time_s": time.perf_counter() - started,
        "condition": {
            "scenario": "circle",
            "pose_source": "ground_truth",
            "plant_profile": "nominal",
            "lidar_profile": "ideal",
            "network_profile": "normal",
            "speed_mps": float(config["controller"]["speed_target_mps"]),
            "circle_radius_m": float(config["simulation"]["circle_radius_m"]),
        },
        "hardware_output_enabled": False,
        "png_exports": png_exports,
        "english_png_exports": english_png_exports,
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )
    report = _write_report(
        output_dir,
        summaries,
        metadata,
        representative_seed,
        full_lap=max_steps == 0,
    )
    print(f"report: {report}")
    return report


def main():
    parser = argparse.ArgumentParser(
        description="Run a matched MPC/MPPI circular presentation experiment."
    )
    parser.add_argument("--vehicle-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--campaign-config", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--seeds", default="123,456,789")
    parser.add_argument(
        "--max-steps",
        type=int,
        default=0,
        help="Limit each run for a smoke check; 0 runs a full lap.",
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.max_steps < 0:
        parser.error("--max-steps must be non-negative")
    run_presentation_experiment(
        vehicle_config=args.vehicle_config,
        campaign_config=args.campaign_config,
        output_dir=args.output_dir,
        seeds=_parse_seeds(args.seeds),
        max_steps=args.max_steps,
        overwrite=args.overwrite,
    )


if __name__ == "__main__":
    main()
