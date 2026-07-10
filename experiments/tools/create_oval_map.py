"""F1TENTH Gym用の楕円形周回コースを生成する。"""

from argparse import ArgumentParser
from pathlib import Path
import math

import numpy as np
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parents[2]


def create_map_image(width, height, track_width, resolution):
    """黒い壁と白い走行領域からなる楕円形コース画像を作る。"""
    image_width = round(width / resolution)
    image_height = round(height / resolution)

    x = (np.arange(image_width) + 0.5) * resolution - width / 2.0
    y = (np.arange(image_height) + 0.5) * resolution - height / 2.0
    grid_x, grid_y = np.meshgrid(x, y)

    outer_a = width / 2.0
    outer_b = height / 2.0
    inner_a = outer_a - track_width
    inner_b = outer_b - track_width

    outer_ellipse = (grid_x / outer_a) ** 2 + (grid_y / outer_b) ** 2 <= 1.0
    inner_ellipse = (grid_x / inner_a) ** 2 + (grid_y / inner_b) ** 2 < 1.0
    road = outer_ellipse & ~inner_ellipse

    return Image.fromarray(np.where(road, 255, 0).astype(np.uint8), mode="L")


def create_waypoints(width, height, track_width, speed, spacing=0.08):
    """楕円の中心線を反時計回りに走るPure Pursuit用waypointを作る。"""
    center_a = width / 2.0 - track_width / 2.0
    center_b = height / 2.0 - track_width / 2.0
    circumference = math.pi * (
        3.0 * (center_a + center_b)
        - math.sqrt((3.0 * center_a + center_b) * (center_a + 3.0 * center_b))
    )
    count = max(32, round(circumference / spacing))
    angles = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)

    xs = center_a * np.cos(angles)
    ys = center_b * np.sin(angles)
    headings = np.arctan2(center_b * np.cos(angles), -center_a * np.sin(angles))
    curvature = (center_a * center_b) / (
        (center_a * np.sin(angles)) ** 2 + (center_b * np.cos(angles)) ** 2
    ) ** 1.5

    segment_lengths = np.hypot(np.roll(xs, -1) - xs, np.roll(ys, -1) - ys)
    distances = np.concatenate(([0.0], np.cumsum(segment_lengths[:-1])))

    rows = [
        "# F1TENTH Gym oval course waypoints",
        "# Counter-clockwise centerline",
        "# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2",
    ]
    rows.extend(
        f"{s:.7f}; {x:.7f}; {y:.7f}; {heading:.7f}; {kappa:.7f}; {speed:.7f}; 0.0000000"
        for s, x, y, heading, kappa in zip(distances, xs, ys, headings, curvature)
    )
    return "\n".join(rows) + "\n", center_a, center_b


def validate_arguments(width, height, track_width, resolution, speed):
    """生成する地図が有効な寸法か確認する。"""
    if min(width, height, track_width, resolution, speed) <= 0.0:
        raise ValueError("width、height、track-width、resolution、speed は正の値にしてください。")
    if track_width >= min(width, height) / 2.0:
        raise ValueError("track-width は width と height の小さい方の半分より小さくしてください。")
    if not math.isclose(round(width / resolution) * resolution, width, abs_tol=1e-9):
        raise ValueError("width は resolution で割り切れる値にしてください。")
    if not math.isclose(round(height / resolution) * resolution, height, abs_tol=1e-9):
        raise ValueError("height は resolution で割り切れる値にしてください。")


def main():
    parser = ArgumentParser(description="F1TENTH Gym用の楕円形周回コースを生成する。")
    parser.add_argument("--width", type=float, default=7.0, help="外側楕円の幅 [m]。")
    parser.add_argument("--height", type=float, default=5.0, help="外側楕円の高さ [m]。")
    parser.add_argument("--track-width", type=float, default=1.0, help="走路幅 [m]。")
    parser.add_argument("--resolution", type=float, default=0.01, help="地図画像の解像度 [m/pixel]。")
    parser.add_argument("--speed", type=float, default=1.0, help="waypointに記録する目標速度 [m/s]。")
    parser.add_argument("--name", default="homur_oval", help="生成するファイルのベース名。")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "experiments" / "maps",
        help="地図ファイルを出力するフォルダ。",
    )
    args = parser.parse_args()

    validate_arguments(args.width, args.height, args.track_width, args.resolution, args.speed)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    map_base = args.output_dir / args.name
    image_path = map_base.with_suffix(".png")
    yaml_path = map_base.with_suffix(".yaml")
    waypoint_path = args.output_dir / f"{args.name}_waypoints.csv"

    create_map_image(args.width, args.height, args.track_width, args.resolution).save(image_path)
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {image_path.name}",
                f"resolution: {args.resolution:.6f}",
                f"origin: [{-args.width / 2.0:.6f}, {-args.height / 2.0:.6f}, 0.000000]",
                "negate: 0",
                "occupied_thresh: 0.45",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )
    waypoints, center_a, center_b = create_waypoints(
        args.width,
        args.height,
        args.track_width,
        args.speed,
    )
    waypoint_path.write_text(waypoints, encoding="utf-8")

    print(f"map image: {image_path}")
    print(f"map yaml: {yaml_path}")
    print(f"waypoints: {waypoint_path}")
    print(f"centerline start pose: x={center_a:.3f}, y=0.000, theta={math.pi / 2.0:.6f}")


if __name__ == "__main__":
    main()
