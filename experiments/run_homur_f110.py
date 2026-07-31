from argparse import ArgumentParser, Namespace
import csv
from datetime import datetime
from pathlib import Path
import sys
import time

import gym
import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from examples.waypoint_follow import PurePursuitPlanner, nearest_point_on_trajectory  # noqa: E402
from f110_gym.envs.base_classes import Integrator  # noqa: E402


def resolve_repo_path(path_text):
    """リポジトリ直下からの相対パスを絶対パスへ変換する。"""
    path = Path(path_text)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def load_config(config_path):
    """YAML設定を読み込み、マップとwaypointのパスを絶対パスへ変換する。"""
    with open(config_path, encoding="utf-8") as file:
        conf_dict = yaml.safe_load(file)

    conf_dict["map_path"] = str(resolve_repo_path(conf_dict["map_path"]))
    conf_dict["wpt_path"] = str(resolve_repo_path(conf_dict["wpt_path"]))
    return Namespace(**conf_dict)


def get_integrator(name):
    """設定ファイルの文字列からIntegrator enumを取得する。"""
    try:
        return getattr(Integrator, name)
    except AttributeError as exc:
        choices = ", ".join(item.name for item in Integrator)
        raise ValueError(f"integrator は {choices} のどれかを指定してください: {name}") from exc


def normalize_angle(angle):
    """角度を [-pi, pi) に正規化する。"""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class RunLogger:
    """waypoint基準の追従誤差を計算し、CSVへ保存する。"""

    fieldnames = (
        "step",
        "sim_time",
        "lap_count",
        "lap_time",
        "pose_x",
        "pose_y",
        "pose_theta",
        "speed_cmd",
        "steer_cmd",
        "linear_vel_x",
        "linear_vel_y",
        "ang_vel_z",
        "cross_track_error",
        "heading_error",
        "progress_distance",
        "ref_x",
        "ref_y",
        "ref_heading",
        "scan_min",
        "scan_mean",
        "scan_max",
        "scan_updated",
        "scan_time",
        "scan_age",
        "scan_update_count",
        "collision",
        "done",
    )

    def __init__(self, planner, config_name, run_name, results_dir):
        self.planner = planner
        self.rows = []
        self.results_dir = Path(results_dir)
        self.results_dir.mkdir(parents=True, exist_ok=True)

        waypoint_xy = np.column_stack(
            (
                planner.waypoints[:, planner.conf.wpt_xind],
                planner.waypoints[:, planner.conf.wpt_yind],
            )
        )
        self.waypoint_xy = np.vstack((waypoint_xy, waypoint_xy[0]))
        diffs = self.waypoint_xy[1:] - self.waypoint_xy[:-1]
        self.segment_lengths = np.linalg.norm(diffs, axis=1)
        self.cumulative_lengths = np.concatenate(([0.0], np.cumsum(self.segment_lengths)))

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_label = run_name.strip().replace(" ", "_")
        config_label = config_name.strip().replace(" ", "_")
        if run_label == config_label:
            stem = f"{timestamp}_{run_label}"
        else:
            stem = f"{timestamp}_{run_label}_{config_label}"
        self.output_path = self.results_dir / f"{stem}.csv"

    def compute_metrics(self, pose_x, pose_y, pose_theta):
        """現在姿勢に対する中心線基準の誤差を返す。"""
        position = np.array([pose_x, pose_y])
        nearest_point, nearest_dist, t, segment_index = nearest_point_on_trajectory(
            position, self.waypoint_xy
        )
        segment_start = self.waypoint_xy[segment_index]
        segment_end = self.waypoint_xy[segment_index + 1]
        segment_vector = segment_end - segment_start
        ref_heading = np.arctan2(segment_vector[1], segment_vector[0])

        offset = position - nearest_point
        cross = segment_vector[0] * offset[1] - segment_vector[1] * offset[0]
        cross_track_error = float(np.sign(cross) * nearest_dist)
        heading_error = float(normalize_angle(pose_theta - ref_heading))
        progress_distance = float(
            self.cumulative_lengths[segment_index] + t * self.segment_lengths[segment_index]
        )

        return {
            "cross_track_error": cross_track_error,
            "heading_error": heading_error,
            "progress_distance": progress_distance,
            "ref_x": float(nearest_point[0]),
            "ref_y": float(nearest_point[1]),
            "ref_heading": float(ref_heading),
        }

    def record(self, step, sim_time, obs, speed_cmd, steer_cmd, done):
        """1 ステップ分の観測と指令値を保存する。"""
        pose_x = float(obs["poses_x"][0])
        pose_y = float(obs["poses_y"][0])
        pose_theta = float(obs["poses_theta"][0])
        metrics = self.compute_metrics(pose_x, pose_y, pose_theta)
        scan = np.asarray(obs["scans"][0], dtype=float)
        self.rows.append(
            {
                "step": step,
                "sim_time": float(sim_time),
                "lap_count": int(obs["lap_counts"][0]),
                "lap_time": float(obs["lap_times"][0]),
                "pose_x": pose_x,
                "pose_y": pose_y,
                "pose_theta": pose_theta,
                "speed_cmd": float(speed_cmd),
                "steer_cmd": float(steer_cmd),
                "linear_vel_x": float(obs["linear_vels_x"][0]),
                "linear_vel_y": float(obs["linear_vels_y"][0]),
                "ang_vel_z": float(obs["ang_vels_z"][0]),
                "cross_track_error": metrics["cross_track_error"],
                "heading_error": metrics["heading_error"],
                "progress_distance": metrics["progress_distance"],
                "ref_x": metrics["ref_x"],
                "ref_y": metrics["ref_y"],
                "ref_heading": metrics["ref_heading"],
                "scan_min": float(np.min(scan)),
                "scan_mean": float(np.mean(scan)),
                "scan_max": float(np.max(scan)),
                "scan_updated": int(obs["scan_updated"][0]),
                "scan_time": float(obs["scan_times"][0]),
                "scan_age": float(obs["scan_ages"][0]),
                "scan_update_count": int(obs["scan_update_counts"][0]),
                "collision": int(obs["collisions"][0]),
                "done": int(done),
            }
        )

    def write(self):
        """保存済み行を CSV ファイルへ書き出す。"""
        with self.output_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)
        return self.output_path


def main():
    parser = ArgumentParser(description="自分用設定でF1TENTH Gymを実行する。")
    parser.add_argument(
        "--config",
        default="experiments/configs/homur_f110.yaml",
        help="リポジトリ直下から見た設定ファイルのパス。",
    )
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="GUI描画を行わずに実行する。動作確認やログ確認向け。",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=0,
        help="最大ステップ数。0なら終了条件まで走らせる。",
    )
    parser.add_argument(
        "--results-dir",
        default="experiments/results",
        help="CSVログを保存するフォルダ。",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="CSVログを保存しない。",
    )
    parser.add_argument(
        "--lap-target",
        type=int,
        default=0,
        help="指定周回数に到達したら終了する。0なら無効。",
    )
    args = parser.parse_args()

    config_path = resolve_repo_path(args.config)
    conf = load_config(config_path)
    config_name = Path(args.config).stem

    car_params = conf.car_params
    controller = conf.controller
    render = getattr(conf, "render", {})
    wheelbase = car_params["lf"] + car_params["lr"]

    planner = PurePursuitPlanner(conf, wheelbase)
    env = gym.make(
        "f110_gym:f110-v0",
        map=conf.map_path,
        map_ext=conf.map_ext,
        num_agents=conf.num_agents,
        timestep=conf.timestep,
        integrator=get_integrator(conf.integrator),
        params=car_params,
        lidar_dist=conf.lidar_dist,
        lidar_config=getattr(conf, "lidar", None),
    )

    if not args.no_render:
        camera_margin = render.get("camera_margin", 800)
        window_width = render.get("window_width")
        window_height = render.get("window_height")
        window_size_applied = False
        render_style_applied = False

        def render_callback(env_renderer):
            """車両を追従するカメラ設定とwaypoint描画を行う。"""
            nonlocal render_style_applied, window_size_applied

            if (
                not window_size_applied
                and window_width is not None
                and window_height is not None
            ):
                env_renderer.set_size(int(window_width), int(window_height))
                window_size_applied = True

            if not render_style_applied:
                env_renderer.score_label.font_size = int(render.get("score_font_size", 36))
                env_renderer.fps_display.label.font_size = int(render.get("fps_font_size", 24))
                env_renderer.show_fps = bool(render.get("show_fps", True))
                env_renderer.map_point_size = float(render.get("obstacle_point_size", 2.0))

                obstacle_color = render.get("obstacle_color")
                if obstacle_color is not None:
                    env_renderer.set_obstacle_color(obstacle_color)

                render_style_applied = True

            x = env_renderer.cars[0].vertices[::2]
            y = env_renderer.cars[0].vertices[1::2]
            top, bottom, left, right = max(y), min(y), min(x), max(x)
            env_renderer.score_label.x = left - camera_margin * 0.6
            env_renderer.score_label.y = bottom - camera_margin * 0.8
            env_renderer.left = left - camera_margin
            env_renderer.right = right + camera_margin
            env_renderer.top = top + camera_margin
            env_renderer.bottom = bottom - camera_margin

            planner.render_waypoints(env_renderer)

        env.add_render_callback(render_callback)

    obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))
    run_logger = None
    if not args.no_log:
        run_logger = RunLogger(planner, config_name, conf.run_name, resolve_repo_path(args.results_dir))
        run_logger.record(
            step=0,
            sim_time=0.0,
            obs=obs,
            speed_cmd=0.0,
            steer_cmd=0.0,
            done=done,
        )

    if not args.no_render:
        env.render()

    laptime = 0.0
    step_count = 0
    start = time.time()
    lap_target_reached = False

    while not done:
        speed, steer = planner.plan(
            obs["poses_x"][0],
            obs["poses_y"][0],
            obs["poses_theta"][0],
            controller["tlad"],
            controller["vgain"],
        )
        obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
        laptime += step_reward
        step_count += 1
        if run_logger is not None:
            run_logger.record(
                step=step_count,
                sim_time=laptime,
                obs=obs,
                speed_cmd=speed,
                steer_cmd=steer,
                done=done,
            )

        if not args.no_render:
            env.render(mode="human")

        if args.lap_target > 0 and int(obs["lap_counts"][0]) >= args.lap_target:
            lap_target_reached = True
            break

        if args.max_steps > 0 and step_count >= args.max_steps:
            break

    print(f"run_name: {conf.run_name}")
    print(f"steps: {step_count}")
    print(f"sim_elapsed_time: {laptime}")
    print(f"real_elapsed_time: {time.time() - start}")
    print(f"done: {done}")
    print(f"lap_count: {int(obs['lap_counts'][0])}")
    print(f"lap_time: {float(obs['lap_times'][0])}")
    print(f"lap_target_reached: {lap_target_reached}")
    if run_logger is not None:
        log_path = run_logger.write()
        print(f"log_csv: {log_path}")


if __name__ == "__main__":
    main()
