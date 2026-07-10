from argparse import ArgumentParser, Namespace
from pathlib import Path
import sys
import time

import gym
import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from examples.waypoint_follow import PurePursuitPlanner  # noqa: E402
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
    args = parser.parse_args()

    config_path = resolve_repo_path(args.config)
    conf = load_config(config_path)

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

    if not args.no_render:
        env.render()

    laptime = 0.0
    step_count = 0
    start = time.time()

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

        if not args.no_render:
            env.render(mode="human")

        if args.max_steps > 0 and step_count >= args.max_steps:
            break

    print(f"run_name: {conf.run_name}")
    print(f"steps: {step_count}")
    print(f"sim_elapsed_time: {laptime}")
    print(f"real_elapsed_time: {time.time() - start}")
    print(f"done: {done}")


if __name__ == "__main__":
    main()
