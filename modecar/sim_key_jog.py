# Keyboard jog operation for the F1TENTH Gym simulator.
#
# This is the simulator counterpart of timer_key_jog.py.
# It does not use pigpio or GPIO. It converts duty-like references to
# F1TENTH Gym actions: [steering_angle_rad, speed_mps].

import os
import sys
import time
from types import SimpleNamespace

import gym
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import f110_gym  # noqa: F401 - registers f110_gym:f110-v0
import utilities
from f110_gym.envs.base_classes import Integrator


# Keyboard jog increments in duty-like units.
SPEED_DELTA_JOG = 0.08
STEER_DELTA_JOG = 0.02
PWM_NEUTRAL_STR = 10.88
PWM_NEUTRAL_SPD = 10.55

# Conservative simulator limits.
MAX_STEER_RAD = 0.4189
MAX_FORWARD_SPEED_MPS = 0.25
MAX_REVERSE_SPEED_MPS = 0.25

# Approximate calibration points used only for simulator conversion.
# In timer_key_jog.py, w decreases spd_ref and a decreases str_ref.
STR_LEFT_DUTY = 8.88
STR_RIGHT_DUTY = 12.88
SPD_FORWARD_DUTY = 9.50
SPD_REVERSE_DUTY = 11.50

# Default map used when running:
#
#   python sim_key_jog.py
#
# Change the active line below to choose another bundled map.
# This default matches examples/waypoint_follow.py.
ACTIVE_MAP_NAME = "example"
# ACTIVE_MAP_NAME = "vegas"
# ACTIVE_MAP_NAME = "berlin"
# ACTIVE_MAP_NAME = "skirk"
# ACTIVE_MAP_NAME = "stata_basement"
# ACTIVE_MAP_NAME = "levine"

# Runtime settings used when running:
#
#   python sim_key_jog.py
#
# Edit these values in code instead of passing command-line options.
INITIAL_X = 0.7
INITIAL_Y = 0.0
INITIAL_THETA = 1.37079632679
RUN_SECONDS = 6000
TIMESTEP = 0.01
LOOP_HZ = 0.0  # 0 disables artificial loop sleep, like examples/waypoint_follow.py.
RENDER_FPS = 30.0
VIEW_MODE = "follow"  # "full" shows the whole map, "follow" follows the car.
CAMERA_RADIUS = 800.0
MAP_PADDING = 150.0
ENABLE_RENDER = True
DARK_BACKGROUND = False
TERMINAL_POLL_HZ = 30.0
STATUS_PRINT_HZ = 2.0
HELP_PRINT_SECONDS = 30.0

MAP_EXTENSIONS = {
    "berlin": ".png",
    "example": ".png",
    "levine": ".pgm",
    "skirk": ".png",
    "stata_basement": ".png",
    "vegas": ".png",
}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def duty_to_steer_rad(str_duty):
    """Convert steering duty-like value to front wheel steering angle [rad]."""
    if str_duty < PWM_NEUTRAL_STR:
        ratio = (PWM_NEUTRAL_STR - str_duty) / (PWM_NEUTRAL_STR - STR_LEFT_DUTY)
        return clamp(ratio, 0.0, 1.0) * MAX_STEER_RAD

    ratio = (str_duty - PWM_NEUTRAL_STR) / (STR_RIGHT_DUTY - PWM_NEUTRAL_STR)
    return -clamp(ratio, 0.0, 1.0) * MAX_STEER_RAD


def duty_to_speed_mps(spd_duty):
    """Convert ESC duty-like value to target speed [m/s]."""
    if spd_duty < PWM_NEUTRAL_SPD:
        ratio = (PWM_NEUTRAL_SPD - spd_duty) / (PWM_NEUTRAL_SPD - SPD_FORWARD_DUTY)
        return clamp(ratio, 0.0, 1.0) * MAX_FORWARD_SPEED_MPS

    ratio = (spd_duty - PWM_NEUTRAL_SPD) / (SPD_REVERSE_DUTY - PWM_NEUTRAL_SPD)
    return -clamp(ratio, 0.0, 1.0) * MAX_REVERSE_SPEED_MPS


def map_base_path(map_name):
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if map_name == "example":
        return os.path.join(repo_root, "examples", "example_map")
    return os.path.join(repo_root, "gym", "f110_gym", "envs", "maps", map_name)


def get_settings():
    if ACTIVE_MAP_NAME not in MAP_EXTENSIONS:
        raise ValueError("Unknown ACTIVE_MAP_NAME: {}".format(ACTIVE_MAP_NAME))
    if VIEW_MODE not in ("follow", "full"):
        raise ValueError("VIEW_MODE must be 'follow' or 'full': {}".format(VIEW_MODE))

    return SimpleNamespace(
        map_name=ACTIVE_MAP_NAME,
        map_path=map_base_path(ACTIVE_MAP_NAME),
        map_ext=MAP_EXTENSIONS[ACTIVE_MAP_NAME],
        x=INITIAL_X,
        y=INITIAL_Y,
        theta=INITIAL_THETA,
        run_seconds=RUN_SECONDS,
        timestep=TIMESTEP,
        loop_hz=LOOP_HZ,
        render_fps=RENDER_FPS,
        view=VIEW_MODE,
        camera_radius=CAMERA_RADIUS,
        map_padding=MAP_PADDING,
        render_enabled=ENABLE_RENDER,
        dark_background=DARK_BACKGROUND,
        terminal_poll_hz=TERMINAL_POLL_HZ,
        status_print_hz=STATUS_PRINT_HZ,
        help_print_seconds=HELP_PRINT_SECONDS,
    )


def write_help(run_seconds):
    print(
        "Enter: stop, w/s: speed, a/d: steering, n: neutral, "
        "auto stop after {:.1f} s".format(run_seconds)
    )


def read_key():
    if not sys.stdin.isatty():
        return 0
    return utilities.getkey()


class WindowKeyReader:
    def __init__(self):
        self._keys = []

    def on_key_press(self, symbol, modifiers):
        from pyglet.window import key

        key_map = {
            key.W: ord("w"),
            key.S: ord("s"),
            key.A: ord("a"),
            key.D: ord("d"),
            key.N: ord("n"),
            key.ENTER: 10,
            key.ESCAPE: 10,
        }
        mapped_key = key_map.get(symbol)
        if mapped_key is not None:
            self._keys.append(mapped_key)

    def getkey(self):
        if not self._keys:
            return 0
        return self._keys.pop(0)


def make_camera_callback(view, camera_radius, map_padding, light_background=True):
    def update_camera(renderer):
        from pyglet.gl import glClearColor

        if light_background:
            glClearColor(232 / 255, 236 / 255, 240 / 255, 1.0)
            label_color = (20, 24, 31, 255)
        else:
            glClearColor(9 / 255, 32 / 255, 87 / 255, 1.0)
            label_color = (255, 255, 255, 255)

        if view == "full" and renderer.map_points is not None:
            xs = renderer.map_points[:, 0]
            ys = renderer.map_points[:, 1]
            renderer.left = float(xs.min() - map_padding)
            renderer.right = float(xs.max() + map_padding)
            renderer.bottom = float(ys.min() - map_padding)
            renderer.top = float(ys.max() + map_padding)
            renderer.score_label.x = renderer.left + 40
            renderer.score_label.y = renderer.top - 80
            renderer.score_label.color = label_color
            return

        if not hasattr(renderer, "cars") or not renderer.cars:
            return

        if hasattr(renderer.cars[0], "colors"):
            renderer.cars[0].colors = [220, 35, 35] * 4

        vertices = renderer.cars[0].vertices
        xs = vertices[::2]
        ys = vertices[1::2]
        center_x = 0.5 * (min(xs) + max(xs))
        center_y = 0.5 * (min(ys) + max(ys))

        renderer.left = center_x - camera_radius
        renderer.right = center_x + camera_radius
        renderer.bottom = center_y - camera_radius
        renderer.top = center_y + camera_radius
        renderer.score_label.x = center_x
        renderer.score_label.y = renderer.top - 25
        renderer.score_label.color = label_color

    return update_camera


def main():
    settings = get_settings()

    env = gym.make(
        "f110_gym:f110-v0",
        map=settings.map_path,
        map_ext=settings.map_ext,
        num_agents=1,
        timestep=settings.timestep,
        integrator=Integrator.RK4,
    )
    if settings.render_enabled:
        env.add_render_callback(
            make_camera_callback(
                settings.view,
                settings.camera_radius,
                settings.map_padding,
                light_background=not settings.dark_background,
            )
        )

    poses = np.array([[settings.x, settings.y, settings.theta]])
    obs, step_reward, done, info = env.reset(poses)

    spd_ref = PWM_NEUTRAL_SPD
    str_ref = PWM_NEUTRAL_STR
    render_enabled = settings.render_enabled
    window_keys = WindowKeyReader()

    print("F1TENTH Gym keyboard jog control...")
    print("map={}, view={}, pose=({}, {}, {})".format(settings.map_path, settings.view, settings.x, settings.y, settings.theta))
    write_help(settings.run_seconds)

    start_time = time.monotonic()
    next_render_time = start_time
    next_terminal_poll_time = start_time
    next_status_print_time = start_time
    next_help_print_time = start_time + settings.help_print_seconds
    sim_time = 0.0
    i = 0
    loop_sleep = 0.0 if settings.loop_hz <= 0 else 1.0 / settings.loop_hz
    render_interval = 0.0 if settings.render_fps <= 0 else 1.0 / settings.render_fps
    terminal_poll_interval = 0.0 if settings.terminal_poll_hz <= 0 else 1.0 / settings.terminal_poll_hz
    status_print_interval = 0.0 if settings.status_print_hz <= 0 else 1.0 / settings.status_print_hz

    if render_enabled:
        try:
            print("initializing renderer...", flush=True)
            env.render(mode="human")
            env.unwrapped.renderer.push_handlers(on_key_press=window_keys.on_key_press)
            print("renderer ready.", flush=True)
            next_render_time = time.monotonic() + render_interval
        except Exception as exc:
            render_enabled = False
            print("render disabled: {}: {}".format(type(exc).__name__, exc))

    try:
        while not done:
            loop_started = time.monotonic()
            i += 1

            now = time.monotonic()
            if now - start_time >= settings.run_seconds:
                print("time limit reached")
                break

            key = window_keys.getkey()
            if not key and now >= next_terminal_poll_time:
                key = read_key()
                next_terminal_poll_time = now + terminal_poll_interval
            if key == 10:
                break
            if key == ord("w"):
                spd_ref -= SPEED_DELTA_JOG * 2
            elif key == ord("s"):
                spd_ref += SPEED_DELTA_JOG * 2
            elif key == ord("a"):
                str_ref -= STEER_DELTA_JOG * 4
            elif key == ord("d"):
                str_ref += STEER_DELTA_JOG * 4
            elif key == ord("n"):
                str_ref = PWM_NEUTRAL_STR
                spd_ref = PWM_NEUTRAL_SPD

            steer_rad = duty_to_steer_rad(str_ref)
            speed_mps = duty_to_speed_mps(spd_ref)
            action = np.array([[steer_rad, speed_mps]])

            obs, step_reward, done, info = env.step(action)
            sim_time += step_reward

            now = time.monotonic()
            if render_enabled and now >= next_render_time:
                try:
                    env.render(mode="human")
                    next_render_time = now + render_interval
                except Exception as exc:
                    render_enabled = False
                    print("render disabled: {}: {}".format(type(exc).__name__, exc))

            if settings.status_print_hz > 0 and now >= next_status_print_time:
                print(
                    "duty(str,spd)=({:.3f}, {:.3f}) action(steer,speed)=({:.3f}, {:.3f}) sim_time={:.2f}".format(
                        str_ref,
                        spd_ref,
                        steer_rad,
                        speed_mps,
                        sim_time,
                    )
                )
                next_status_print_time = now + status_print_interval
            if settings.help_print_seconds > 0 and now >= next_help_print_time:
                write_help(settings.run_seconds)
                next_help_print_time = now + settings.help_print_seconds

            elapsed = time.monotonic() - loop_started
            if loop_sleep > 0:
                time.sleep(max(0.0, loop_sleep - elapsed))

    except KeyboardInterrupt:
        print("stop!")
    finally:
        neutral_action = np.array([[0.0, 0.0]])
        env.step(neutral_action)
        if hasattr(env, "close"):
            env.close()

    print("finish.")


if __name__ == "__main__":
    main()
