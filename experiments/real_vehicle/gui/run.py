"""Launch the interactive real-vehicle simulation GUI."""

import argparse
from pathlib import Path

import pyglet
from pyglet.window import key

from ..evaluation.common import DEFAULT_CONFIG, load_real_config
from .renderer import SimulationRenderer
from .simulation_view import SimulationView


class SimulationWindow:
    def __init__(self, args):
        config, parameters, limits = load_real_config(args.config)
        self.simulation = SimulationView(
            config,
            parameters,
            limits,
            controller_name=args.controller,
            scenario=args.scenario,
            noise_std=args.noise_std,
            dropout_probability=args.dropout_probability,
            delay_s=args.delay_s,
            initial_x_error=args.initial_x_error,
            initial_y_error=args.initial_y_error,
            initial_heading_error=args.initial_heading_error,
            complex_map=args.complex_map,
            max_steps=args.max_steps,
        )
        self.output = args.output
        self.sim_speed = max(float(args.sim_speed), 0.05)
        self.paused = False
        self.saved = False
        self.window = pyglet.window.Window(
            width=args.window_width,
            height=args.window_height,
            caption="Real Vehicle Simulation",
            resizable=True,
        )
        self.renderer = SimulationRenderer(self.window, self.simulation, scale=args.scale)
        self.renderer.update(self.simulation.frame)
        self.window.push_handlers(self)
        pyglet.clock.schedule_interval(self._tick, self.simulation.dt / self.sim_speed)

    def _tick(self, _elapsed):
        if not self.paused:
            frame = self.simulation.step()
            self.renderer.update(frame)
            if frame["finished"]:
                self.paused = True
                self._save()

    def on_draw(self):
        self.renderer.draw()

    def on_resize(self, width, height):
        self.renderer.resize(width, height)

    def on_key_press(self, symbol, _modifiers):
        if symbol == key.SPACE:
            self.paused = not self.paused
        elif symbol == key.R:
            self.simulation.reset()
            self.renderer.update(self.simulation.frame)
            self.paused = False
            self.saved = False
        elif symbol in (key.PLUS, key.NUM_ADD, key.EQUAL):
            self.renderer.set_scale(min(self.renderer.scale * 1.1, 180.0))
        elif symbol in (key.MINUS, key.NUM_SUBTRACT):
            self.renderer.set_scale(max(self.renderer.scale / 1.1, 20.0))
        elif symbol in (key.ESCAPE, key.Q):
            self._save()
            self.window.close()

    def on_close(self):
        self._save()
        pyglet.clock.unschedule(self._tick)

    def _save(self):
        if self.saved:
            return
        summary = self.simulation.save(self.output)
        self.saved = True
        print(f"summary: {summary}")
        print(f"csv: {self.output}")


def parse_args():
    parser = argparse.ArgumentParser(description="Interactive real-vehicle simulation GUI")
    parser.add_argument("--controller", choices=("mpc", "mppi"), default="mppi")
    parser.add_argument("--scenario", choices=("straight", "circle", "localized"), default="straight")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=Path("experiments/real_vehicle/results/gui.csv"))
    parser.add_argument("--noise-std", type=float, default=None)
    parser.add_argument("--dropout-probability", type=float, default=None)
    parser.add_argument("--delay-s", type=float, default=None)
    parser.add_argument("--initial-x-error", type=float, default=0.0)
    parser.add_argument("--initial-y-error", type=float, default=0.0)
    parser.add_argument("--initial-heading-error", type=float, default=0.0)
    parser.add_argument("--complex-map", action="store_true")
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--sim-speed", type=float, default=1.0, help="simulation speed relative to real time")
    parser.add_argument("--scale", type=float, default=70.0, help="pixels per meter")
    parser.add_argument("--window-width", type=int, default=1280)
    parser.add_argument("--window-height", type=int, default=820)
    return parser.parse_args()


def main():
    args = parse_args()
    app = SimulationWindow(args)
    del app
    pyglet.app.run()


if __name__ == "__main__":
    main()
