"""Pyglet renderer for the real-vehicle simulation."""

import math

import pyglet
from pyglet.gl import GL_LINES, GL_TRIANGLES


class SimulationRenderer:
    """Draw the map, vehicle, LiDAR, localization, and controller preview."""

    def __init__(self, window, simulation, scale=70.0):
        self.window = window
        self.simulation = simulation
        self.scale = float(scale)
        self.window_width = window.width
        self.window_height = window.height
        self.panel_width = 300
        self.static_vertex_lists = []
        self.dynamic_vertex_lists = []
        self.static_batch = pyglet.graphics.Batch()
        self.dynamic_batch = pyglet.graphics.Batch()
        self.info_label = pyglet.text.Label(
            "",
            x=self.window_width - self.panel_width + 20,
            y=self.window_height - 30,
            width=self.panel_width - 35,
            multiline=True,
            anchor_x="left",
            anchor_y="top",
            font_size=13,
            color=(235, 240, 248, 255),
        )
        self.title_label = pyglet.text.Label(
            "Real Vehicle Simulation",
            x=20,
            y=self.window_height - 28,
            font_size=16,
            bold=True,
            color=(235, 240, 248, 255),
        )
        self._build_static()

    def resize(self, width, height):
        self.window_width = width
        self.window_height = height
        self.info_label.x = width - self.panel_width + 20
        self.info_label.y = height - 30
        self.title_label.y = height - 28

    def set_scale(self, scale):
        """Change zoom and rebuild the world-space static geometry."""
        self.scale = float(scale)
        for vertex_list in self.static_vertex_lists:
            vertex_list.delete()
        self.static_vertex_lists = []
        self.static_batch = pyglet.graphics.Batch()
        self._build_static()

    def _world_to_screen(self, x, y):
        config = self.simulation.config["lidar"]
        x_min = float(config.get("map_x_min_m", -5.0))
        y_min = float(config.get("map_y_min_m", -5.0))
        origin_x = 25.0 - x_min * self.scale
        origin_y = 35.0 - y_min * self.scale
        return origin_x + float(x) * self.scale, origin_y + float(y) * self.scale

    def _add_lines(self, segments, color):
        vertices = []
        colors = []
        for x1, y1, x2, y2 in segments:
            sx1, sy1 = self._world_to_screen(x1, y1)
            sx2, sy2 = self._world_to_screen(x2, y2)
            vertices.extend([sx1, sy1, sx2, sy2])
            colors.extend([*color, *color])
        if not vertices:
            return None
        vertex_list = self.static_batch.add(
            len(vertices) // 2,
            GL_LINES,
            None,
            ("v2f", vertices),
            ("c3B", colors),
        )
        self.static_vertex_lists.append(vertex_list)
        return vertex_list

    def _build_static(self):
        config = self.simulation.config["lidar"]
        x_min = float(config.get("map_x_min_m", -5.0))
        x_max = float(config.get("map_x_max_m", 5.0))
        y_min = float(config.get("map_y_min_m", -5.0))
        y_max = float(config.get("map_y_max_m", 5.0))
        self._add_lines(
            [
                (x_min, y_min, x_max, y_min),
                (x_max, y_min, x_max, y_max),
                (x_max, y_max, x_min, y_max),
                (x_min, y_max, x_min, y_min),
            ],
            (90, 100, 120),
        )
        for obstacle_x_min, obstacle_x_max, obstacle_y_min, obstacle_y_max in self.simulation.lidar.obstacles:
            self._add_lines(
                [
                    (obstacle_x_min, obstacle_y_min, obstacle_x_max, obstacle_y_min),
                    (obstacle_x_max, obstacle_y_min, obstacle_x_max, obstacle_y_max),
                    (obstacle_x_max, obstacle_y_max, obstacle_x_min, obstacle_y_max),
                    (obstacle_x_min, obstacle_y_max, obstacle_x_min, obstacle_y_min),
                ],
                (170, 100, 100),
            )
        if self.simulation.scenario == "circle":
            points = []
            for index in range(128):
                a = 2.0 * math.pi * index / 128.0
                b = 2.0 * math.pi * (index + 1) / 128.0
                points.append((
                    self.simulation.radius * math.cos(a),
                    self.simulation.radius * math.sin(a),
                    self.simulation.radius * math.cos(b),
                    self.simulation.radius * math.sin(b),
                ))
        else:
            points = []
            for index in range(80):
                x1 = self.simulation.goal_x * index / 79.0
                x2 = self.simulation.goal_x * (index + 1) / 79.0 if index < 79 else x1
                points.append((x1, 0.0, x2, 0.0))
        self._add_lines(points, (220, 220, 220))

    def _dynamic_add(self, count, mode, vertices, colors):
        vertex_list = self.dynamic_batch.add(
            count,
            mode,
            None,
            ("v2f", vertices),
            ("c3B", colors),
        )
        self.dynamic_vertex_lists.append(vertex_list)
        return vertex_list

    def _clear_dynamic(self):
        for vertex_list in self.dynamic_vertex_lists:
            vertex_list.delete()
        self.dynamic_vertex_lists = []
        self.dynamic_batch = pyglet.graphics.Batch()

    def _draw_vehicle(self, state, color):
        phi, x, y, _ = [float(value) for value in state]
        length = 0.48
        width = 0.24
        front = (x + 0.5 * length * math.cos(phi), y + 0.5 * length * math.sin(phi))
        rear = (x - 0.5 * length * math.cos(phi), y - 0.5 * length * math.sin(phi))
        left = (rear[0] + 0.5 * width * math.cos(phi + math.pi / 2.0), rear[1] + 0.5 * width * math.sin(phi + math.pi / 2.0))
        right = (rear[0] + 0.5 * width * math.cos(phi - math.pi / 2.0), rear[1] + 0.5 * width * math.sin(phi - math.pi / 2.0))
        points = [front, left, right]
        vertices = []
        for point in points:
            screen = self._world_to_screen(*point)
            vertices.extend(screen)
        self._dynamic_add(3, GL_TRIANGLES, vertices, [*color] * 3)
        self._dynamic_add(2, GL_LINES, [*self._world_to_screen(*rear), *self._world_to_screen(*front)], [*color, *color])

    def _draw_polyline(self, states, color):
        if states is None or len(states) < 2:
            return
        segments = []
        for first, second in zip(states[:-1], states[1:]):
            segments.append((float(first[1]), float(first[2]), float(second[1]), float(second[2])))
        vertices = []
        colors = []
        for x1, y1, x2, y2 in segments:
            vertices.extend([*self._world_to_screen(x1, y1), *self._world_to_screen(x2, y2)])
            colors.extend([*color, *color])
        self._dynamic_add(len(vertices) // 2, GL_LINES, vertices, colors)

    def _draw_scan(self, frame):
        """Draw a compact 360-degree LiDAR indicator around the vehicle."""
        scan = frame.get("scan")
        state = frame["true_state"]
        if scan is None:
            return
        del scan
        _, x, y = [float(value) for value in state[:3]]
        # ビーム線は省略し、360度LiDARの使用を示すリングだけ描く。
        radius = 0.45
        segments = []
        points = 72
        for index in range(points):
            first = 2.0 * math.pi * index / points
            second = 2.0 * math.pi * (index + 1) / points
            segments.append(
                (
                    x + radius * math.cos(first),
                    y + radius * math.sin(first),
                    x + radius * math.cos(second),
                    y + radius * math.sin(second),
                )
            )
        vertices = []
        colors = []
        for x1, y1, x2, y2 in segments:
            vertices.extend([*self._world_to_screen(x1, y1), *self._world_to_screen(x2, y2)])
            colors.extend([70, 190, 220, 70, 190, 220])
        self._dynamic_add(len(vertices) // 2, GL_LINES, vertices, colors)

    def _draw_control_direction(self, frame):
        state = frame["true_state"]
        phi, x, y, steer = [float(value) for value in state]
        angle = phi + steer
        endpoint = (x + 0.8 * math.cos(angle), y + 0.8 * math.sin(angle))
        vertices = [*self._world_to_screen(x, y), *self._world_to_screen(*endpoint)]
        self._dynamic_add(2, GL_LINES, vertices, [245, 205, 60, 245, 205, 60])

    def update(self, frame):
        self._clear_dynamic()
        self._draw_scan(frame)
        self._draw_polyline(frame["info"].get("predicted_states"), (220, 80, 220))
        self._draw_vehicle(frame["true_state"], (70, 150, 230))
        estimated = frame.get("estimated_state")
        if estimated is not None:
            self._draw_vehicle(estimated, (245, 150, 50))
            phi, x, y = [float(value) for value in estimated[:3]]
            vertices = [
                *self._world_to_screen(x, y),
                *self._world_to_screen(x + 0.55 * math.cos(phi), y + 0.55 * math.sin(phi)),
            ]
            self._dynamic_add(2, GL_LINES, vertices, [245, 150, 50, 245, 150, 50])
        self._draw_control_direction(frame)
        self.info_label.text = self._format_info(frame)

    def _format_info(self, frame):
        state = frame["true_state"]
        control = frame["control"]
        info = frame["info"]
        actuator = frame.get("actuator")
        command = frame.get("command")
        scan = frame.get("scan")
        estimated = frame.get("estimated_state")
        xy_error = "n/a"
        if estimated is not None:
            xy_error = f"{math.hypot(float(estimated[1] - state[1]), float(estimated[2] - state[2])):.3f} m"
        valid_ratio = "n/a" if scan is None else f"{self._valid_ratio(scan):.1%}"
        age = "n/a" if scan is None else f"{self._scan_age(scan, frame):.3f} s"
        clamp = frame.get("clamp")
        clamp_text = "none" if clamp is None else f"speed={int(clamp.clamped_speed)} steer={int(clamp.clamped_steer)}"
        duty_text = "n/a" if command is None else (
            f"requested={command.requested_esc_duty_percent:.2f}% "
            f"applied={command.esc_duty_percent:.2f}%"
        )
        status = "finished" if frame.get("finished") else "running"
        if frame.get("reached_goal"):
            status += " / goal reached"
        return (
            f"controller: {self.simulation.controller_name}\n"
            f"scenario: {self.simulation.scenario}\n"
            f"status: {status}\n"
            f"time: {self.simulation.time_s:.2f} s\n"
            f"position: ({state[1]:.2f}, {state[2]:.2f}) m\n"
            f"speed: target={control[0]:.3f} m/s\n"
            f"speed: applied={self._applied_speed(control, actuator):.3f} m/s\n"
            f"ESC duty: {duty_text}\n"
            f"ESC limits: stop=10.30% start=10.16% min=10.10%\n"
            f"steer: {state[3]:.3f} rad\n"
            f"steer rate: {control[1]:.3f} rad/s\n"
            f"position error: {xy_error}\n"
            f"LiDAR valid: {valid_ratio}\n"
            f"LiDAR age: {age}\n"
            f"controller time: {1000.0 * float(info.get('solve_time_s', 0.0)):.3f} ms\n"
            f"clamp: {clamp_text}\n\n"
            f"SPACE pause/resume\nR reset\n+/- zoom\nESC/Q quit"
        )

    @staticmethod
    def _applied_speed(control, actuator):
        return float(control[0]) if actuator is None else float(actuator.applied_v)

    @staticmethod
    def _valid_ratio(scan):
        ranges = scan["ranges"]
        return sum(float(value) < float(scan["range_max"]) - 1e-9 for value in ranges) / max(len(ranges), 1)

    def _scan_age(self, scan, frame):
        del frame
        return self.simulation.time_s - float(scan["header"]["stamp_s"])

    def draw(self):
        self.window.clear()
        self.static_batch.draw()
        self.dynamic_batch.draw()
        self.title_label.draw()
        self.info_label.draw()
