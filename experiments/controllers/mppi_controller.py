import numpy as np

from examples.waypoint_follow import PurePursuitPlanner, nearest_point_on_trajectory

from .base_controller import BaseController


class MPPIController(BaseController):
    """Minimal MPPI controller for centerline tracking."""

    def __init__(self, conf, car_params):
        self.conf = conf
        self.car_params = car_params
        self.controller_conf = conf.controller
        self.wheelbase = car_params["lf"] + car_params["lr"]
        self.speed_target = float(self.controller_conf.get("target_speed", 1.0))
        self.horizon = int(self.controller_conf.get("horizon", 12))
        self.num_samples = int(self.controller_conf.get("num_samples", 128))
        self.temperature = float(self.controller_conf.get("temperature", 0.5))
        self.noise_sigma = float(self.controller_conf.get("noise_sigma", 0.08))
        self.position_weight = float(self.controller_conf.get("position_weight", 4.0))
        self.heading_weight = float(self.controller_conf.get("heading_weight", 0.8))
        self.terminal_position_weight = float(self.controller_conf.get("terminal_position_weight", 8.0))
        self.terminal_heading_weight = float(self.controller_conf.get("terminal_heading_weight", 1.5))
        self.control_weight = float(self.controller_conf.get("control_weight", 0.02))
        self.delta_weight = float(self.controller_conf.get("delta_weight", 0.8))
        self.nominal_weight = float(self.controller_conf.get("nominal_weight", 1.2))
        self.steer_min = float(car_params["s_min"])
        self.steer_max = float(car_params["s_max"])
        self.timestep = float(conf.timestep)
        self.prev_sequence = np.zeros((self.horizon,), dtype=float)
        self.rng = np.random.default_rng(int(self.controller_conf.get("seed", 7)))

        self.render_helper = PurePursuitPlanner(conf, self.wheelbase)
        self.nominal_planner = PurePursuitPlanner(conf, self.wheelbase)
        self.waypoints = self.render_helper.waypoints
        self.path_xy = np.column_stack(
            (
                self.waypoints[:, conf.wpt_xind],
                self.waypoints[:, conf.wpt_yind],
            )
        )
        self.closed_path_xy = np.vstack((self.path_xy, self.path_xy[0]))
        diffs = self.closed_path_xy[1:] - self.closed_path_xy[:-1]
        self.segment_lengths = np.linalg.norm(diffs, axis=1)
        self.cumulative_lengths = np.concatenate(([0.0], np.cumsum(self.segment_lengths)))
        self.track_length = float(self.cumulative_lengths[-1])
        self.segment_headings = np.arctan2(diffs[:, 1], diffs[:, 0])
        self.interp_s = np.append(self.cumulative_lengths[:-1], self.track_length)
        self.interp_x = np.append(self.path_xy[:, 0], self.path_xy[0, 0])
        self.interp_y = np.append(self.path_xy[:, 1], self.path_xy[0, 1])
        self.interp_cos = np.append(np.cos(self.segment_headings), np.cos(self.segment_headings[0]))
        self.interp_sin = np.append(np.sin(self.segment_headings), np.sin(self.segment_headings[0]))

    def plan(self, obs):
        state = np.array(
            [
                float(obs["poses_x"][0]),
                float(obs["poses_y"][0]),
                float(obs["poses_theta"][0]),
            ]
        )
        current_speed = float(obs["linear_vels_x"][0])
        nominal_speed, nominal_steer = self.nominal_planner.plan(
            state[0],
            state[1],
            state[2],
            self.conf.controller.get("tlad", 0.8),
            self.conf.controller.get("vgain", 1.0),
        )
        speed_target = max(self.speed_target, current_speed, float(nominal_speed))
        progress = self._compute_progress(state[:2])
        reference = self._build_reference(progress, speed_target)

        nominal_sequence = np.full((self.horizon,), float(nominal_steer), dtype=float)
        base_sequence = np.clip(
            0.7 * self.prev_sequence + 0.3 * nominal_sequence,
            self.steer_min,
            self.steer_max,
        )

        noises = self.rng.normal(0.0, self.noise_sigma, size=(self.num_samples, self.horizon))
        candidate_sequences = np.clip(base_sequence[None, :] + noises, self.steer_min, self.steer_max)
        costs = np.empty((self.num_samples,), dtype=float)
        for sample_index in range(self.num_samples):
            costs[sample_index] = self._trajectory_cost(
                state,
                speed_target,
                candidate_sequences[sample_index],
                reference,
                nominal_sequence,
            )

        min_cost = float(np.min(costs))
        weights = np.exp(-(costs - min_cost) / max(self.temperature, 1e-6))
        weights_sum = float(np.sum(weights))
        if not np.isfinite(weights_sum) or weights_sum <= 1e-12:
            refined_sequence = base_sequence
        else:
            refined_sequence = np.sum(candidate_sequences * weights[:, None], axis=0) / weights_sum
            refined_sequence = np.clip(refined_sequence, self.steer_min, self.steer_max)

        self.prev_sequence[:-1] = refined_sequence[1:]
        self.prev_sequence[-1] = refined_sequence[-1]
        steer = float(np.clip(refined_sequence[0], self.steer_min, self.steer_max))
        return speed_target, steer

    def render_waypoints(self, env_renderer):
        self.render_helper.render_waypoints(env_renderer)

    def _compute_progress(self, position):
        nearest_point, _, t, segment_index = nearest_point_on_trajectory(position, self.closed_path_xy)
        del nearest_point
        return float(self.cumulative_lengths[segment_index] + t * self.segment_lengths[segment_index])

    def _build_reference(self, progress, speed_target):
        step_ds = max(speed_target * self.timestep, 1e-4)
        samples = progress + step_ds * np.arange(1, self.horizon + 1, dtype=float)
        wrapped = np.mod(samples, self.track_length)
        ref_x = np.interp(wrapped, self.interp_s, self.interp_x)
        ref_y = np.interp(wrapped, self.interp_s, self.interp_y)
        ref_heading = np.arctan2(
            np.interp(wrapped, self.interp_s, self.interp_sin),
            np.interp(wrapped, self.interp_s, self.interp_cos),
        )
        return np.column_stack((ref_x, ref_y, ref_heading))

    def _rollout(self, state, speed_target, steer_sequence):
        x, y, theta = state
        predicted = np.empty((self.horizon, 3), dtype=float)
        for index, steer in enumerate(steer_sequence):
            x += speed_target * np.cos(theta) * self.timestep
            y += speed_target * np.sin(theta) * self.timestep
            theta += speed_target * np.tan(steer) / self.wheelbase * self.timestep
            theta = self._normalize_angle(theta)
            predicted[index] = (x, y, theta)
        return predicted

    def _trajectory_cost(self, state, speed_target, steer_sequence, reference, nominal_sequence):
        predicted = self._rollout(state, speed_target, steer_sequence)
        cost = 0.0
        prev_steer = nominal_sequence[0]
        for index in range(self.horizon):
            dx = predicted[index, 0] - reference[index, 0]
            dy = predicted[index, 1] - reference[index, 1]
            heading_error = self._normalize_angle(predicted[index, 2] - reference[index, 2])
            position_weight = (
                self.terminal_position_weight if index == self.horizon - 1 else self.position_weight
            )
            heading_weight = (
                self.terminal_heading_weight if index == self.horizon - 1 else self.heading_weight
            )
            steer = steer_sequence[index]
            delta = steer - prev_steer
            cost += position_weight * (dx * dx + dy * dy)
            cost += heading_weight * heading_error * heading_error
            cost += self.control_weight * steer * steer
            cost += self.delta_weight * delta * delta
            cost += self.nominal_weight * (steer - nominal_sequence[index]) ** 2
            prev_steer = steer
        return float(cost)

    @staticmethod
    def _normalize_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi
