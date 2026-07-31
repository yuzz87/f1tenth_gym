"""Shared rollout and cost functions for the actual-vehicle controllers."""

import numpy as np

from ..models.bicycle_model import normalize_angle


class ControllerBase:
    def __init__(self, parameters, limits, timestep_s, config=None):
        self.parameters = parameters
        self.limits = limits
        self.timestep_s = float(timestep_s)
        self.config = dict(config or {})
        self.horizon = int(self.config.get("horizon", 12))
        self.position_weight = float(self.config.get("position_weight", 20.0))
        self.heading_weight = float(self.config.get("heading_weight", 4.0))
        self.steer_weight = float(self.config.get("steer_weight", 0.4))
        self.speed_weight = float(self.config.get("speed_weight", 0.8))
        self.input_rate_weight = float(self.config.get("input_rate_weight", 0.15))
        self.terminal_weight = float(self.config.get("terminal_weight", 30.0))
        self.speed_target = float(self.config.get("speed_target_mps", limits.speed_max_mps))
        self.prev_sequence = np.zeros((self.horizon, 2), dtype=float)
        self.prev_sequence[:, 0] = np.clip(
            self.speed_target, limits.speed_min_mps, limits.speed_max_mps
        )

    def reset(self):
        self.prev_sequence.fill(0.0)
        self.prev_sequence[:, 0] = np.clip(
            self.speed_target, self.limits.speed_min_mps, self.limits.speed_max_mps
        )

    def _prepare_reference(self, reference):
        values = np.asarray(reference, dtype=float)
        if values.ndim != 2 or values.shape[1] < 4:
            raise ValueError("reference must have shape (N, 4+) as [phi,x,y,psi,...]")
        if values.shape[0] < self.horizon:
            padding = np.repeat(
                values[-1:, :],
                self.horizon - values.shape[0],
                axis=0,
            )
            values = np.vstack((values, padding))
        return values[: self.horizon]

    def _reference_controls(self, reference):
        if reference.shape[1] >= 6:
            desired_v = reference[:, 4]
            desired_omega = reference[:, 5]
        else:
            desired_v = np.full(self.horizon, self.speed_target, dtype=float)
            desired_omega = np.zeros(self.horizon, dtype=float)
        return np.column_stack((
            np.clip(desired_v, self.limits.speed_min_mps, self.limits.speed_max_mps),
            np.clip(
                desired_omega,
                self.limits.steer_rate_min_rad_s,
                self.limits.steer_rate_max_rad_s,
            ),
        ))

    def rollout(self, state, controls):
        state = np.asarray(state, dtype=float).copy()
        controls = np.asarray(controls, dtype=float)
        states = np.empty((controls.shape[0], 4), dtype=float)
        wheelbase = float(self.parameters.wheelbase_m)

        def model_derivative(current, control):
            velocity, steer_rate = control
            phi = current[0]
            psi = current[3]
            return np.array(
                [
                    velocity / wheelbase * np.tan(psi),
                    velocity * np.cos(phi),
                    velocity * np.sin(phi),
                    steer_rate,
                ],
                dtype=float,
            )

        for index, control in enumerate(controls):
            control = np.array([
                np.clip(control[0], self.limits.speed_min_mps, self.limits.speed_max_mps),
                np.clip(
                    control[1],
                    self.limits.steer_rate_min_rad_s,
                    self.limits.steer_rate_max_rad_s,
                ),
            ])
            k1 = model_derivative(state, control)
            k2 = model_derivative(state + 0.5 * self.timestep_s * k1, control)
            k3 = model_derivative(state + 0.5 * self.timestep_s * k2, control)
            k4 = model_derivative(state + self.timestep_s * k3, control)
            state = state + self.timestep_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
            state[0] = float(normalize_angle(state[0]))
            state[3] = float(np.clip(
                state[3],
                self.limits.steer_min_rad,
                self.limits.steer_max_rad,
            ))
            states[index] = state
        return states

    def cost(self, state, controls, reference):
        predicted = self.rollout(state, controls)
        return self.cost_from_prediction(predicted, controls, reference)

    def cost_from_prediction(self, predicted, controls, reference):
        predicted = np.asarray(predicted, dtype=float)
        controls = np.asarray(controls, dtype=float)
        reference = self._prepare_reference(reference)
        desired_controls = self._reference_controls(reference)
        total = 0.0
        # 入力の順番は [v, omega]。状態の psi を前ステップ入力に混ぜない。
        previous = np.array([desired_controls[0, 0], 0.0], dtype=float)
        for index, predicted_state in enumerate(predicted):
            position_error = predicted_state[1:3] - reference[index, 1:3]
            heading_error = float(normalize_angle(predicted_state[0] - reference[index, 0]))
            steer_error = float(normalize_angle(predicted_state[3] - reference[index, 3]))
            control = controls[index]
            input_delta = control - previous
            stage = (
                self.position_weight * float(np.dot(position_error, position_error))
                + self.heading_weight * heading_error**2
                + self.steer_weight * steer_error**2
                + self.speed_weight * (control[0] - desired_controls[index, 0]) ** 2
                + self.input_rate_weight * float(np.dot(input_delta, input_delta))
            )
            if index == len(predicted) - 1:
                stage *= self.terminal_weight
            total += stage
            previous = control
        return float(total)

    def _shift(self, sequence):
        sequence = np.asarray(sequence, dtype=float)
        self.prev_sequence[:-1] = sequence[1:]
        self.prev_sequence[-1] = sequence[-1]
