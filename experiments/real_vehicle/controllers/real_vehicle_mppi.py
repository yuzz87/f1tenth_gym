"""Two-input MPPI for the actual RC-car bicycle model."""

import time

import numpy as np

from .common import ControllerBase


class RealVehicleMPPI(ControllerBase):
    """Sample speed and steering-rate sequences and apply a weighted update."""

    def __init__(self, parameters, limits, timestep_s, config=None):
        super().__init__(parameters, limits, timestep_s, config)
        self.num_samples = int(self.config.get("num_samples", 256))
        self.temperature = max(float(self.config.get("temperature", 0.5)), 1e-6)
        self.speed_noise = float(self.config.get("speed_noise_mps", 0.04))
        self.steer_rate_noise = float(self.config.get("steer_rate_noise_rad_s", 0.20))
        self.use_noise_cost = bool(self.config.get("use_noise_cost", True))
        self.noise_cost_weight = float(self.config.get("noise_cost_weight", 0.01))
        self.integrator = str(self.config.get("mppi_integrator", "rk4")).lower()
        self.rng = np.random.default_rng(int(self.config.get("seed", 7)))

    def plan(self, state, reference):
        started = time.perf_counter()
        reference = self._prepare_reference(reference)
        desired = self._reference_controls(reference)
        base = 0.7 * self.prev_sequence + 0.3 * desired
        noise = np.empty((self.num_samples, self.horizon, 2), dtype=float)
        shape = (self.num_samples, self.horizon)
        noise[:, :, 0] = self.rng.normal(0.0, self.speed_noise, shape)
        noise[:, :, 1] = self.rng.normal(0.0, self.steer_rate_noise, shape)
        candidates = self._reflect_to_bounds(
            base[None, :, :] + noise,
            np.array([self.limits.speed_min_mps, self.limits.steer_rate_min_rad_s]),
            np.array([self.limits.speed_max_mps, self.limits.steer_rate_max_rad_s]),
        )
        noise = candidates - base[None, :, :]
        costs = self._batch_cost(state, candidates, reference)
        if self.use_noise_cost:
            variance = np.maximum(
                np.array([self.speed_noise**2, self.steer_rate_noise**2], dtype=float),
                1e-9,
            )
            # MPPIの入力ノイズ確率補正項。入力単位ごとに分散で正規化する。
            noise_correction = self.temperature * np.sum(
                base[None, :, :] * noise / variance[None, None, :],
                axis=(1, 2),
            )
            costs = costs + self.noise_cost_weight * noise_correction
        minimum = float(np.min(costs))
        weights = np.exp(-(costs - minimum) / self.temperature)
        if not np.isfinite(weights).all() or float(np.sum(weights)) <= 1e-12:
            sequence = base
        else:
            sequence = np.sum(
                candidates * weights[:, None, None], axis=0
            ) / float(np.sum(weights))
        sequence = np.clip(
            sequence,
            [self.limits.speed_min_mps, self.limits.steer_rate_min_rad_s],
            [self.limits.speed_max_mps, self.limits.steer_rate_max_rad_s],
        )
        predicted = self.rollout(state, sequence)
        self._shift(sequence)
        return sequence[0].copy(), {
            "controller": "mppi",
            "cost": float(self.cost_from_prediction(predicted, sequence, reference)),
            "solve_time_s": time.perf_counter() - started,
            "success": True,
            "predicted_states": predicted,
            "best_sample_cost": minimum,
        }

    def _batch_rollout(self, state, controls):
        """Vectorized rollout for all MPPI samples."""
        state = np.asarray(state, dtype=float)
        batch = np.repeat(state[None, :], controls.shape[0], axis=0)
        predicted = np.empty((controls.shape[0], self.horizon, 4), dtype=float)
        wheelbase = float(self.parameters.wheelbase_m)
        for index in range(self.horizon):
            velocity = controls[:, index, 0]
            steer_rate = controls[:, index, 1]
            if self.integrator == "euler":
                derivative = self._batch_derivative(
                    batch, velocity, steer_rate, wheelbase
                )
                batch += self.timestep_s * derivative
            elif self.integrator == "rk4":
                k1 = self._batch_derivative(batch, velocity, steer_rate, wheelbase)
                k2 = self._batch_derivative(
                    batch + 0.5 * self.timestep_s * k1,
                    velocity,
                    steer_rate,
                    wheelbase,
                )
                k3 = self._batch_derivative(
                    batch + 0.5 * self.timestep_s * k2,
                    velocity,
                    steer_rate,
                    wheelbase,
                )
                k4 = self._batch_derivative(
                    batch + self.timestep_s * k3,
                    velocity,
                    steer_rate,
                    wheelbase,
                )
                batch += (
                    self.timestep_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
                )
            else:
                raise ValueError("mppi_integrator must be 'euler' or 'rk4'")
            batch[:, 3] = np.clip(
                batch[:, 3],
                self.limits.steer_min_rad,
                self.limits.steer_max_rad,
            )
            batch[:, 0] = (batch[:, 0] + np.pi) % (2.0 * np.pi) - np.pi
            predicted[:, index] = batch
        return predicted

    @staticmethod
    def _batch_derivative(batch, velocity, steer_rate, wheelbase):
        phi = batch[:, 0]
        psi = batch[:, 3]
        return np.column_stack((
            velocity / wheelbase * np.tan(psi),
            velocity * np.cos(phi),
            velocity * np.sin(phi),
            steer_rate,
        ))

    @staticmethod
    def _reflect_to_bounds(values, lower, upper):
        """Reflect samples at bounds to avoid clipping bias near saturation."""
        values = np.asarray(values, dtype=float)
        result = values.copy()
        for index, (minimum, maximum) in enumerate(zip(lower, upper)):
            span = float(maximum - minimum)
            if span <= 0.0:
                result[..., index] = minimum
                continue
            period = 2.0 * span
            wrapped = np.mod(result[..., index] - minimum, period)
            result[..., index] = np.where(
                wrapped <= span,
                minimum + wrapped,
                maximum - (wrapped - span),
            )
        return result

    def _batch_cost(self, state, controls, reference):
        reference = self._prepare_reference(reference)
        desired = self._reference_controls(reference)
        predicted = self._batch_rollout(state, controls)
        position_error = predicted[:, :, 1:3] - reference[None, :, 1:3]
        heading_error = (predicted[:, :, 0] - reference[None, :, 0] + np.pi) % (2.0 * np.pi) - np.pi
        steer_error = (predicted[:, :, 3] - reference[None, :, 3] + np.pi) % (2.0 * np.pi) - np.pi
        previous_control = np.empty_like(controls)
        # バッチ側も [v, omega] の順序で前ステップ入力を作る。
        previous_control[:, 0] = np.array([desired[0, 0], 0.0])
        previous_control[:, 1:] = controls[:, :-1]
        input_delta = controls - previous_control
        stage = (
            self.position_weight * np.sum(position_error * position_error, axis=2)
            + self.heading_weight * heading_error * heading_error
            + self.steer_weight * steer_error * steer_error
            + self.speed_weight * (controls[:, :, 0] - desired[None, :, 0]) ** 2
            + self.input_rate_weight * np.sum(input_delta * input_delta, axis=2)
        )
        stage[:, -1] *= self.terminal_weight
        return np.sum(stage, axis=1)
