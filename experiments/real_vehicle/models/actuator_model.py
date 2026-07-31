"""Simple actuator dynamics for the real-vehicle simulation plant."""

from dataclasses import dataclass, replace

import numpy as np

from .bicycle_model import VehicleLimits, VehicleParameters, step_model


@dataclass(frozen=True)
class ActuatorDiagnostics:
    """Requested, target, and actually applied actuator values."""

    requested_v: float
    target_v: float
    applied_v: float
    requested_omega: float
    target_psi: float
    applied_psi: float
    applied_omega: float
    battery_speed_scale: float
    longitudinal_slip_ratio: float
    cornering_gain: float


class ActuatorModel:
    """First-order ESC and steering-servo approximation.

    The controller still commands ``[v, omega]``. The plant applies speed and
    steering with configurable time constants before advancing the bicycle
    model.
    """

    def __init__(self, parameters: VehicleParameters, limits: VehicleLimits, config=None, seed=7):
        del parameters
        self.limits = limits
        config = dict(config or {})
        self.enabled = bool(config.get("enabled", False))
        self.speed_tau_s = max(float(config.get("speed_time_constant_s", 0.0)), 0.0)
        self.steer_tau_s = max(float(config.get("steer_time_constant_s", 0.0)), 0.0)
        self.speed_noise_std_mps = max(float(config.get("speed_noise_std_mps", 0.0)), 0.0)
        self.steer_noise_std_rad = max(float(config.get("steer_noise_std_rad", 0.0)), 0.0)
        self.speed_deadband_mps = max(float(config.get("speed_deadband_mps", 0.0)), 0.0)
        self.steer_deadband_rad = max(float(config.get("steer_deadband_rad", 0.0)), 0.0)
        self.steer_neutral_offset_rad = float(config.get("steer_neutral_offset_rad", 0.0))
        self.battery_speed_scale = float(np.clip(
            config.get("battery_speed_scale", 1.0), 0.0, 1.5
        ))
        self.longitudinal_slip_ratio = float(np.clip(
            config.get("longitudinal_slip_ratio", 0.0), 0.0, 0.95
        ))
        self.cornering_gain = float(np.clip(
            config.get("cornering_gain", 1.0), 0.05, 1.5
        ))
        self.rng = np.random.default_rng(int(seed))
        self.applied_v = 0.0
        self.applied_psi = 0.0
        self.commanded_psi = 0.0

    def reset(self, state):
        """Reset actuator outputs to the supplied vehicle state."""
        self.applied_v = 0.0
        self.applied_psi = float(np.asarray(state, dtype=float)[3])
        self.commanded_psi = self.applied_psi

    @staticmethod
    def _first_order(current, target, timestep_s, time_constant_s):
        if time_constant_s <= 0.0:
            return float(target)
        alpha = 1.0 - np.exp(-timestep_s / time_constant_s)
        return float(current + alpha * (target - current))

    def step(self, state, control, parameters, timestep_s, integrator="rk4"):
        """Apply actuator dynamics and advance the vehicle model."""
        state = np.asarray(state, dtype=float)
        requested_v, requested_omega = map(float, np.asarray(control, dtype=float))
        target_v = requested_v * self.battery_speed_scale
        if abs(target_v) < self.speed_deadband_mps:
            target_v = 0.0
        target_v = float(np.clip(
            target_v,
            self.limits.speed_min_mps,
            self.limits.speed_max_mps,
        ))
        limited_omega = float(np.clip(
            requested_omega,
            self.limits.steer_rate_min_rad_s,
            self.limits.steer_rate_max_rad_s,
        ))
        self.commanded_psi = float(np.clip(
            self.commanded_psi + limited_omega * timestep_s,
            self.limits.steer_min_rad,
            self.limits.steer_max_rad,
        ))
        target_psi = float(np.clip(
            self.commanded_psi + self.steer_neutral_offset_rad,
            self.limits.steer_min_rad,
            self.limits.steer_max_rad,
        ))
        if abs(target_psi - self.applied_psi) < self.steer_deadband_rad:
            target_psi = self.applied_psi

        if not self.enabled:
            applied_v = target_v
            applied_psi = target_psi
        else:
            applied_v = self._first_order(
                self.applied_v,
                target_v,
                timestep_s,
                self.speed_tau_s,
            )
            applied_psi = self._first_order(
                self.applied_psi,
                target_psi,
                timestep_s,
                self.steer_tau_s,
            )
            if self.speed_noise_std_mps > 0.0:
                applied_v += float(self.rng.normal(0.0, self.speed_noise_std_mps))
            if self.steer_noise_std_rad > 0.0:
                applied_psi += float(self.rng.normal(0.0, self.steer_noise_std_rad))
            applied_v = float(np.clip(
                applied_v,
                self.limits.speed_min_mps,
                self.limits.speed_max_mps,
            ))
            applied_psi = float(np.clip(
                applied_psi,
                self.limits.steer_min_rad,
                self.limits.steer_max_rad,
            ))

        applied_v *= 1.0 - self.longitudinal_slip_ratio
        applied_omega = (applied_psi - float(state[3])) / timestep_s
        applied_omega = float(np.clip(
            applied_omega,
            self.limits.steer_rate_min_rad_s,
            self.limits.steer_rate_max_rad_s,
        ))
        applied_psi = float(state[3] + applied_omega * timestep_s)
        effective_parameters = replace(
            parameters,
            wheelbase_m=parameters.wheelbase_m / self.cornering_gain,
        )
        next_state, clamp = step_model(
            state,
            [applied_v, applied_omega],
            effective_parameters,
            self.limits,
            timestep_s,
            integrator=integrator,
        )
        self.applied_v = float(clamp.applied_v)
        self.applied_psi = float(next_state[3])
        diagnostics = ActuatorDiagnostics(
            requested_v=requested_v,
            target_v=target_v,
            applied_v=self.applied_v,
            requested_omega=requested_omega,
            target_psi=target_psi,
            applied_psi=self.applied_psi,
            applied_omega=float(clamp.applied_omega),
            battery_speed_scale=self.battery_speed_scale,
            longitudinal_slip_ratio=self.longitudinal_slip_ratio,
            cornering_gain=self.cornering_gain,
        )
        return next_state, clamp, diagnostics
