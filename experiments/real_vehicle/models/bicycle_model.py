"""Four-state bicycle model for the actual RC car simulation."""

from dataclasses import dataclass
from typing import Literal

import numpy as np

STATE_SIZE = 4
INPUT_SIZE = 2


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi)."""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class VehicleParameters:
    wheelbase_m: float = 0.25
    wheel_diameter_m: float = 0.066
    encoder_teeth: int = 36


@dataclass(frozen=True)
class VehicleLimits:
    speed_min_mps: float = 0.0
    speed_max_mps: float = 0.30
    steer_min_rad: float = -0.3141592653589793
    steer_max_rad: float = 0.3141592653589793
    steer_rate_min_rad_s: float = -0.70
    steer_rate_max_rad_s: float = 0.70


@dataclass(frozen=True)
class InputClampResult:
    requested_v: float
    applied_v: float
    requested_omega: float
    applied_omega: float
    next_steer_rad: float
    clamped_speed: bool
    clamped_steer: bool


def _as_state(state):
    values = np.asarray(state, dtype=float)
    if values.shape != (STATE_SIZE,):
        raise ValueError("state must have shape (4,) as [phi, x, y, psi]")
    return values


def _as_input(control):
    values = np.asarray(control, dtype=float)
    if values.shape != (INPUT_SIZE,):
        raise ValueError("input must have shape (2,) as [v, omega]")
    return values


def clamp_input(state, control, limits: VehicleLimits, timestep_s: float):
    """Apply speed, steering-rate, and next-angle constraints with diagnostics."""
    state = _as_state(state)
    control = _as_input(control)
    if timestep_s <= 0.0:
        raise ValueError("timestep_s must be positive")

    requested_v, requested_omega = map(float, control)
    applied_v = float(np.clip(requested_v, limits.speed_min_mps, limits.speed_max_mps))
    rate_limited = float(
        np.clip(
            requested_omega,
            limits.steer_rate_min_rad_s,
            limits.steer_rate_max_rad_s,
        )
    )
    requested_next_steer = float(state[3] + rate_limited * timestep_s)
    next_steer = float(
        np.clip(requested_next_steer, limits.steer_min_rad, limits.steer_max_rad)
    )
    applied_omega = (next_steer - float(state[3])) / timestep_s
    return InputClampResult(
        requested_v=requested_v,
        applied_v=applied_v,
        requested_omega=requested_omega,
        applied_omega=applied_omega,
        next_steer_rad=next_steer,
        clamped_speed=not np.isclose(applied_v, requested_v),
        clamped_steer=(
            not np.isclose(applied_omega, requested_omega)
            or not np.isclose(next_steer, requested_next_steer)
        ),
    )


def derivatives(state, control, parameters: VehicleParameters):
    """Return [phi_dot, x_dot, y_dot, psi_dot]."""
    state = _as_state(state)
    control = _as_input(control)
    phi, _, _, psi = state
    velocity, steer_rate = control
    wheelbase = float(parameters.wheelbase_m)
    if wheelbase <= 0.0:
        raise ValueError("wheelbase_m must be positive")
    return np.array(
        [
            velocity / wheelbase * np.tan(psi),
            velocity * np.cos(phi),
            velocity * np.sin(phi),
            steer_rate,
        ],
        dtype=float,
    )


def step_model(
    state,
    control,
    parameters: VehicleParameters,
    limits: VehicleLimits,
    timestep_s: float,
    integrator: Literal["euler", "rk4"] = "rk4",
):
    """Advance one constrained model step and return state plus clamp metadata."""
    state = _as_state(state)
    clamped = clamp_input(state, control, limits, timestep_s)
    applied_control = np.array([clamped.applied_v, clamped.applied_omega], dtype=float)

    if integrator.lower() == "euler":
        next_state = state + timestep_s * derivatives(state, applied_control, parameters)
    elif integrator.lower() == "rk4":
        k1 = derivatives(state, applied_control, parameters)
        k2 = derivatives(state + 0.5 * timestep_s * k1, applied_control, parameters)
        k3 = derivatives(state + 0.5 * timestep_s * k2, applied_control, parameters)
        k4 = derivatives(state + timestep_s * k3, applied_control, parameters)
        next_state = state + timestep_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    else:
        raise ValueError("integrator must be 'euler' or 'rk4'")

    next_state[0] = float(normalize_angle(next_state[0]))
    next_state[3] = float(
        np.clip(next_state[3], limits.steer_min_rad, limits.steer_max_rad)
    )
    return next_state, clamped
