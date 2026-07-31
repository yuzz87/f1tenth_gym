"""Simulation-side command conversion with explicit ESC duty safety limits."""

from dataclasses import dataclass

import numpy as np

from ..models.bicycle_model import VehicleLimits


STEER_SLOPE = -0.186785136654
STEER_INTERCEPT = 2.018473280947

DEFAULT_ESC_STOP_DUTY = 10.30
DEFAULT_ESC_START_DUTY = 10.16
DEFAULT_ESC_FORWARD_MIN_DUTY = 10.10


def steer_duty_to_angle(duty_percent, neutral_duty_percent=None):
    duty = float(duty_percent)
    if neutral_duty_percent is not None:
        return STEER_SLOPE * (duty - float(neutral_duty_percent))
    return STEER_SLOPE * duty + STEER_INTERCEPT


def steer_angle_to_duty(angle_rad, neutral_duty_percent=None):
    angle = float(angle_rad)
    if neutral_duty_percent is not None:
        return float(neutral_duty_percent) + angle / STEER_SLOPE
    return (angle - STEER_INTERCEPT) / STEER_SLOPE


def _esc_limits(
    stop_duty_percent=DEFAULT_ESC_STOP_DUTY,
    start_duty_percent=DEFAULT_ESC_START_DUTY,
    forward_min_duty_percent=DEFAULT_ESC_FORWARD_MIN_DUTY,
):
    stop = float(stop_duty_percent)
    start = float(start_duty_percent)
    forward_min = float(forward_min_duty_percent)
    if not forward_min <= start <= stop:
        raise ValueError(
            "ESC duty limits must satisfy forward_min <= start <= stop"
        )
    return stop, start, forward_min


def sanitize_esc_duty(
    duty_percent,
    stop_duty_percent=DEFAULT_ESC_STOP_DUTY,
    start_duty_percent=DEFAULT_ESC_START_DUTY,
    forward_min_duty_percent=DEFAULT_ESC_FORWARD_MIN_DUTY,
    allow_experimental_duty=False,
):
    """Apply stop, start, and forward-side duty limits.

    Lower duty means stronger forward command for this ESC. Values below the
    configured forward limit are available only in simulation experiment mode.
    """
    stop, start, forward_min = _esc_limits(
        stop_duty_percent,
        start_duty_percent,
        forward_min_duty_percent,
    )
    requested = float(duty_percent)
    if requested >= stop:
        applied = stop
    elif requested > start:
        # Keep the uncertain deadband from becoming a marginal start command.
        applied = start
    elif requested < forward_min and not allow_experimental_duty:
        applied = forward_min
    else:
        applied = requested
    return applied, not np.isclose(applied, requested)


def speed_to_esc_duty(
    speed_mps,
    speed_command_max_mps=0.30,
    stop_duty_percent=DEFAULT_ESC_STOP_DUTY,
    start_duty_percent=DEFAULT_ESC_START_DUTY,
    forward_min_duty_percent=DEFAULT_ESC_FORWARD_MIN_DUTY,
):
    """Map virtual model speed to a duty label without encoder calibration.

    This is a simulation command envelope, not a claim about measured ground
    speed. Positive model speed starts at the observed reliable-start duty and
    reaches the configured forward-side limit at the model speed limit.
    """
    stop, start, forward_min = _esc_limits(
        stop_duty_percent,
        start_duty_percent,
        forward_min_duty_percent,
    )
    speed = float(speed_mps)
    speed_max = float(speed_command_max_mps)
    if speed <= 0.0:
        return stop
    if speed_max <= 0.0:
        raise ValueError("speed_command_max_mps must be positive")
    fraction = float(np.clip(speed / speed_max, 0.0, 1.0))
    return start - fraction * (start - forward_min)


def esc_duty_to_speed(
    duty_percent,
    speed_command_max_mps=0.30,
    stop_duty_percent=DEFAULT_ESC_STOP_DUTY,
    start_duty_percent=DEFAULT_ESC_START_DUTY,
    forward_min_duty_percent=DEFAULT_ESC_FORWARD_MIN_DUTY,
    allow_experimental_duty=False,
):
    """Convert the simulation duty envelope back to virtual model speed."""
    applied, _ = sanitize_esc_duty(
        duty_percent,
        stop_duty_percent,
        start_duty_percent,
        forward_min_duty_percent,
        allow_experimental_duty,
    )
    stop, start, forward_min = _esc_limits(
        stop_duty_percent,
        start_duty_percent,
        forward_min_duty_percent,
    )
    if applied >= start:
        return 0.0
    speed_max = float(speed_command_max_mps)
    if speed_max <= 0.0:
        raise ValueError("speed_command_max_mps must be positive")
    fraction = (start - applied) / (start - forward_min)
    return float(np.clip(fraction * speed_max, 0.0, speed_max))


@dataclass(frozen=True)
class CommandResult:
    requested_v: float
    applied_v: float
    requested_psi: float
    applied_psi: float
    steer_duty_percent: float
    requested_esc_duty_percent: float
    esc_duty_percent: float
    clamped_speed: bool
    clamped_steer: bool
    clamped_esc_duty: bool
    experimental_duty: bool


class CommandAdapter:
    """Convert model commands to Gym actions and safe duty diagnostics."""

    def __init__(
        self,
        limits: VehicleLimits,
        timestep_s,
        neutral_steer_duty=10.895,
        esc_config=None,
        allow_experimental_duty=False,
    ):
        self.limits = limits
        self.timestep_s = float(timestep_s)
        self.neutral_steer_duty = float(neutral_steer_duty)
        esc_config = dict(esc_config or {})
        self.esc_stop_duty = float(
            esc_config.get("stop_duty_percent", DEFAULT_ESC_STOP_DUTY)
        )
        self.esc_start_duty = float(
            esc_config.get("start_duty_percent", DEFAULT_ESC_START_DUTY)
        )
        self.esc_forward_min_duty = float(
            esc_config.get(
                "forward_min_duty_percent",
                DEFAULT_ESC_FORWARD_MIN_DUTY,
            )
        )
        self.allow_experimental_duty = bool(allow_experimental_duty)
        _esc_limits(
            self.esc_stop_duty,
            self.esc_start_duty,
            self.esc_forward_min_duty,
        )
        if self.timestep_s <= 0.0:
            raise ValueError("timestep_s must be positive")

    def _speed_to_duty(self, speed_mps):
        return speed_to_esc_duty(
            speed_mps,
            speed_command_max_mps=self.limits.speed_max_mps,
            stop_duty_percent=self.esc_stop_duty,
            start_duty_percent=self.esc_start_duty,
            forward_min_duty_percent=self.esc_forward_min_duty,
        )

    def normalize_esc_duty(self, duty_percent):
        """Return the duty after this adapter's safety policy is applied."""
        return sanitize_esc_duty(
            duty_percent,
            self.esc_stop_duty,
            self.esc_start_duty,
            self.esc_forward_min_duty,
            self.allow_experimental_duty,
        )

    def model_input_to_command(self, state, control):
        """Return a Gym action and requested/applied command diagnostics."""
        state = np.asarray(state, dtype=float)
        if state.shape != (4,):
            raise ValueError("state must have shape (4,)")
        requested_v, requested_omega = map(float, control)
        rate_limited = float(np.clip(
            requested_omega,
            self.limits.steer_rate_min_rad_s,
            self.limits.steer_rate_max_rad_s,
        ))
        requested_psi = float(state[3] + requested_omega * self.timestep_s)
        rate_limited_psi = float(state[3] + rate_limited * self.timestep_s)
        applied_v = float(np.clip(
            requested_v,
            self.limits.speed_min_mps,
            self.limits.speed_max_mps,
        ))
        applied_psi = float(np.clip(
            rate_limited_psi,
            self.limits.steer_min_rad,
            self.limits.steer_max_rad,
        ))
        steer_duty = steer_angle_to_duty(
            applied_psi,
            self.neutral_steer_duty,
        )
        requested_duty = self._speed_to_duty(applied_v)
        esc_duty, clamped_duty = sanitize_esc_duty(
            requested_duty,
            self.esc_stop_duty,
            self.esc_start_duty,
            self.esc_forward_min_duty,
            self.allow_experimental_duty,
        )
        return np.array([applied_psi, applied_v], dtype=float), CommandResult(
            requested_v=requested_v,
            applied_v=applied_v,
            requested_psi=requested_psi,
            applied_psi=applied_psi,
            steer_duty_percent=steer_duty,
            requested_esc_duty_percent=requested_duty,
            esc_duty_percent=esc_duty,
            clamped_speed=not np.isclose(requested_v, applied_v),
            clamped_steer=(
                not np.isclose(requested_omega, rate_limited)
                or not np.isclose(requested_psi, applied_psi)
            ),
            clamped_esc_duty=clamped_duty,
            experimental_duty=(
                self.allow_experimental_duty
                and esc_duty < self.esc_forward_min_duty
            ),
        )

    def duty_to_gym_command(self, steer_duty_percent, esc_duty_percent):
        """Convert a duty command to the simulation Gym action convention."""
        steer = float(np.clip(
            steer_duty_to_angle(
                steer_duty_percent,
                self.neutral_steer_duty,
            ),
            self.limits.steer_min_rad,
            self.limits.steer_max_rad,
        ))
        speed = esc_duty_to_speed(
            esc_duty_percent,
            speed_command_max_mps=self.limits.speed_max_mps,
            stop_duty_percent=self.esc_stop_duty,
            start_duty_percent=self.esc_start_duty,
            forward_min_duty_percent=self.esc_forward_min_duty,
            allow_experimental_duty=self.allow_experimental_duty,
        )
        return np.array([steer, speed], dtype=float)
