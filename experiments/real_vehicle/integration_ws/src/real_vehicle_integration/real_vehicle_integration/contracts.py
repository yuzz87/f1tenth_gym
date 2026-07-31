"""Typed, ROS-independent contracts shared by integration nodes."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseEstimate:
    stamp_s: float
    phi_rad: float
    x_m: float
    y_m: float
    steering_angle_rad: float = 0.0
    frame_id: str = "map"


@dataclass(frozen=True)
class ControlCommand:
    stamp_s: float
    sequence_id: int
    speed_mps: float
    steering_angle_rad: float
    steering_rate_rad_s: float
    frame_id: str = "base_link"


@dataclass(frozen=True)
class ActuatorOutput:
    stamp_s: float
    sequence_id: int
    esc_duty_percent: float
    steering_duty_percent: float
    enabled: bool
    reason: str = ""


@dataclass(frozen=True)
class ContractLimits:
    speed_min_mps: float = 0.0
    speed_max_mps: float = 0.30
    steer_min_rad: float = -0.3141592653589793
    steer_max_rad: float = 0.3141592653589793
    steer_rate_min_rad_s: float = -0.70
    steer_rate_max_rad_s: float = 0.70
    future_tolerance_s: float = 0.05


def _require_finite(name, *values):
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError(f"{name} contains NaN or Inf")


def _validate_age(stamp_s, now_s, max_age_s, future_tolerance_s):
    age_s = float(now_s) - float(stamp_s)
    if age_s < -float(future_tolerance_s):
        raise ValueError("timestamp is too far in the future")
    if age_s > float(max_age_s):
        raise ValueError("message is stale")


def validate_pose(pose, now_s, max_age_s, limits=ContractLimits()):
    if not isinstance(pose, PoseEstimate):
        raise TypeError("pose must be PoseEstimate")
    _require_finite(
        "pose",
        pose.stamp_s,
        pose.phi_rad,
        pose.x_m,
        pose.y_m,
        pose.steering_angle_rad,
    )
    if not pose.frame_id:
        raise ValueError("pose frame_id must not be empty")
    if not limits.steer_min_rad <= pose.steering_angle_rad <= limits.steer_max_rad:
        raise ValueError("pose steering angle is outside limits")
    _validate_age(pose.stamp_s, now_s, max_age_s, limits.future_tolerance_s)
    return pose


def validate_control(command, now_s, max_age_s, limits=ContractLimits()):
    if not isinstance(command, ControlCommand):
        raise TypeError("command must be ControlCommand")
    _require_finite(
        "control",
        command.stamp_s,
        command.speed_mps,
        command.steering_angle_rad,
        command.steering_rate_rad_s,
    )
    if command.sequence_id < 0:
        raise ValueError("sequence_id must be non-negative")
    if not command.frame_id:
        raise ValueError("control frame_id must not be empty")
    tolerance = 1e-6
    if not (
        limits.speed_min_mps - tolerance
        <= command.speed_mps
        <= limits.speed_max_mps + tolerance
    ):
        raise ValueError("control speed is outside limits")
    if not (
        limits.steer_min_rad - tolerance
        <= command.steering_angle_rad
        <= limits.steer_max_rad + tolerance
    ):
        raise ValueError("control steering angle is outside limits")
    if not (
        limits.steer_rate_min_rad_s - tolerance
        <= command.steering_rate_rad_s
        <= limits.steer_rate_max_rad_s + tolerance
    ):
        raise ValueError("control steering rate is outside limits")
    _validate_age(command.stamp_s, now_s, max_age_s, limits.future_tolerance_s)
    return ControlCommand(
        stamp_s=command.stamp_s,
        sequence_id=command.sequence_id,
        speed_mps=min(max(command.speed_mps, limits.speed_min_mps), limits.speed_max_mps),
        steering_angle_rad=min(
            max(command.steering_angle_rad, limits.steer_min_rad),
            limits.steer_max_rad,
        ),
        steering_rate_rad_s=min(
            max(command.steering_rate_rad_s, limits.steer_rate_min_rad_s),
            limits.steer_rate_max_rad_s,
        ),
        frame_id=command.frame_id,
    )


def neutral_control(stamp_s, sequence_id=0, frame_id="base_link"):
    return ControlCommand(
        stamp_s=float(stamp_s),
        sequence_id=int(sequence_id),
        speed_mps=0.0,
        steering_angle_rad=0.0,
        steering_rate_rad_s=0.0,
        frame_id=str(frame_id),
    )
