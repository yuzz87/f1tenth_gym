"""Conversions between ROS 2 messages and ROS-independent contracts."""

import math

from .contracts import ControlCommand, PoseEstimate

try:
    from ackermann_msgs.msg import AckermannDriveStamped
except ImportError:
    AckermannDriveStamped = None


def require_ackermann_messages():
    if AckermannDriveStamped is None:
        raise RuntimeError(
            "ackermann_msgs is required; install ros-foxy-ackermann-msgs "
            "after sourcing ROS 2 Foxy"
        )


def stamp_to_seconds(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        float(quaternion.w) * float(quaternion.z)
        + float(quaternion.x) * float(quaternion.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        float(quaternion.y) ** 2 + float(quaternion.z) ** 2
    )
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw_rad):
    half = 0.5 * float(yaw_rad)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def pose_message_to_contract(message, steering_angle_rad=0.0):
    pose = message.pose.pose
    return PoseEstimate(
        stamp_s=stamp_to_seconds(message.header.stamp),
        phi_rad=quaternion_to_yaw(pose.orientation),
        x_m=float(pose.position.x),
        y_m=float(pose.position.y),
        steering_angle_rad=float(steering_angle_rad),
        frame_id=str(message.header.frame_id),
    )


def ackermann_message_to_contract(message, sequence_id):
    require_ackermann_messages()
    return ControlCommand(
        stamp_s=stamp_to_seconds(message.header.stamp),
        sequence_id=int(sequence_id),
        speed_mps=float(message.drive.speed),
        steering_angle_rad=float(message.drive.steering_angle),
        steering_rate_rad_s=float(message.drive.steering_angle_velocity),
        frame_id=str(message.header.frame_id),
    )


def contract_to_ackermann_message(command, stamp=None):
    require_ackermann_messages()
    message = AckermannDriveStamped()
    if stamp is not None:
        message.header.stamp = stamp
    message.header.frame_id = command.frame_id
    message.drive.speed = float(command.speed_mps)
    message.drive.steering_angle = float(command.steering_angle_rad)
    message.drive.steering_angle_velocity = float(command.steering_rate_rad_s)
    return message
