"""Helpers for reading common ROS parameters into core configuration."""

from .contracts import ContractLimits
from .safety_monitor import SafetyConfig


COMMON_PARAMETERS = {
    "scan_topic": "/scan",
    "odom_topic": "/odom",
    "pose_topic": "/localization/pose",
    "control_request_topic": "/vehicle/control_request",
    "control_safe_topic": "/vehicle/control_safe",
    "actuator_status_topic": "/vehicle/actuator_status",
    "controller_status_topic": "/vehicle/controller_status",
    "localization_status_topic": "/localization/status",
    "lidar_transport_status_topic": "/lidar/transport_status",
    "safety_status_topic": "/vehicle/safety_status",
    "emergency_stop_topic": "/vehicle/emergency_stop",
    "map_frame": "map",
    "odom_frame": "odom",
    "base_frame": "base_link",
    "laser_frame": "laser",
    "control_rate_hz": 100.0,
    "mpc_control_rate_hz": 50.0,
    "safety_rate_hz": 50.0,
    "actuator_rate_hz": 50.0,
    "command_timeout_s": 0.10,
    "pose_timeout_s": 0.20,
    "scan_timeout_s": 0.30,
    "future_tolerance_s": 0.05,
    "speed_min_mps": 0.0,
    "speed_max_mps": 0.30,
    "steer_min_rad": -0.3141592653589793,
    "steer_max_rad": 0.3141592653589793,
    "steer_rate_min_rad_s": -0.70,
    "steer_rate_max_rad_s": 0.70,
}


def declare_common_parameters(node):
    for name, default in COMMON_PARAMETERS.items():
        if not node.has_parameter(name):
            node.declare_parameter(name, default)


def parameter_value(node, name):
    return node.get_parameter(name).value


def contract_limits_from_node(node):
    return ContractLimits(
        speed_min_mps=float(parameter_value(node, "speed_min_mps")),
        speed_max_mps=float(parameter_value(node, "speed_max_mps")),
        steer_min_rad=float(parameter_value(node, "steer_min_rad")),
        steer_max_rad=float(parameter_value(node, "steer_max_rad")),
        steer_rate_min_rad_s=float(parameter_value(node, "steer_rate_min_rad_s")),
        steer_rate_max_rad_s=float(parameter_value(node, "steer_rate_max_rad_s")),
        future_tolerance_s=float(parameter_value(node, "future_tolerance_s")),
    )


def safety_config_from_node(node):
    return SafetyConfig(
        command_timeout_s=float(parameter_value(node, "command_timeout_s")),
        pose_timeout_s=float(parameter_value(node, "pose_timeout_s")),
        scan_timeout_s=float(parameter_value(node, "scan_timeout_s")),
        limits=contract_limits_from_node(node),
    )
