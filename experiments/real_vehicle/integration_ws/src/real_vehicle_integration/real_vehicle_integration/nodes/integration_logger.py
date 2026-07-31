"""Write synchronized integration observations whenever a safe command arrives."""

import csv
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from ..configuration import declare_common_parameters, parameter_value
from ..repository import enable_repository_imports
from ..ros_support import (
    AckermannDriveStamped,
    quaternion_to_yaw,
    require_ackermann_messages,
    stamp_to_seconds,
)


class IntegrationLoggerNode(Node):
    FIELDS = (
        "receive_time_s",
        "command_stamp_s",
        "pose_stamp_s",
        "pose_age_s",
        "phi_rad",
        "x_m",
        "y_m",
        "speed_mps",
        "steering_angle_rad",
        "steering_rate_rad_s",
        "actuator_state",
        "esc_duty_percent",
        "steering_duty_percent",
        "hardware_output_enabled",
        "controller_state",
        "controller_solve_time_ms",
        "controller_overrun",
        "controller_deadline_exceeded",
        "localization_state",
        "localization_match_score",
        "localization_valid_beams",
        "lidar_transport_state",
        "lidar_scan_rate_hz",
        "lidar_raw_points",
        "lidar_valid_beams",
        "lidar_invalid_packets",
        "lidar_incomplete_scans",
        "lidar_sequence_gaps",
        "lidar_source_clock_offset_s",
        "safety_state",
        "safety_fault_reason",
        "safety_scan_age_s",
        "safety_pose_age_s",
        "safety_control_age_s",
    )

    def __init__(self):
        require_ackermann_messages()
        super().__init__("real_vehicle_integration_logger")
        declare_common_parameters(self)
        root = enable_repository_imports()
        self.declare_parameter(
            "log_path",
            str(
                root
                / "experiments/real_vehicle/results/ros2_integration/integration.csv"
            ),
        )
        self.latest_pose = None
        self.actuator_values = {}
        self.controller_values = {}
        self.localization_values = {}
        self.lidar_values = {}
        self.safety_values = {}
        self.path = Path(str(parameter_value(self, "log_path"))).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(parameter_value(self, "pose_topic")),
            self._on_pose,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            str(parameter_value(self, "actuator_status_topic")),
            self._on_status,
            10,
        )
        for topic_name, callback in (
            ("controller_status_topic", self._on_controller_status),
            ("localization_status_topic", self._on_localization_status),
            ("lidar_transport_status_topic", self._on_lidar_status),
            ("safety_status_topic", self._on_safety_status),
        ):
            self.create_subscription(
                DiagnosticArray,
                str(parameter_value(self, topic_name)),
                callback,
                10,
            )
        self.create_subscription(
            AckermannDriveStamped,
            str(parameter_value(self, "control_safe_topic")),
            self._on_command,
            10,
        )

    def _on_pose(self, message):
        self.latest_pose = message

    def _on_status(self, message):
        if not message.status:
            return
        status = message.status[0]
        self.actuator_values = {value.key: value.value for value in status.values}
        self.actuator_values["state"] = status.message

    @staticmethod
    def _diagnostic_values(message):
        if not message.status:
            return {}
        status = message.status[0]
        values = {value.key: value.value for value in status.values}
        values["state"] = status.message
        return values

    def _on_controller_status(self, message):
        self.controller_values = self._diagnostic_values(message)

    def _on_localization_status(self, message):
        self.localization_values = self._diagnostic_values(message)

    def _on_lidar_status(self, message):
        self.lidar_values = self._diagnostic_values(message)

    def _on_safety_status(self, message):
        self.safety_values = self._diagnostic_values(message)

    def _on_command(self, message):
        receive_s = self.get_clock().now().nanoseconds * 1e-9
        pose_stamp_s = float("nan")
        phi = x = y = float("nan")
        if self.latest_pose is not None:
            pose_stamp_s = stamp_to_seconds(self.latest_pose.header.stamp)
            pose = self.latest_pose.pose.pose
            phi = quaternion_to_yaw(pose.orientation)
            x = float(pose.position.x)
            y = float(pose.position.y)
        self.writer.writerow({
            "receive_time_s": receive_s,
            "command_stamp_s": stamp_to_seconds(message.header.stamp),
            "pose_stamp_s": pose_stamp_s,
            "pose_age_s": receive_s - pose_stamp_s,
            "phi_rad": phi,
            "x_m": x,
            "y_m": y,
            "speed_mps": float(message.drive.speed),
            "steering_angle_rad": float(message.drive.steering_angle),
            "steering_rate_rad_s": float(message.drive.steering_angle_velocity),
            "actuator_state": self.actuator_values.get("state", "unknown"),
            "esc_duty_percent": self.actuator_values.get("esc_duty_percent", ""),
            "steering_duty_percent": self.actuator_values.get(
                "steering_duty_percent", ""
            ),
            "hardware_output_enabled": self.actuator_values.get(
                "hardware_output_enabled", "false"
            ),
            "controller_state": self.controller_values.get("state", "unknown"),
            "controller_solve_time_ms": self.controller_values.get(
                "solve_time_ms", ""
            ),
            "controller_overrun": self.controller_values.get("overrun", ""),
            "controller_deadline_exceeded": self.controller_values.get(
                "deadline_exceeded", ""
            ),
            "localization_state": self.localization_values.get("state", "unknown"),
            "localization_match_score": self.localization_values.get(
                "match_score_m2",
                self.localization_values.get("match_score", ""),
            ),
            "localization_valid_beams": self.localization_values.get(
                "valid_beams", ""
            ),
            "lidar_transport_state": self.lidar_values.get("state", "unknown"),
            "lidar_scan_rate_hz": self.lidar_values.get("scan_rate_hz", ""),
            "lidar_raw_points": self.lidar_values.get("raw_points", ""),
            "lidar_valid_beams": self.lidar_values.get("valid_beams", ""),
            "lidar_invalid_packets": self.lidar_values.get(
                "invalid_packet_count", ""
            ),
            "lidar_incomplete_scans": self.lidar_values.get(
                "incomplete_scans", ""
            ),
            "lidar_sequence_gaps": self.lidar_values.get("sequence_gaps", ""),
            "lidar_source_clock_offset_s": self.lidar_values.get(
                "source_clock_offset_s", ""
            ),
            "safety_state": self.safety_values.get("state", "unknown"),
            "safety_fault_reason": self.safety_values.get("fault_reason", ""),
            "safety_scan_age_s": self.safety_values.get("scan_age_s", ""),
            "safety_pose_age_s": self.safety_values.get("pose_age_s", ""),
            "safety_control_age_s": self.safety_values.get("control_age_s", ""),
        })
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IntegrationLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
