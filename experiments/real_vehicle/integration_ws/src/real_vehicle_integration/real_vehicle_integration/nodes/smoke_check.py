"""Observe the running integration graph and report a machine-readable result."""

import json

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ..configuration import declare_common_parameters, parameter_value
from ..ros_support import AckermannDriveStamped, require_ackermann_messages


class SmokeCheckNode(Node):
    def __init__(self):
        require_ackermann_messages()
        super().__init__("real_vehicle_smoke_check")
        declare_common_parameters(self)
        self.declare_parameter("duration_s", 5.0)
        self.counts = {
            "scan": 0,
            "odom": 0,
            "pose": 0,
            "control_request": 0,
            "control_safe": 0,
            "actuator_status": 0,
        }
        self.maximum_safe_speed_mps = 0.0
        self.hardware_output_seen = False
        self.finished = False
        self.success = False
        self.create_subscription(
            LaserScan,
            str(parameter_value(self, "scan_topic")),
            lambda _message: self._count("scan"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(parameter_value(self, "odom_topic")),
            lambda _message: self._count("odom"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(parameter_value(self, "pose_topic")),
            lambda _message: self._count("pose"),
            10,
        )
        self.create_subscription(
            AckermannDriveStamped,
            str(parameter_value(self, "control_request_topic")),
            lambda _message: self._count("control_request"),
            10,
        )
        self.create_subscription(
            AckermannDriveStamped,
            str(parameter_value(self, "control_safe_topic")),
            self._on_safe_control,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            str(parameter_value(self, "actuator_status_topic")),
            self._on_actuator_status,
            10,
        )
        duration = max(float(self.get_parameter("duration_s").value), 0.1)
        self.timer = self.create_timer(duration, self._finish)

    def _count(self, name):
        self.counts[name] += 1

    def _on_safe_control(self, message):
        self._count("control_safe")
        self.maximum_safe_speed_mps = max(
            self.maximum_safe_speed_mps, float(message.drive.speed)
        )

    def _on_actuator_status(self, message):
        self._count("actuator_status")
        for status in message.status:
            values = {value.key: value.value for value in status.values}
            self.hardware_output_seen = self.hardware_output_seen or (
                values.get("hardware_output_enabled", "false").lower() == "true"
            )

    def _finish(self):
        missing = [name for name, count in self.counts.items() if count == 0]
        self.success = not missing and not self.hardware_output_seen
        result = {
            "success": self.success,
            "counts": self.counts,
            "maximum_safe_speed_mps": self.maximum_safe_speed_mps,
            "hardware_output_seen": self.hardware_output_seen,
            "missing_topics": missing,
        }
        print(json.dumps(result, indent=2), flush=True)
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = SmokeCheckNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)
