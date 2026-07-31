"""Latch a stop if hardware-capable nodes enter the Phase 6 ROS graph."""

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from ..hardware_isolation import (
    find_forbidden_nodes,
    is_isolated_control_topic,
)
from ..ros_support import (
    AckermannDriveStamped,
    require_ackermann_messages,
)


class DryRunGuardNode(Node):
    """Audit graph isolation and dry-run command constraints."""

    def __init__(self):
        require_ackermann_messages()
        super().__init__("real_vehicle_dry_run_guard")
        self.declare_parameter(
            "control_safe_topic",
            "/dry_run/control_safe",
        )
        self.declare_parameter(
            "emergency_stop_topic",
            "/dry_run/emergency_stop",
        )
        self.declare_parameter(
            "status_topic",
            "/dry_run/hardware_guard_status",
        )
        self.declare_parameter("speed_min_mps", 0.0)
        self.declare_parameter("speed_max_mps", 0.30)
        self.declare_parameter("steer_min_rad", -0.3141592653589793)
        self.declare_parameter("steer_max_rad", 0.3141592653589793)
        self.declare_parameter("steer_rate_min_rad_s", -0.70)
        self.declare_parameter("steer_rate_max_rad_s", 0.70)

        self.control_safe_topic = str(
            self.get_parameter("control_safe_topic").value
        )
        self.topic_isolated = is_isolated_control_topic(
            self.control_safe_topic
        )
        self.forbidden_nodes = []
        self.violation_latched = not self.topic_isolated
        self.command_count = 0
        self.non_neutral_count = 0
        self.constraint_violation_count = 0
        self.max_abs_speed_mps = 0.0
        self.max_abs_steering_rad = 0.0
        self.max_abs_steering_rate_rad_s = 0.0

        emergency_qos = QoSProfile(depth=1)
        emergency_qos.reliability = ReliabilityPolicy.RELIABLE
        emergency_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.emergency_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            emergency_qos,
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            AckermannDriveStamped,
            self.control_safe_topic,
            self._on_command,
            10,
        )
        self.create_timer(0.10, self._audit)

    @staticmethod
    def _inside(value, lower, upper):
        return (
            math.isfinite(value)
            and float(lower) - 1e-6 <= value <= float(upper) + 1e-6
        )

    def _on_command(self, message):
        speed = float(message.drive.speed)
        steering = float(message.drive.steering_angle)
        steering_rate = float(message.drive.steering_angle_velocity)
        self.command_count += 1
        if abs(speed) > 1e-6 or abs(steering_rate) > 1e-6:
            self.non_neutral_count += 1
        self.max_abs_speed_mps = max(self.max_abs_speed_mps, abs(speed))
        self.max_abs_steering_rad = max(
            self.max_abs_steering_rad,
            abs(steering),
        )
        self.max_abs_steering_rate_rad_s = max(
            self.max_abs_steering_rate_rad_s,
            abs(steering_rate),
        )
        checks = (
            self._inside(
                speed,
                self.get_parameter("speed_min_mps").value,
                self.get_parameter("speed_max_mps").value,
            ),
            self._inside(
                steering,
                self.get_parameter("steer_min_rad").value,
                self.get_parameter("steer_max_rad").value,
            ),
            self._inside(
                steering_rate,
                self.get_parameter("steer_rate_min_rad_s").value,
                self.get_parameter("steer_rate_max_rad_s").value,
            ),
        )
        if not all(checks):
            self.constraint_violation_count += 1
            self.violation_latched = True

    def _publish_stop(self):
        message = Bool()
        message.data = True
        self.emergency_publisher.publish(message)

    def _audit(self):
        self.forbidden_nodes = find_forbidden_nodes(
            self.get_node_names()
        )
        if self.forbidden_nodes:
            self.violation_latched = True
        if self.violation_latched:
            self._publish_stop()

        hardware_path_absent = (
            self.topic_isolated
            and not self.forbidden_nodes
            and not self.violation_latched
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "real_vehicle_dry_run_guard"
        status.hardware_id = "ros_graph_isolation"
        status.level = (
            DiagnosticStatus.OK
            if hardware_path_absent
            else DiagnosticStatus.ERROR
        )
        status.message = (
            "hardware_path_absent"
            if hardware_path_absent
            else "hardware_isolation_violation"
        )
        status.values = [
            KeyValue(
                key="hardware_path_absent",
                value=str(hardware_path_absent).lower(),
            ),
            KeyValue(
                key="topic_isolated",
                value=str(self.topic_isolated).lower(),
            ),
            KeyValue(
                key="violation_latched",
                value=str(self.violation_latched).lower(),
            ),
            KeyValue(
                key="forbidden_nodes",
                value=",".join(self.forbidden_nodes),
            ),
            KeyValue(
                key="command_count",
                value=str(self.command_count),
            ),
            KeyValue(
                key="non_neutral_count",
                value=str(self.non_neutral_count),
            ),
            KeyValue(
                key="constraint_violation_count",
                value=str(self.constraint_violation_count),
            ),
            KeyValue(
                key="max_abs_speed_mps",
                value=f"{self.max_abs_speed_mps:.9f}",
            ),
            KeyValue(
                key="max_abs_steering_rad",
                value=f"{self.max_abs_steering_rad:.9f}",
            ),
            KeyValue(
                key="max_abs_steering_rate_rad_s",
                value=f"{self.max_abs_steering_rate_rad_s:.9f}",
            ),
            KeyValue(
                key="control_safe_topic",
                value=self.control_safe_topic,
            ),
        ]
        message.status = [status]
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DryRunGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
