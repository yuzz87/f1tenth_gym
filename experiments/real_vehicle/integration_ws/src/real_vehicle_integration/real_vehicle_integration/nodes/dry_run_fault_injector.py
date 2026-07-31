"""Relay real sensor/control messages and inject explicitly triggered faults."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from ..dry_run_faults import DryRunFaultGate, SUPPORTED_FAULT_MODES
from ..ros_support import AckermannDriveStamped, require_ackermann_messages


class DryRunFaultInjectorNode(Node):
    """Provide a zero-hardware message boundary with deterministic outages."""

    def __init__(self):
        require_ackermann_messages()
        super().__init__("real_vehicle_dry_run_fault_injector")
        self.declare_parameter("fault_mode", "none")
        self.declare_parameter("scan_input_topic", "/scan")
        self.declare_parameter("scan_output_topic", "/dry_run/scan")
        self.declare_parameter(
            "pose_input_topic",
            "/localization/pose",
        )
        self.declare_parameter("pose_output_topic", "/dry_run/pose")
        self.declare_parameter(
            "control_input_topic",
            "/dry_run/control_raw",
        )
        self.declare_parameter(
            "control_output_topic",
            "/dry_run/control_request",
        )
        self.declare_parameter(
            "emergency_stop_topic",
            "/dry_run/emergency_stop",
        )
        self.declare_parameter(
            "status_topic",
            "/dry_run/fault_injector_status",
        )
        mode = str(self.get_parameter("fault_mode").value).strip().lower()
        try:
            self.gate = DryRunFaultGate(mode)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            raise
        self.trigger_stamp_s = None

        self.scan_publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter("scan_output_topic").value),
            qos_profile_sensor_data,
        )
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_output_topic").value),
            10,
        )
        self.control_publisher = self.create_publisher(
            AckermannDriveStamped,
            str(self.get_parameter("control_output_topic").value),
            10,
        )
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
            LaserScan,
            str(self.get_parameter("scan_input_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_input_topic").value),
            self._on_pose,
            10,
        )
        self.create_subscription(
            AckermannDriveStamped,
            str(self.get_parameter("control_input_topic").value),
            self._on_control,
            10,
        )
        self.create_service(Trigger, "~/trigger", self._on_trigger)
        self.create_service(Trigger, "~/clear", self._on_clear)
        self.create_timer(0.10, self._publish_status)
        self._publish_emergency(False)
        self.get_logger().info(
            "dry-run fault relay ready: "
            f"mode={self.gate.mode}; trigger service is explicit"
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, message):
        if self.gate.should_relay("scan"):
            self.scan_publisher.publish(message)

    def _on_pose(self, message):
        if self.gate.should_relay("pose"):
            self.pose_publisher.publish(message)

    def _on_control(self, message):
        if self.gate.should_relay("control"):
            self.control_publisher.publish(message)

    def _publish_emergency(self, active):
        message = Bool()
        message.data = bool(active)
        self.emergency_publisher.publish(message)

    def _on_trigger(self, _request, response):
        try:
            self.gate.trigger()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            return response
        self.trigger_stamp_s = self._now_s()
        self._publish_emergency(self.gate.emergency_stop_active)
        response.success = True
        response.message = f"triggered {self.gate.mode}"
        self.get_logger().warning(response.message)
        return response

    def _on_clear(self, _request, response):
        self.gate.clear()
        self.trigger_stamp_s = None
        self._publish_emergency(False)
        response.success = True
        response.message = "fault cleared"
        self.get_logger().info(response.message)
        return response

    def _publish_status(self):
        values = self.gate.statistics()
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "real_vehicle_dry_run_fault_injector"
        status.hardware_id = "ros_only_fault_gate"
        status.level = (
            DiagnosticStatus.WARN
            if self.gate.active
            else DiagnosticStatus.OK
        )
        status.message = (
            f"active:{self.gate.mode}"
            if self.gate.active
            else f"inactive:{self.gate.mode}"
        )
        values["supported_modes"] = ",".join(SUPPORTED_FAULT_MODES)
        values["trigger_stamp_s"] = (
            "nan"
            if self.trigger_stamp_s is None
            else f"{self.trigger_stamp_s:.9f}"
        )
        status.values = [
            KeyValue(key=str(key), value=str(value).lower())
            for key, value in values.items()
        ]
        message.status = [status]
        self.status_publisher.publish(message)

    def destroy_node(self):
        try:
            self._publish_emergency(False)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DryRunFaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
