"""Authorize controller requests and publish only safe vehicle commands."""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from ..configuration import declare_common_parameters, parameter_value, safety_config_from_node
from ..contracts import neutral_control
from ..ros_support import (
    AckermannDriveStamped,
    ackermann_message_to_contract,
    contract_to_ackermann_message,
    require_ackermann_messages,
    stamp_to_seconds,
)
from ..safety_monitor import SafetyMonitor
from ..state_machine import SafetyState


class SafetyNode(Node):
    def __init__(self):
        require_ackermann_messages()
        super().__init__("real_vehicle_safety")
        declare_common_parameters(self)
        self.declare_parameter("auto_arm", False)
        self.declare_parameter("auto_start", False)
        self.monitor = SafetyMonitor(safety_config_from_node(self))
        self.sequence_id = 0
        self.last_reported_fault = ""
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            str(parameter_value(self, "control_safe_topic")),
            1,
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(parameter_value(self, "safety_status_topic")),
            10,
        )
        self.create_subscription(
            AckermannDriveStamped,
            str(parameter_value(self, "control_request_topic")),
            self._on_control,
            1,
        )
        self.create_subscription(
            LaserScan,
            str(parameter_value(self, "scan_topic")),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(parameter_value(self, "pose_topic")),
            self._on_pose,
            10,
        )
        emergency_qos = QoSProfile(depth=1)
        emergency_qos.reliability = ReliabilityPolicy.RELIABLE
        emergency_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            str(parameter_value(self, "emergency_stop_topic")),
            self._on_emergency_stop,
            emergency_qos,
        )
        self.create_service(SetBool, "~/arm", self._on_arm)
        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/stop", self._on_stop)
        self.create_service(Trigger, "~/reset_fault", self._on_reset)
        rate = max(float(parameter_value(self, "safety_rate_hz")), 1e-6)
        self.timer = self.create_timer(1.0 / rate, self._watchdog)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, message):
        self.monitor.update_scan(stamp_to_seconds(message.header.stamp))

    def _on_pose(self, message):
        self.monitor.update_pose(stamp_to_seconds(message.header.stamp))

    def _on_emergency_stop(self, message):
        self.monitor.set_emergency_stop(bool(message.data))
        if message.data:
            self._publish_neutral("emergency_stop")

    def _on_control(self, message):
        self.sequence_id += 1
        command = ackermann_message_to_contract(message, self.sequence_id)
        safe = self.monitor.process(command, self._now_s())
        self.publisher.publish(
            contract_to_ackermann_message(safe, self.get_clock().now().to_msg())
        )

    def _on_arm(self, request, response):
        try:
            if request.data:
                self.monitor.arm(self._now_s())
                response.message = "armed and ready"
            else:
                self.monitor.stop()
                if self.monitor.machine.state == SafetyState.STOPPING:
                    self.monitor.machine.mark_stopped()
                response.message = "stopped"
            response.success = True
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        self._publish_neutral("arm_state_change")
        return response

    def _on_start(self, _request, response):
        try:
            self.monitor.start(self._now_s())
            response.success = True
            response.message = "running"
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _on_stop(self, _request, response):
        self.monitor.stop()
        if self.monitor.machine.state == SafetyState.STOPPING:
            self.monitor.machine.mark_stopped()
        self._publish_neutral("manual_stop")
        response.success = True
        response.message = "stopped"
        return response

    def _on_reset(self, _request, response):
        try:
            self.monitor.reset()
            response.success = True
            response.message = "disarmed"
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        self._publish_neutral("fault_reset")
        return response

    def _publish_neutral(self, reason):
        self.sequence_id += 1
        command = neutral_control(
            self._now_s(),
            self.sequence_id,
            str(parameter_value(self, "base_frame")),
        )
        self.publisher.publish(
            contract_to_ackermann_message(command, self.get_clock().now().to_msg())
        )
        if reason:
            self.get_logger().debug(f"neutral command: {reason}")

    def _watchdog(self):
        now_s = self._now_s()
        if bool(parameter_value(self, "auto_arm")):
            try:
                if self.monitor.machine.state == SafetyState.DISARMED:
                    self.monitor.arm(now_s)
                if (
                    bool(parameter_value(self, "auto_start"))
                    and self.monitor.machine.state == SafetyState.READY
                    and self.monitor.last_command_stamp_s is not None
                ):
                    self.monitor.start(now_s)
            except RuntimeError:
                pass
        problem = self.monitor.watchdog(now_s)
        if problem and problem != self.last_reported_fault:
            self.get_logger().error(f"safety fault: {problem}")
            self.last_reported_fault = problem
        if not self.monitor.machine.output_enabled:
            self._publish_neutral(problem or self.monitor.machine.state.value)
        self._publish_status(now_s, problem)

    def _publish_status(self, now_s, problem):
        state = self.monitor.machine.state
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "real_vehicle_safety"
        status.hardware_id = "raspberry_pi_safety"
        status.level = (
            DiagnosticStatus.ERROR if state == SafetyState.FAULT else
            DiagnosticStatus.OK if state == SafetyState.RUNNING else
            DiagnosticStatus.WARN
        )
        status.message = problem or state.value

        def age(stamp):
            return "nan" if stamp is None else f"{now_s - stamp:.6f}"

        status.values = [
            KeyValue(key="state", value=state.value),
            KeyValue(key="fault_reason", value=self.monitor.machine.fault_reason),
            KeyValue(key="scan_age_s", value=age(self.monitor.last_scan_stamp_s)),
            KeyValue(key="pose_age_s", value=age(self.monitor.last_pose_stamp_s)),
            KeyValue(key="control_age_s", value=age(self.monitor.last_command_stamp_s)),
            KeyValue(
                key="output_enabled",
                value=str(self.monitor.machine.output_enabled).lower(),
            ),
        ]
        message.status = [status]
        self.status_publisher.publish(message)

    def destroy_node(self):
        try:
            self._publish_neutral("node_shutdown")
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
