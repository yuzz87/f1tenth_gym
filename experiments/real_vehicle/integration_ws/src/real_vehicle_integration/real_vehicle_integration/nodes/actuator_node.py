"""Raspberry Pi command endpoint with an independent command watchdog."""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from ..actuator_policy import is_neutral_stop
from ..configuration import (
    contract_limits_from_node,
    declare_common_parameters,
    parameter_value,
)
from ..contracts import ActuatorOutput, validate_control
from ..pwm_backends import create_pwm_backend
from ..repository import enable_repository_imports
from ..ros_support import (
    AckermannDriveStamped,
    ackermann_message_to_contract,
    require_ackermann_messages,
)


class ActuatorNode(Node):
    def __init__(self):
        require_ackermann_messages()
        super().__init__("real_vehicle_actuator")
        declare_common_parameters(self)
        self.declare_parameter("esc_stop_duty_percent", 10.30)
        self.declare_parameter("esc_start_duty_percent", 10.16)
        self.declare_parameter("esc_forward_min_duty_percent", 10.10)
        self.declare_parameter("steering_neutral_duty_percent", 10.895)
        self.declare_parameter("pwm_frequency_hz", 70.0)
        self.declare_parameter("pwm_backend", "mock")
        self.declare_parameter("hardware_output_enabled", False)
        self.declare_parameter("allow_uncalibrated_speed_to_duty", False)
        enable_repository_imports()
        from experiments.real_vehicle.adapters.command_adapter import (
            sanitize_esc_duty,
            speed_to_esc_duty,
            steer_angle_to_duty,
        )

        self.sanitize_esc_duty = sanitize_esc_duty
        self.speed_to_esc_duty = speed_to_esc_duty
        self.steer_angle_to_duty = steer_angle_to_duty
        self.limits = contract_limits_from_node(self)
        self.hardware_output_enabled = bool(
            parameter_value(self, "hardware_output_enabled")
        )
        self.allow_uncalibrated = bool(
            parameter_value(self, "allow_uncalibrated_speed_to_duty")
        )
        self.output_armed = False
        self.emergency_stop = False
        self.emergency_stop_input = False
        self.sequence_id = 0
        self.last_command_receive_s = None
        self.last_output = None
        self.current_output = None
        self.backend = create_pwm_backend(
            str(parameter_value(self, "pwm_backend")),
            hardware_output_enabled=self.hardware_output_enabled,
            esc_neutral=float(parameter_value(self, "esc_stop_duty_percent")),
            steering_neutral=float(
                parameter_value(self, "steering_neutral_duty_percent")
            ),
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(parameter_value(self, "actuator_status_topic")),
            10,
        )
        self.create_subscription(
            AckermannDriveStamped,
            str(parameter_value(self, "control_safe_topic")),
            self._on_command,
            1,
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
        self.create_service(SetBool, "~/arm_output", self._on_arm_output)
        self.create_service(
            Trigger,
            "~/reset_emergency_stop",
            self._on_reset_emergency_stop,
        )
        rate = max(float(parameter_value(self, "actuator_rate_hz")), 1e-6)
        self.timer = self.create_timer(1.0 / rate, self._watchdog)
        self._neutral("startup")

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_arm_output(self, request, response):
        if request.data and self.emergency_stop:
            response.success = False
            response.message = "emergency stop is active"
            return response
        self.output_armed = bool(request.data)
        if not self.output_armed:
            self._neutral("output_disarmed")
        response.success = True
        response.message = "armed" if self.output_armed else "disarmed"
        return response

    def _on_emergency_stop(self, message):
        self.emergency_stop_input = bool(message.data)
        if message.data:
            self.emergency_stop = True
            self.output_armed = False
            self._neutral("emergency_stop")

    def _on_reset_emergency_stop(self, _request, response):
        if self.emergency_stop_input:
            response.success = False
            response.message = "emergency stop input is still active"
            return response
        self.emergency_stop = False
        self.output_armed = False
        self._neutral("emergency_stop_reset")
        response.success = True
        response.message = "emergency stop reset; output remains disarmed"
        return response

    def _on_command(self, message):
        now_s = self._now_s()
        self.sequence_id += 1
        try:
            command = ackermann_message_to_contract(message, self.sequence_id)
            command = validate_control(
                command,
                now_s,
                float(parameter_value(self, "command_timeout_s")),
                self.limits,
            )
        except (TypeError, ValueError) as exc:
            self._neutral(f"invalid_command:{exc}")
            return
        self.last_command_receive_s = now_s
        if self.emergency_stop:
            self._neutral("emergency_stop")
            return
        if is_neutral_stop(command):
            self._neutral("safe_stop")
            return
        if command.speed_mps > 0.0 and not self.allow_uncalibrated:
            self._neutral("uncalibrated_speed_to_duty")
            return
        esc_duty = self.speed_to_esc_duty(
            command.speed_mps,
            speed_command_max_mps=self.limits.speed_max_mps,
            stop_duty_percent=float(parameter_value(self, "esc_stop_duty_percent")),
            start_duty_percent=float(parameter_value(self, "esc_start_duty_percent")),
            forward_min_duty_percent=float(
                parameter_value(self, "esc_forward_min_duty_percent")
            ),
        )
        esc_duty, _clamped = self.sanitize_esc_duty(
            esc_duty,
            stop_duty_percent=float(parameter_value(self, "esc_stop_duty_percent")),
            start_duty_percent=float(parameter_value(self, "esc_start_duty_percent")),
            forward_min_duty_percent=float(
                parameter_value(self, "esc_forward_min_duty_percent")
            ),
            allow_experimental_duty=False,
        )
        steering_duty = self.steer_angle_to_duty(
            command.steering_angle_rad,
            float(parameter_value(self, "steering_neutral_duty_percent")),
        )
        enabled = self.hardware_output_enabled and self.output_armed
        reason = "running" if enabled else "dry_run"
        output = ActuatorOutput(
            stamp_s=now_s,
            sequence_id=command.sequence_id,
            esc_duty_percent=float(esc_duty),
            steering_duty_percent=float(steering_duty),
            enabled=enabled,
            reason=reason,
        )
        self.current_output = output
        self.last_output = self.backend.apply(output)
        self._publish_status(output)

    def _neutral(self, reason):
        now_s = self._now_s()
        self.current_output = ActuatorOutput(
            stamp_s=now_s,
            sequence_id=self.sequence_id,
            esc_duty_percent=float(parameter_value(self, "esc_stop_duty_percent")),
            steering_duty_percent=float(
                parameter_value(self, "steering_neutral_duty_percent")
            ),
            enabled=False,
            reason=str(reason),
        )
        self.last_output = self.backend.neutral(now_s, reason)
        self._publish_status(self.current_output)

    def _watchdog(self):
        now_s = self._now_s()
        if self.last_command_receive_s is None:
            if self.current_output is not None:
                self._publish_status(self.current_output)
            return
        if now_s - self.last_command_receive_s > float(
            parameter_value(self, "command_timeout_s")
        ):
            self.last_command_receive_s = None
            self._neutral("command_timeout")
        elif self.current_output is not None:
            self._publish_status(self.current_output)

    def _publish_status(self, output):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "real_vehicle_actuator"
        status.hardware_id = "raspberry_pi_pwm"
        status.level = DiagnosticStatus.OK if output.enabled else DiagnosticStatus.WARN
        status.message = output.reason
        status.values = [
            KeyValue(key="esc_duty_percent", value=f"{output.esc_duty_percent:.6f}"),
            KeyValue(
                key="steering_duty_percent",
                value=f"{output.steering_duty_percent:.6f}",
            ),
            KeyValue(key="enabled", value=str(bool(output.enabled)).lower()),
            KeyValue(key="output_armed", value=str(self.output_armed).lower()),
            KeyValue(
                key="hardware_output_enabled",
                value=str(self.hardware_output_enabled).lower(),
            ),
        ]
        message.status = [status]
        self.status_publisher.publish(message)

    def destroy_node(self):
        try:
            self.get_logger().warning("actuator neutral: node_shutdown")
            self._neutral("node_shutdown")
            self.backend.close(self._now_s())
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
