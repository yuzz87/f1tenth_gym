"""Bridge safe ROS 2 commands to the standalone Raspberry Pi UDP agent."""

import socket
import time
import uuid

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from ..configuration import declare_common_parameters, parameter_value
from ..repository import enable_repository_imports
from ..ros_support import (
    AckermannDriveStamped,
    ackermann_message_to_contract,
    require_ackermann_messages,
)


class UdpActuatorBridgeNode(Node):
    def __init__(self):
        require_ackermann_messages()
        super().__init__("udp_actuator_bridge")
        declare_common_parameters(self)
        self.declare_parameter("pi_udp_host", "raspberrypi.local")
        self.declare_parameter("pi_udp_port", 5005)
        self.declare_parameter("udp_local_port", 5007)
        self.declare_parameter("udp_status_timeout_s", 0.30)
        self.declare_parameter("allow_uncalibrated_speed_to_duty", False)
        self.declare_parameter("esc_stop_duty_percent", 10.30)
        self.declare_parameter("esc_start_duty_percent", 10.16)
        self.declare_parameter("esc_forward_min_duty_percent", 10.10)
        self.declare_parameter("steering_neutral_duty_percent", 10.895)
        self.declare_parameter("steering_min_duty_percent", 9.05)
        self.declare_parameter("steering_max_duty_percent", 12.42)
        enable_repository_imports()
        from experiments.real_vehicle.adapters.command_adapter import (
            sanitize_esc_duty,
            speed_to_esc_duty,
            steer_angle_to_duty,
        )
        from experiments.real_vehicle.pi_agent.protocol import (
            ProtocolError,
            decode_status,
            encode_command,
            make_command,
        )

        self.sanitize_esc_duty = sanitize_esc_duty
        self.speed_to_esc_duty = speed_to_esc_duty
        self.steer_angle_to_duty = steer_angle_to_duty
        self.ProtocolError = ProtocolError
        self.decode_status = decode_status
        self.encode_command = encode_command
        self.make_command = make_command
        remote_host = str(parameter_value(self, "pi_udp_host"))
        remote_port = int(parameter_value(self, "pi_udp_port"))
        self.remote = (socket.gethostbyname(remote_host), remote_port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("0.0.0.0", int(parameter_value(self, "udp_local_port"))))
        self.socket.setblocking(False)
        self.session_id = uuid.uuid4().hex
        self.sequence_id = 0
        self.last_status = None
        self.last_status_monotonic = None
        self.sent_packets = 0
        self.received_status_packets = 0
        self.invalid_status_packets = 0
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
        self.timer = self.create_timer(0.02, self._poll_status)
        self.diagnostic_timer = self.create_timer(0.10, self._publish_status)
        self._send_neutral("bridge_startup")

    def _next_sequence(self):
        self.sequence_id += 1
        return self.sequence_id

    def _duty_values(self, command):
        stop_duty = float(parameter_value(self, "esc_stop_duty_percent"))
        steering_neutral = float(
            parameter_value(self, "steering_neutral_duty_percent")
        )
        if command.speed_mps <= 1e-9:
            return stop_duty, steering_neutral, False, "safe_stop"
        if not bool(parameter_value(self, "allow_uncalibrated_speed_to_duty")):
            return stop_duty, steering_neutral, False, "uncalibrated_speed_to_duty"
        esc_duty = self.speed_to_esc_duty(
            command.speed_mps,
            speed_command_max_mps=float(parameter_value(self, "speed_max_mps")),
            stop_duty_percent=stop_duty,
            start_duty_percent=float(parameter_value(self, "esc_start_duty_percent")),
            forward_min_duty_percent=float(
                parameter_value(self, "esc_forward_min_duty_percent")
            ),
        )
        esc_duty, _clamped = self.sanitize_esc_duty(
            esc_duty,
            stop_duty_percent=stop_duty,
            start_duty_percent=float(parameter_value(self, "esc_start_duty_percent")),
            forward_min_duty_percent=float(
                parameter_value(self, "esc_forward_min_duty_percent")
            ),
            allow_experimental_duty=False,
        )
        steering_duty = self.steer_angle_to_duty(
            command.steering_angle_rad,
            steering_neutral,
        )
        steering_duty = min(
            max(
                steering_duty,
                float(parameter_value(self, "steering_min_duty_percent")),
            ),
            float(parameter_value(self, "steering_max_duty_percent")),
        )
        return float(esc_duty), float(steering_duty), True, "running_candidate"

    def _send(self, esc_duty, steering_duty, enable_output, reason):
        packet = self.encode_command(self.make_command(
            session_id=self.session_id,
            sequence_id=self._next_sequence(),
            sent_at_unix_s=time.time(),
            esc_duty_percent=float(esc_duty),
            steering_duty_percent=float(steering_duty),
            enable_output=bool(enable_output),
            reason=str(reason),
        ))
        try:
            self.socket.sendto(packet, self.remote)
            self.sent_packets += 1
        except OSError as exc:
            self.get_logger().error(f"UDP send failed: {exc}")

    def _send_neutral(self, reason):
        self._send(
            float(parameter_value(self, "esc_stop_duty_percent")),
            float(parameter_value(self, "steering_neutral_duty_percent")),
            False,
            reason,
        )

    def _on_command(self, message):
        try:
            command = ackermann_message_to_contract(message, self.sequence_id + 1)
            esc_duty, steering_duty, enable_output, reason = self._duty_values(
                command
            )
            self._send(esc_duty, steering_duty, enable_output, reason)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid safe command: {exc}")
            self._send_neutral("invalid_safe_command")

    def _poll_status(self):
        while True:
            try:
                packet, sender = self.socket.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().error(f"UDP receive failed: {exc}")
                break
            try:
                if sender != self.remote:
                    raise self.ProtocolError("status sender mismatch")
                status = self.decode_status(packet)
                if status["session_id"] not in ("", self.session_id):
                    raise self.ProtocolError("status session mismatch")
                self.last_status = status
                self.last_status_monotonic = time.monotonic()
                self.received_status_packets += 1
            except (self.ProtocolError, TypeError, ValueError) as exc:
                self.invalid_status_packets += 1
                self.get_logger().warning(f"invalid UDP status: {exc}")

    def _publish_status(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        diagnostic = DiagnosticStatus()
        diagnostic.name = "real_vehicle_actuator"
        diagnostic.hardware_id = "raspberry_pi_udp_pwm"
        now = time.monotonic()
        age_s = (
            float("inf")
            if self.last_status_monotonic is None
            else now - self.last_status_monotonic
        )
        timed_out = age_s > float(parameter_value(self, "udp_status_timeout_s"))
        if self.last_status is None or timed_out:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = "udp_status_timeout"
            values = {
                "esc_duty_percent": parameter_value(
                    self,
                    "esc_stop_duty_percent",
                ),
                "steering_duty_percent": parameter_value(
                    self,
                    "steering_neutral_duty_percent",
                ),
                "hardware_output_enabled": False,
                "output_armed": False,
                "state": "unreachable",
                "reason": "udp_status_timeout",
                "accepted_packets": 0,
                "rejected_packets": 0,
                "watchdog_count": 0,
            }
        else:
            values = self.last_status
            if values["state"] == "fault":
                diagnostic.level = DiagnosticStatus.ERROR
            elif values["state"] == "running":
                diagnostic.level = DiagnosticStatus.OK
            else:
                diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = values["reason"]
        diagnostic.values = [
            KeyValue(key="esc_duty_percent", value=str(values["esc_duty_percent"])),
            KeyValue(
                key="steering_duty_percent",
                value=str(values["steering_duty_percent"]),
            ),
            KeyValue(
                key="hardware_output_enabled",
                value=str(bool(values["hardware_output_enabled"])).lower(),
            ),
            KeyValue(
                key="output_armed",
                value=str(bool(values["output_armed"])).lower(),
            ),
            KeyValue(key="agent_state", value=str(values.get("state", "unreachable"))),
            KeyValue(key="agent_reason", value=str(values.get("reason", ""))),
            KeyValue(key="udp_status_age_s", value=f"{age_s:.6f}"),
            KeyValue(key="udp_sent_packets", value=str(self.sent_packets)),
            KeyValue(
                key="udp_received_status_packets",
                value=str(self.received_status_packets),
            ),
            KeyValue(
                key="udp_invalid_status_packets",
                value=str(self.invalid_status_packets),
            ),
            KeyValue(
                key="accepted_packets",
                value=str(values["accepted_packets"]),
            ),
            KeyValue(
                key="rejected_packets",
                value=str(values["rejected_packets"]),
            ),
            KeyValue(key="watchdog_count", value=str(values["watchdog_count"])),
        ]
        message.status = [diagnostic]
        self.status_publisher.publish(message)

    def destroy_node(self):
        try:
            for _ in range(3):
                self._send_neutral("bridge_shutdown")
                time.sleep(0.02)
        finally:
            self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpActuatorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
