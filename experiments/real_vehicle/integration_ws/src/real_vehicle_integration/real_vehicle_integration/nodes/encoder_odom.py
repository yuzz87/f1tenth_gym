"""Receive Pi encoder UDP packets and publish model-based ROS 2 odometry."""

import math
import socket
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from ..configuration import declare_common_parameters, parameter_value
from ..odom_estimator import (
    CALIBRATED_ENCODER_COUNTS_PER_REVOLUTION,
    CALIBRATED_ENCODER_DISTANCE_PER_COUNT_M,
    ENCODER_CALIBRATION_LABEL,
    BicycleOdomEstimator,
    encoder_count_to_distance,
)
from ..repository import enable_repository_imports
from ..ros_support import yaw_to_quaternion


class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__("real_vehicle_encoder_odom")
        declare_common_parameters(self)
        self.declare_parameter("encoder_udp_bind_host", "0.0.0.0")
        self.declare_parameter("encoder_udp_port", 5011)
        self.declare_parameter("encoder_udp_allowed_host", "")
        self.declare_parameter("encoder_timeout_s", 0.50)
        self.declare_parameter("encoder_gpio_a", 22)
        self.declare_parameter("encoder_gpio_b", 27)
        self.declare_parameter("encoder_wheel_diameter_m", 0.066)
        self.declare_parameter(
            "encoder_counts_per_revolution",
            CALIBRATED_ENCODER_COUNTS_PER_REVOLUTION,
        )
        self.declare_parameter(
            "encoder_distance_per_count_m",
            CALIBRATED_ENCODER_DISTANCE_PER_COUNT_M,
        )
        self.declare_parameter(
            "encoder_calibration_label",
            ENCODER_CALIBRATION_LABEL,
        )
        self.declare_parameter("wheelbase_m", 0.25)
        self.declare_parameter("steering_neutral_duty_percent", 10.895)
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("encoder_status_topic", "/encoder/transport_status")

        enable_repository_imports()
        from experiments.real_vehicle.adapters.command_adapter import (
            steer_duty_to_angle,
        )
        from experiments.real_vehicle.pi_encoder_agent.protocol import (
            ProtocolError,
            decode_sample,
        )
        from experiments.real_vehicle.pi_encoder_agent.receiver import (
            EncoderPacketMonitor,
        )

        self.steer_duty_to_angle = steer_duty_to_angle
        self.ProtocolError = ProtocolError
        self.decode_sample = decode_sample
        self.monitor = EncoderPacketMonitor(
            allowed_sender=str(parameter_value(self, "encoder_udp_allowed_host"))
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((
            str(parameter_value(self, "encoder_udp_bind_host")),
            int(parameter_value(self, "encoder_udp_port")),
        ))
        self.socket.setblocking(False)

        distance_per_count = float(
            parameter_value(self, "encoder_distance_per_count_m")
        )
        if distance_per_count <= 0.0:
            wheel_diameter = float(
                parameter_value(self, "encoder_wheel_diameter_m")
            )
            counts_per_revolution = float(
                parameter_value(self, "encoder_counts_per_revolution")
            )
            if wheel_diameter <= 0.0 or counts_per_revolution <= 0.0:
                raise ValueError("encoder wheel and count parameters must be positive")
            distance_per_count = math.pi * wheel_diameter / counts_per_revolution
        self.distance_per_count_m = distance_per_count
        self.estimator = BicycleOdomEstimator(
            wheelbase_m=float(parameter_value(self, "wheelbase_m")),
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            str(parameter_value(self, "odom_topic")),
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(parameter_value(self, "encoder_status_topic")),
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            DiagnosticArray,
            str(parameter_value(self, "actuator_status_topic")),
            self._on_actuator_status,
            10,
        )
        self.last_steering_duty = float(
            parameter_value(self, "steering_neutral_duty_percent")
        )
        self.last_steering_angle_rad = 0.0
        self.last_sample = None
        self.last_sample_received_monotonic = None
        self.last_source_monotonic_ns = None
        self.last_speed_mps = 0.0
        self.last_angular_speed_rad_s = 0.0
        self.last_sender = ""
        self.last_decode_error = ""
        self.timer = self.create_timer(0.01, self._poll_udp)
        self.odom_timer = self.create_timer(0.02, self._publish_odom)
        self.status_timer = self.create_timer(0.10, self._publish_status)

        self.get_logger().info(
            "encoder odom listening "
            f"{parameter_value(self, 'encoder_udp_bind_host')}:"
            f"{parameter_value(self, 'encoder_udp_port')} "
            f"distance_per_count_m={self.distance_per_count_m:.9f} "
            f"calibration={parameter_value(self, 'encoder_calibration_label')}"
        )

    def _on_actuator_status(self, message):
        for status in message.status:
            for value in status.values:
                if value.key != "steering_duty_percent":
                    continue
                try:
                    duty = float(value.value)
                    neutral = float(
                        parameter_value(self, "steering_neutral_duty_percent")
                    )
                    self.last_steering_duty = duty
                    self.last_steering_angle_rad = float(
                        self.steer_duty_to_angle(duty, neutral)
                    )
                except (TypeError, ValueError):
                    self.get_logger().warning(
                        f"invalid steering duty in actuator status: {value.value}"
                    )

    def _poll_udp(self):
        while True:
            try:
                packet, sender = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().error(f"encoder UDP receive failed: {exc}")
                break

            received = time.monotonic()
            sample = self.monitor.accept(packet, sender, received)
            if sample is None:
                self.last_decode_error = "invalid_or_rejected_packet"
                continue
            self.last_decode_error = ""
            self.last_sender = f"{sender[0]}:{sender[1]}"
            self._apply_sample(sample, received)

    def _apply_sample(self, sample, received_monotonic):
        # A new session starts from its first received count; it must not cause
        # a pose jump after an agent restart.
        if self.last_sample is None or sample["session_id"] != self.last_sample["session_id"]:
            self.last_sample = sample
            self.last_sample_received_monotonic = received_monotonic
            self.last_source_monotonic_ns = sample["source_monotonic_ns"]
            self.last_speed_mps = 0.0
            self.last_angular_speed_rad_s = 0.0
            return

        sample_period_s = float(sample["sample_period_s"])
        if sample_period_s <= 1e-9:
            sample_period_s = max(
                received_monotonic - float(self.last_sample_received_monotonic),
                1e-9,
            )
        distance_m = encoder_count_to_distance(
            sample["delta_count"], self.distance_per_count_m
        )
        self.estimator.integrate(distance_m, self.last_steering_angle_rad)
        self.last_speed_mps = distance_m / sample_period_s
        self.last_angular_speed_rad_s = (
            self.last_speed_mps
            / self.estimator.wheelbase_m
            * math.tan(self.last_steering_angle_rad)
        )
        self.last_sample = sample
        self.last_sample_received_monotonic = received_monotonic
        self.last_source_monotonic_ns = sample["source_monotonic_ns"]

    def _publish_odom(self):
        now = self.get_clock().now().to_msg()
        yaw_rad, x_m, y_m = self.estimator.pose()
        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)
        odom_frame = str(parameter_value(self, "odom_frame"))
        base_frame = str(parameter_value(self, "base_frame"))
        message = Odometry()
        message.header.stamp = now
        message.header.frame_id = odom_frame
        message.child_frame_id = base_frame
        message.pose.pose.position.x = float(x_m)
        message.pose.pose.position.y = float(y_m)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        age_s = self._packet_age_s()
        if age_s > float(parameter_value(self, "encoder_timeout_s")):
            linear_speed = 0.0
            angular_speed = 0.0
        else:
            linear_speed = self.last_speed_mps
            angular_speed = self.last_angular_speed_rad_s
        message.twist.twist.linear.x = float(linear_speed)
        message.twist.twist.angular.z = float(angular_speed)

        # The encoder observes forward travel directly. Lateral and yaw
        # uncertainty are deliberately larger because they come from a model.
        distance_uncertainty = max(0.002, abs(self.estimator.distance_m) * 0.02)
        yaw_uncertainty = max(0.05, abs(self.estimator.distance_m) * 0.10)
        message.pose.covariance[0] = distance_uncertainty ** 2
        message.pose.covariance[7] = distance_uncertainty ** 2 * 4.0
        message.pose.covariance[35] = yaw_uncertainty ** 2
        message.twist.covariance[0] = 0.05 ** 2
        message.twist.covariance[35] = 0.20 ** 2
        self.odom_publisher.publish(message)

        if bool(parameter_value(self, "publish_odom_tf")):
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = odom_frame
            transform.child_frame_id = base_frame
            transform.transform.translation.x = float(x_m)
            transform.transform.translation.y = float(y_m)
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    def _packet_age_s(self):
        if self.last_sample_received_monotonic is None:
            return math.inf
        return max(0.0, time.monotonic() - self.last_sample_received_monotonic)

    def _publish_status(self):
        status_message = DiagnosticArray()
        status_message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "real_vehicle_encoder_transport"
        status.hardware_id = "raspberry_pi_encoder_udp"
        packet_age_s = self._packet_age_s()
        timed_out = packet_age_s > float(parameter_value(self, "encoder_timeout_s"))
        invalid_transition_count = 0
        if self.last_sample is not None:
            invalid_transition_count = self.last_sample["invalid_transition_count"]
        if timed_out:
            status.level = DiagnosticStatus.ERROR
            status.message = "encoder_udp_timeout"
        elif invalid_transition_count > 0 or self.monitor.sequence_gaps > 0:
            status.level = DiagnosticStatus.WARN
            status.message = "encoder_quality_warning"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "encoder_healthy"
        status.values = [
            KeyValue(key="sender", value=self.last_sender),
            KeyValue(key="packet_age_s", value=f"{packet_age_s:.6f}"),
            KeyValue(key="packet_count", value=str(self.monitor.accepted_packets)),
            KeyValue(key="invalid_packet_count", value=str(self.monitor.invalid_packet_count)),
            KeyValue(key="rejected_sender_count", value=str(self.monitor.rejected_sender_count)),
            KeyValue(key="duplicate_packets", value=str(self.monitor.duplicate_packets)),
            KeyValue(key="sequence_gaps", value=str(self.monitor.sequence_gaps)),
            KeyValue(key="session_changes", value=str(self.monitor.session_changes)),
            KeyValue(key="session_id", value=str(self.monitor.session_id or "")),
            KeyValue(key="distance_per_count_m", value=f"{self.distance_per_count_m:.9f}"),
            KeyValue(key="calibration", value=str(parameter_value(self, "encoder_calibration_label"))),
            KeyValue(key="invalid_transition_count", value=str(invalid_transition_count)),
            KeyValue(key="odom_x_m", value=f"{self.estimator.x_m:.6f}"),
            KeyValue(key="odom_y_m", value=f"{self.estimator.y_m:.6f}"),
            KeyValue(key="odom_yaw_rad", value=f"{self.estimator.yaw_rad:.6f}"),
            KeyValue(key="measured_speed_mps", value=f"{self.last_speed_mps if not timed_out else 0.0:.6f}"),
            KeyValue(key="steering_angle_rad", value=f"{self.last_steering_angle_rad:.6f}"),
        ]
        status_message.status = [status]
        self.status_publisher.publish(status_message)

    def destroy_node(self):
        try:
            self.socket.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
