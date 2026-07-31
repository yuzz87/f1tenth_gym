"""Relay scan, odometry, and control topics with deterministic network faults."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ..fault_injection import FaultInjectionQueue, FaultProfile
from ..ros_support import AckermannDriveStamped, require_ackermann_messages


STREAMS = ("scan", "odom", "control")


class NetworkFaultInjectorNode(Node):
    def __init__(self):
        require_ackermann_messages()
        super().__init__("network_fault_injector")
        self.declare_parameter("relay_scan", False)
        self.declare_parameter("relay_odom", False)
        self.declare_parameter("relay_control", False)
        self.declare_parameter("scan_input_topic", "/transport/scan")
        self.declare_parameter("scan_output_topic", "/scan")
        self.declare_parameter("odom_input_topic", "/transport/odom")
        self.declare_parameter("odom_output_topic", "/odom")
        self.declare_parameter("control_input_topic", "/transport/control_request")
        self.declare_parameter("control_output_topic", "/vehicle/control_request")
        self.declare_parameter("status_topic", "/network_fault_injector/status")
        self.declare_parameter("fault_seed", 1)
        self.declare_parameter("relay_rate_hz", 500.0)
        self.queues = {}
        self.relay_publishers = {}
        self._configure_stream("scan", LaserScan, qos_profile_sensor_data, 0)
        self._configure_stream("odom", Odometry, qos_profile_sensor_data, 1000)
        self._configure_stream("control", AckermannDriveStamped, 10, 2000)
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("status_topic").value),
            10,
        )
        rate_hz = max(float(self.get_parameter("relay_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate_hz, self._release_ready)
        self.create_timer(1.0, self._publish_status)

    def _configure_stream(self, name, message_type, qos, seed_offset):
        self.declare_parameter(f"{name}_delay_s", 0.0)
        self.declare_parameter(f"{name}_delay_jitter_s", 0.0)
        self.declare_parameter(f"{name}_dropout_probability", 0.0)
        self.declare_parameter(f"{name}_burst_start_probability", 0.0)
        self.declare_parameter(f"{name}_burst_length_messages", 1)
        self.declare_parameter(f"{name}_outage_after_s", -1.0)
        self.declare_parameter(f"{name}_outage_duration_s", 0.0)
        self.declare_parameter(f"{name}_duplicate_probability", 0.0)
        self.declare_parameter(f"{name}_stale_replay_probability", 0.0)
        self.declare_parameter(f"{name}_minimum_publish_period_s", 0.0)
        if not bool(self.get_parameter(f"relay_{name}").value):
            return
        profile = FaultProfile(
            delay_s=float(self.get_parameter(f"{name}_delay_s").value),
            delay_jitter_s=float(
                self.get_parameter(f"{name}_delay_jitter_s").value
            ),
            dropout_probability=float(
                self.get_parameter(f"{name}_dropout_probability").value
            ),
            burst_start_probability=float(
                self.get_parameter(f"{name}_burst_start_probability").value
            ),
            burst_length_messages=int(
                self.get_parameter(f"{name}_burst_length_messages").value
            ),
            outage_after_s=float(
                self.get_parameter(f"{name}_outage_after_s").value
            ),
            outage_duration_s=float(
                self.get_parameter(f"{name}_outage_duration_s").value
            ),
            duplicate_probability=float(
                self.get_parameter(f"{name}_duplicate_probability").value
            ),
            stale_replay_probability=float(
                self.get_parameter(f"{name}_stale_replay_probability").value
            ),
            minimum_publish_period_s=float(
                self.get_parameter(f"{name}_minimum_publish_period_s").value
            ),
            seed=int(self.get_parameter("fault_seed").value) + seed_offset,
        )
        self.queues[name] = FaultInjectionQueue(profile)
        self.relay_publishers[name] = self.create_publisher(
            message_type,
            str(self.get_parameter(f"{name}_output_topic").value),
            qos,
        )
        self.create_subscription(
            message_type,
            str(self.get_parameter(f"{name}_input_topic").value),
            lambda message, stream=name: self._receive(stream, message),
            qos,
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _receive(self, stream, message):
        self.queues[stream].push(message, self._now_s())

    def _release_ready(self):
        now_s = self._now_s()
        for stream, queue in self.queues.items():
            for message in queue.pop_ready(now_s):
                self.relay_publishers[stream].publish(message)

    def _publish_status(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        statuses = []
        for stream, queue in self.queues.items():
            values = queue.statistics()
            status = DiagnosticStatus()
            status.name = f"network_fault_injector/{stream}"
            status.hardware_id = "ros2_dds_transport"
            status.level = DiagnosticStatus.WARN if values["dropped"] else DiagnosticStatus.OK
            status.message = "faults_injected" if values["dropped"] else "relaying"
            status.values = [
                KeyValue(key=key, value=str(value)) for key, value in values.items()
            ]
            status.values.extend([
                KeyValue(key="delay_s", value=str(queue.profile.delay_s)),
                KeyValue(
                    key="dropout_probability",
                    value=str(queue.profile.dropout_probability),
                ),
            ])
            statuses.append(status)
        message.status = statuses
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = NetworkFaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
