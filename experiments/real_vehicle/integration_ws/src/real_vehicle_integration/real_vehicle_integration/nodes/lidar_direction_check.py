"""Report nearest obstacles in four sectors for mounting-direction checks."""

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ..configuration import declare_common_parameters, parameter_value
from ..lidar_analysis import sector_minima


class LidarDirectionCheckNode(Node):
    def __init__(self):
        super().__init__("lidar_direction_check")
        declare_common_parameters(self)
        self.declare_parameter("lidar_direction_sector_half_width_deg", 15.0)
        self.declare_parameter(
            "lidar_direction_status_topic",
            "/lidar/direction_status",
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(parameter_value(self, "lidar_direction_status_topic")),
            10,
        )
        self.last_log_ns = 0
        self.create_subscription(
            LaserScan,
            str(parameter_value(self, "scan_topic")),
            self._on_scan,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _format(value):
        return "inf" if not math.isfinite(value) else f"{value:.3f}"

    def _on_scan(self, message):
        minima = sector_minima(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            half_width_rad=math.radians(float(parameter_value(
                self,
                "lidar_direction_sector_half_width_deg",
            ))),
        )
        finite = {
            name: value for name, value in minima.items() if math.isfinite(value)
        }
        nearest = min(finite, key=finite.get) if finite else "none"
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "real_vehicle_lidar_direction"
        status.hardware_id = str(message.header.frame_id)
        status.level = DiagnosticStatus.OK if finite else DiagnosticStatus.WARN
        status.message = nearest
        status.values = [
            KeyValue(key=name + "_m", value=self._format(value))
            for name, value in minima.items()
        ]
        diagnostic_array.status = [status]
        self.status_publisher.publish(diagnostic_array)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= 1000000000:
            self.last_log_ns = now_ns
            self.get_logger().info(
                "front={front} m left={left} m rear={rear} m right={right} m "
                "nearest={nearest}".format(
                    nearest=nearest,
                    **{name: self._format(value) for name, value in minima.items()},
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarDirectionCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
