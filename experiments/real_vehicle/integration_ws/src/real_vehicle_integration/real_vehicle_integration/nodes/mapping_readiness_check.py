"""Verify that every input required for real-vehicle mapping is live."""

import json
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class MappingReadinessCheck(Node):
    def __init__(self):
        super().__init__("mapping_readiness_check")
        self.declare_parameter("timeout_s", 10.0)
        self.started_monotonic = time.monotonic()
        self.received = {
            "scan": False,
            "odom": False,
            "map": False,
            "lidar_status": False,
            "encoder_status": False,
            "map_to_base_tf": False,
        }
        self.diagnostic_messages = {
            "lidar": "",
            "encoder": "",
        }
        self.diagnostic_healthy = {
            "lidar": False,
            "encoder": False,
        }
        self.result = None

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            LaserScan,
            "/scan",
            lambda _message: self._mark("scan"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            lambda _message: self._mark("odom"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            lambda _message: self._mark("map"),
            map_qos,
        )
        self.create_subscription(
            DiagnosticArray,
            "/lidar/transport_status",
            self._on_lidar_status,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/encoder/transport_status",
            self._on_encoder_status,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.10, self._check)

    def _mark(self, key):
        self.received[key] = True

    def _on_lidar_status(self, message):
        self._record_status("lidar", "lidar_status", message)

    def _on_encoder_status(self, message):
        self._record_status("encoder", "encoder_status", message)

    def _record_status(self, source, key, message):
        self.received[key] = True
        if message.status:
            self.diagnostic_messages[source] = message.status[0].message
            self.diagnostic_healthy[source] = (
                message.status[0].level < DiagnosticStatus.ERROR
            )

    def _check(self):
        if not self.received["map_to_base_tf"]:
            self.received["map_to_base_tf"] = bool(
                self.tf_buffer.can_transform(
                    "map",
                    "base_link",
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
            )

        elapsed_s = time.monotonic() - self.started_monotonic
        if all(self.received.values()) and all(self.diagnostic_healthy.values()):
            self._finish(True, elapsed_s)
            return

        timeout_s = float(self.get_parameter("timeout_s").value)
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            timeout_s = 10.0
        if elapsed_s >= timeout_s:
            self._finish(False, elapsed_s)

    def _finish(self, passed, elapsed_s):
        if self.result is not None:
            return
        missing = [key for key, value in self.received.items() if not value]
        unhealthy = [
            key for key, value in self.diagnostic_healthy.items() if not value
        ]
        self.result = {
            "status": "passed" if passed else "failed",
            "elapsed_s": round(elapsed_s, 3),
            "missing": missing,
            "unhealthy": unhealthy,
            "received": self.received,
            "diagnostics": self.diagnostic_messages,
        }
        print(json.dumps(self.result, indent=2, sort_keys=True), flush=True)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MappingReadinessCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.result is None:
            node.result = {"status": "interrupted"}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.result and node.result.get("status") == "passed" else 1
