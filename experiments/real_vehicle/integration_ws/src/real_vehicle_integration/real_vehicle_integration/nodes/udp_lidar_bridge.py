"""Receive normalized RPLIDAR scans over UDP and publish ROS 2 LaserScan."""

from collections import deque
import math
import socket
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ..configuration import declare_common_parameters, parameter_value
from ..lidar_analysis import mask_near_field_sector
from ..lidar_transport import (
    FLAG_HEALTH_OK,
    LidarProtocolError,
    ScanReassembler,
    decode_scan_packet,
)


class UdpLidarBridgeNode(Node):
    def __init__(self):
        super().__init__("udp_lidar_bridge")
        declare_common_parameters(self)
        self.declare_parameter("lidar_udp_bind_host", "0.0.0.0")
        self.declare_parameter("lidar_udp_port", 5010)
        self.declare_parameter("lidar_udp_allowed_host", "")
        self.declare_parameter("lidar_reassembly_timeout_s", 0.25)
        self.declare_parameter("lidar_source_timeout_s", 0.50)
        self.declare_parameter("lidar_use_source_timestamp", False)
        self.declare_parameter("lidar_minimum_valid_ratio", 0.05)
        self.declare_parameter("lidar_rear_self_filter_enabled", True)
        self.declare_parameter("lidar_rear_self_filter_half_width_deg", 30.0)
        self.declare_parameter("lidar_rear_self_filter_max_range_m", 12.0)
        self.allowed_address = self._resolve_allowed_address(
            str(parameter_value(self, "lidar_udp_allowed_host"))
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        bind_address = (
            str(parameter_value(self, "lidar_udp_bind_host")),
            int(parameter_value(self, "lidar_udp_port")),
        )
        try:
            self.socket.bind(bind_address)
        except OSError as exc:
            self.socket.close()
            raise RuntimeError(
                "cannot bind LiDAR UDP socket to "
                f"{bind_address[0]}:{bind_address[1]}; another receiver may "
                "already be running"
            ) from exc
        self.socket.setblocking(False)
        self.reassembler = ScanReassembler(
            timeout_s=float(parameter_value(self, "lidar_reassembly_timeout_s"))
        )
        self.scan_publisher = self.create_publisher(
            LaserScan,
            str(parameter_value(self, "scan_topic")),
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(parameter_value(self, "lidar_transport_status_topic")),
            10,
        )
        self.packet_count = 0
        self.invalid_packet_count = 0
        self.rejected_sender_count = 0
        self.last_packet_monotonic_s = None
        self.last_scan_monotonic_s = None
        self.last_scan = None
        self.last_sender = ""
        self.last_source_clock_offset_s = float("nan")
        self.last_masked_beams = 0
        self.last_published_valid_beams = 0
        self.scan_receive_times = deque(maxlen=64)
        self.create_timer(0.005, self._poll_socket)
        self.create_timer(0.10, self._publish_diagnostics)
        address = self.socket.getsockname()
        self.get_logger().info(
            f"LiDAR UDP listening on {address[0]}:{address[1]}"
        )

    @staticmethod
    def _resolve_allowed_address(host):
        if not host:
            return None
        return socket.gethostbyname(host)

    def _poll_socket(self):
        now = time.monotonic()
        self.reassembler.expire(now)
        while True:
            try:
                datagram, sender = self.socket.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().error(f"LiDAR UDP receive failed: {exc}")
                break
            if self.allowed_address is not None and sender[0] != self.allowed_address:
                self.rejected_sender_count += 1
                continue
            self.packet_count += 1
            self.last_packet_monotonic_s = time.monotonic()
            self.last_sender = f"{sender[0]}:{sender[1]}"
            try:
                packet = decode_scan_packet(datagram)
                scan = self.reassembler.add(
                    packet,
                    now_s=self.last_packet_monotonic_s,
                )
            except (LidarProtocolError, TypeError, ValueError) as exc:
                self.invalid_packet_count += 1
                if self.invalid_packet_count <= 3 or self.invalid_packet_count % 100 == 0:
                    self.get_logger().warning(f"invalid LiDAR UDP packet: {exc}")
                continue
            if scan is not None:
                self._publish_scan(scan)

    def _publish_scan(self, scan):
        ranges = [float(value) for value in scan.ranges]
        self.last_masked_beams = 0
        if bool(parameter_value(self, "lidar_rear_self_filter_enabled")):
            ranges, self.last_masked_beams = mask_near_field_sector(
                ranges,
                scan.angle_min_rad,
                scan.angle_increment_rad,
                -math.pi,
                math.radians(float(parameter_value(
                    self,
                    "lidar_rear_self_filter_half_width_deg",
                ))),
                float(parameter_value(
                    self,
                    "lidar_rear_self_filter_max_range_m",
                )),
            )
        self.last_published_valid_beams = sum(
            math.isfinite(value) for value in ranges
        )
        message = LaserScan()
        if bool(parameter_value(self, "lidar_use_source_timestamp")):
            message.header.stamp.sec = int(scan.capture_time_ns // 1000000000)
            message.header.stamp.nanosec = int(scan.capture_time_ns % 1000000000)
        else:
            message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(parameter_value(self, "laser_frame"))
        message.angle_min = float(scan.angle_min_rad)
        message.angle_increment = float(scan.angle_increment_rad)
        message.angle_max = float(
            scan.angle_min_rad + scan.angle_increment_rad * (len(scan.ranges) - 1)
        )
        message.range_min = float(scan.range_min_m)
        message.range_max = float(scan.range_max_m)
        message.scan_time = float(scan.scan_period_us) * 1e-6
        message.time_increment = (
            message.scan_time / len(scan.ranges) if scan.ranges else 0.0
        )
        message.ranges = ranges
        self.scan_publisher.publish(message)
        now_monotonic = time.monotonic()
        self.last_scan_monotonic_s = now_monotonic
        self.scan_receive_times.append(now_monotonic)
        self.last_source_clock_offset_s = (
            time.time() - scan.capture_time_ns * 1e-9
        )
        self.last_scan = scan

    def _receive_rate_hz(self):
        if len(self.scan_receive_times) < 2:
            return 0.0
        elapsed = self.scan_receive_times[-1] - self.scan_receive_times[0]
        return (len(self.scan_receive_times) - 1) / elapsed if elapsed > 0 else 0.0

    def _publish_diagnostics(self):
        now = time.monotonic()
        self.reassembler.expire(now)
        status_message = DiagnosticArray()
        status_message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "real_vehicle_lidar_transport"
        status.hardware_id = "rplidar_a1_udp"
        age_s = (
            float("inf")
            if self.last_scan_monotonic_s is None
            else now - self.last_scan_monotonic_s
        )
        timeout_s = float(parameter_value(self, "lidar_source_timeout_s"))
        valid_ratio = (
            0.0
            if self.last_scan is None or not self.last_scan.ranges
            else self.last_published_valid_beams / len(self.last_scan.ranges)
        )
        health_ok = bool(
            self.last_scan is not None
            and self.last_scan.flags & FLAG_HEALTH_OK
        )
        if self.last_scan is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "waiting_for_lidar_udp"
        elif age_s > timeout_s:
            status.level = DiagnosticStatus.ERROR
            status.message = "lidar_udp_timeout"
        elif not health_ok:
            status.level = DiagnosticStatus.ERROR
            status.message = "lidar_health_not_ok"
        elif valid_ratio < float(parameter_value(self, "lidar_minimum_valid_ratio")):
            status.level = DiagnosticStatus.WARN
            status.message = "low_valid_beam_ratio"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "receiving"
        latest = self.last_scan
        values = {
            "sender": self.last_sender,
            "scan_age_s": f"{age_s:.6f}",
            "scan_rate_hz": f"{self._receive_rate_hz():.3f}",
            "packet_count": self.packet_count,
            "invalid_packet_count": self.invalid_packet_count,
            "rejected_sender_count": self.rejected_sender_count,
            "completed_scans": self.reassembler.completed_scans,
            "incomplete_scans": self.reassembler.incomplete_scans,
            "duplicate_packets": self.reassembler.duplicate_packets,
            "sequence_gaps": self.reassembler.sequence_gaps,
            "pending_scans": len(self.reassembler.pending),
            "source_clock_offset_s": f"{self.last_source_clock_offset_s:.6f}",
            "session_id": "" if latest is None else latest.session_id,
            "scan_id": "" if latest is None else latest.scan_id,
            "raw_points": "" if latest is None else latest.raw_points,
            "source_valid_beams": "" if latest is None else latest.valid_beams,
            "valid_beams": "" if latest is None else self.last_published_valid_beams,
            "rear_self_filter_enabled": str(bool(parameter_value(
                self,
                "lidar_rear_self_filter_enabled",
            ))).lower(),
            "rear_self_filter_masked_beams": self.last_masked_beams,
            "rear_self_filter_half_width_deg": parameter_value(
                self,
                "lidar_rear_self_filter_half_width_deg",
            ),
            "rear_self_filter_max_range_m": parameter_value(
                self,
                "lidar_rear_self_filter_max_range_m",
            ),
            "total_beams": "" if latest is None else len(latest.ranges),
            "valid_ratio": f"{valid_ratio:.6f}",
            "health_ok": str(health_ok).lower(),
        }
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()
        ]
        status_message.status = [status]
        self.status_publisher.publish(status_message)

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpLidarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
