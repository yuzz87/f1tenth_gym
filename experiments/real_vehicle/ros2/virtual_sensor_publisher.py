"""Publish virtual LaserScan, Odometry, and TF messages for ROS2 checks."""

import argparse
import math
from pathlib import Path

import numpy as np

from ..evaluation.common import DEFAULT_CONFIG, load_real_config
from ..localization import VirtualLidar
from ..models import step_model

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster
except ImportError:
    rclpy = None
    TransformStamped = None
    Odometry = None
    LaserScan = None
    TransformBroadcaster = None

    class Node:
        pass


def _quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(float(yaw) / 2.0), math.cos(float(yaw) / 2.0)


class VirtualSensorPublisher(Node):
    """Publish a deterministic straight-moving virtual vehicle."""

    def __init__(self, config, scan_topic="/scan", odom_topic="/odom", duration_s=0.0):
        if rclpy is None:
            raise RuntimeError("ROS2 rclpy is required; source ROS2 Foxy first")
        super().__init__("real_vehicle_virtual_sensors")
        self.config = config
        self.parameters, self.limits = self._load_model(config)
        self.dt = float(config["simulation"]["timestep_s"])
        self.speed = float(config["controller"]["speed_target_mps"])
        self.duration_s = float(duration_s)
        self.elapsed_s = 0.0
        self.state = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
        self.lidar = VirtualLidar(config["lidar"], seed=int(config["controller"].get("seed", 7)))
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_link"
        self.laser_frame = str(config["lidar"].get("frame_id", "laser"))
        self.scan_publisher = self.create_publisher(LaserScan, scan_topic, 10)
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        update_rate = max(float(config["lidar"].get("update_rate_hz", 10.0)), 1e-6)
        self.timer = self.create_timer(1.0 / update_rate, self._publish_step)

    @staticmethod
    def _load_model(config):
        parameters, limits = load_real_config(Path(config["_config_path"]))[1:]
        return parameters, limits

    def _publish_step(self):
        stamp = self.get_clock().now().to_msg()
        scan = self.lidar.scan(self.state, self.elapsed_s)
        scan_message = LaserScan()
        scan_message.header.stamp = stamp
        scan_message.header.frame_id = self.laser_frame
        scan_message.angle_min = float(scan["angle_min"])
        scan_message.angle_max = float(scan["angle_max"])
        scan_message.angle_increment = float(scan["angle_increment"])
        scan_message.time_increment = float(scan["scan_time"] if "scan_time" in scan else 0.0)
        scan_message.scan_time = 1.0 / max(float(self.config["lidar"].get("update_rate_hz", 10.0)), 1e-6)
        scan_message.range_min = float(scan["range_min"])
        scan_message.range_max = float(scan["range_max"])
        scan_message.ranges = [float(value) for value in scan["ranges"]]
        self.scan_publisher.publish(scan_message)

        odom_message = Odometry()
        odom_message.header.stamp = stamp
        odom_message.header.frame_id = self.odom_frame
        odom_message.child_frame_id = self.base_frame
        odom_message.pose.pose.position.x = float(self.state[1])
        odom_message.pose.pose.position.y = float(self.state[2])
        qx, qy, qz, qw = _quaternion_from_yaw(self.state[0])
        odom_message.pose.pose.orientation.x = qx
        odom_message.pose.pose.orientation.y = qy
        odom_message.pose.pose.orientation.z = qz
        odom_message.pose.pose.orientation.w = qw
        odom_message.twist.twist.linear.x = self.speed
        self.odom_publisher.publish(odom_message)
        self._publish_tf(stamp)

        self.state, _ = step_model(
            self.state,
            [self.speed, 0.0],
            self.parameters,
            self.limits,
            self.dt,
            integrator=str(self.config["simulation"].get("integrator", "rk4")),
        )
        self.elapsed_s += self.dt
        if self.duration_s > 0.0 and self.elapsed_s >= self.duration_s:
            self.get_logger().info("virtual sensor duration completed")
            rclpy.shutdown()

    def _publish_tf(self, stamp):
        transforms = []
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = self.map_frame
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.rotation.w = 1.0
        transforms.append(map_to_odom)

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        odom_to_base.transform.translation.x = float(self.state[1])
        odom_to_base.transform.translation.y = float(self.state[2])
        qx, qy, qz, qw = _quaternion_from_yaw(self.state[0])
        odom_to_base.transform.rotation.x = qx
        odom_to_base.transform.rotation.y = qy
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw
        transforms.append(odom_to_base)

        base_to_laser = TransformStamped()
        base_to_laser.header.stamp = stamp
        base_to_laser.header.frame_id = self.base_frame
        base_to_laser.child_frame_id = self.laser_frame
        base_to_laser.transform.rotation.w = 1.0
        transforms.append(base_to_laser)
        self.tf_broadcaster.sendTransform(transforms)


def main():
    parser = argparse.ArgumentParser(description="Publish virtual /scan, /odom, and TF")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--duration", type=float, default=0.0)
    args = parser.parse_args()
    if rclpy is None:
        raise SystemExit("ROS2 rclpy is not available; source ROS2 Foxy first")
    config, _parameters, _limits = load_real_config(args.config)
    config["_config_path"] = str(args.config)
    rclpy.init()
    node = VirtualSensorPublisher(config, args.scan_topic, args.odom_topic, args.duration)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

