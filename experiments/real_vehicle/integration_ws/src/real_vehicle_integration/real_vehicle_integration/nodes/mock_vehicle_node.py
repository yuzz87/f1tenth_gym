"""Closed-loop virtual vehicle that publishes /scan, /odom, and TF."""

import math

import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from ..configuration import declare_common_parameters, parameter_value
from ..repository import enable_repository_imports
from ..ros_support import AckermannDriveStamped, require_ackermann_messages, yaw_to_quaternion


class MockVehicleNode(Node):
    def __init__(self):
        require_ackermann_messages()
        super().__init__("real_vehicle_mock_vehicle")
        declare_common_parameters(self)
        root = enable_repository_imports()
        from experiments.real_vehicle.evaluation.common import load_real_config
        from experiments.real_vehicle.localization import VirtualLidar
        from experiments.real_vehicle.models import ActuatorModel

        self.declare_parameter(
            "vehicle_config",
            str(root / "experiments/real_vehicle/config/real_vehicle.yaml"),
        )
        self.declare_parameter("reference_type", "straight")
        self.declare_parameter("circle_radius_m", 2.0)
        config, parameters, limits = load_real_config(
            parameter_value(self, "vehicle_config")
        )
        self.config = config
        self.parameters = parameters
        self.limits = limits
        self.dt = float(config["simulation"]["timestep_s"])
        self.state = np.zeros(4, dtype=float)
        if str(parameter_value(self, "reference_type")).lower() == "circle":
            self.state[0] = math.pi / 2.0
            self.state[1] = float(parameter_value(self, "circle_radius_m"))
        self.control = np.zeros(2, dtype=float)
        self.last_command_s = None
        self.actuator = ActuatorModel(
            parameters,
            limits,
            config["simulation"].get("actuator", {}),
            seed=int(config["controller"].get("seed", 7)),
        )
        self.actuator.reset(self.state)
        self.lidar = VirtualLidar(
            config["lidar"], seed=int(config["controller"].get("seed", 7))
        )
        self.last_scan_publish_s = -math.inf
        self.scan_period_s = 1.0 / max(
            float(config["lidar"].get("update_rate_hz", 10.0)), 1e-6
        )
        self.scan_rate_jitter_fraction = max(
            float(config["lidar"].get("update_rate_jitter_fraction", 0.0)),
            0.0,
        )
        self.drop_burst_messages = bool(
            config["lidar"].get("burst_drop_message", False)
        )
        self.schedule_rng = np.random.default_rng(
            int(config["controller"].get("seed", 7)) + 9000
        )
        self.next_scan_publish_s = -math.inf
        self.scan_publisher = self.create_publisher(
            LaserScan,
            str(parameter_value(self, "scan_topic")),
            qos_profile_sensor_data,
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            str(parameter_value(self, "odom_topic")),
            qos_profile_sensor_data,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            AckermannDriveStamped,
            str(parameter_value(self, "control_safe_topic")),
            self._on_control,
            1,
        )
        self.timer = self.create_timer(self.dt, self._step)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_control(self, message):
        self.control[0] = float(np.clip(
            message.drive.speed,
            self.limits.speed_min_mps,
            self.limits.speed_max_mps,
        ))
        self.control[1] = float(np.clip(
            message.drive.steering_angle_velocity,
            self.limits.steer_rate_min_rad_s,
            self.limits.steer_rate_max_rad_s,
        ))
        self.last_command_s = self._now_s()

    def _step(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        timeout = float(parameter_value(self, "command_timeout_s"))
        if self.last_command_s is None or now_s - self.last_command_s > timeout:
            self.control[0] = 0.0
            self.control[1] = float(np.clip(
                -self.state[3] / max(self.dt, 1e-6),
                self.limits.steer_rate_min_rad_s,
                self.limits.steer_rate_max_rad_s,
            ))
        self.state, _clamp, _actuator_log = self.actuator.step(
            self.state,
            self.control,
            self.parameters,
            self.dt,
            integrator=str(self.config["simulation"].get("integrator", "rk4")),
        )
        self._publish_odom_and_tf(now.to_msg())
        if now_s >= self.next_scan_publish_s:
            self._publish_scan(now_s)
            self.last_scan_publish_s = now_s
            jitter = 0.0
            if self.scan_rate_jitter_fraction > 0.0:
                jitter = float(self.schedule_rng.normal(
                    0.0,
                    self.scan_rate_jitter_fraction,
                ))
            self.next_scan_publish_s = now_s + self.scan_period_s * max(
                1.0 + jitter,
                0.1,
            )

    def _publish_scan(self, now_s):
        scan = self.lidar.scan(self.state, now_s)
        if self.drop_burst_messages and scan.get("burst_dropped", False):
            return
        message = LaserScan()
        stamp_s = float(scan["header"]["stamp_s"])
        message.header.stamp.sec = int(stamp_s)
        message.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
        message.header.frame_id = str(parameter_value(self, "laser_frame"))
        message.angle_min = float(scan["angle_min"])
        message.angle_max = float(scan["angle_max"])
        message.angle_increment = float(scan["angle_increment"])
        message.time_increment = 0.0
        message.scan_time = float(self.scan_period_s)
        message.range_min = float(scan["range_min"])
        message.range_max = float(scan["range_max"])
        message.ranges = [float(value) for value in scan["ranges"]]
        self.scan_publisher.publish(message)

    def _publish_odom_and_tf(self, stamp):
        map_frame = str(parameter_value(self, "map_frame"))
        odom_frame = str(parameter_value(self, "odom_frame"))
        base_frame = str(parameter_value(self, "base_frame"))
        laser_frame = str(parameter_value(self, "laser_frame"))
        qx, qy, qz, qw = yaw_to_quaternion(self.state[0])
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = float(self.state[1])
        odom.pose.pose.position.y = float(self.state[2])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self.control[0])
        odom.twist.twist.angular.z = float(
            self.control[0] / self.parameters.wheelbase_m * np.tan(self.state[3])
        )
        self.odom_publisher.publish(odom)

        transforms = []
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = map_frame
        map_to_odom.child_frame_id = odom_frame
        map_to_odom.transform.rotation.w = 1.0
        transforms.append(map_to_odom)
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = odom_frame
        odom_to_base.child_frame_id = base_frame
        odom_to_base.transform.translation.x = float(self.state[1])
        odom_to_base.transform.translation.y = float(self.state[2])
        odom_to_base.transform.rotation.x = qx
        odom_to_base.transform.rotation.y = qy
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw
        transforms.append(odom_to_base)
        base_to_laser = TransformStamped()
        base_to_laser.header.stamp = stamp
        base_to_laser.header.frame_id = base_frame
        base_to_laser.child_frame_id = laser_frame
        base_to_laser.transform.rotation.w = 1.0
        transforms.append(base_to_laser)
        self.tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = MockVehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
