"""Known-map LiDAR localizer exposed as a ROS 2 node."""

import numpy as np

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ..configuration import declare_common_parameters, parameter_value
from ..lidar_transport import resample_periodic_scan
from ..repository import enable_repository_imports
from ..ros_support import quaternion_to_yaw


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("real_vehicle_localization")
        declare_common_parameters(self)
        root = enable_repository_imports()
        from experiments.real_vehicle.evaluation.common import load_real_config
        from experiments.real_vehicle.localization import GridSearchLocalizer, VirtualLidar

        self.declare_parameter(
            "vehicle_config",
            str(root / "experiments/real_vehicle/config/real_vehicle.yaml"),
        )
        self.declare_parameter("localizer_xy_step_m", 0.10)
        self.declare_parameter("localizer_theta_step_rad", 0.05)
        self.declare_parameter("localizer_search_xy_m", 0.30)
        self.declare_parameter("localizer_search_theta_rad", 0.15)
        self.declare_parameter("localizer_allow_scan_resample", True)
        config, _parameters, _limits = load_real_config(
            parameter_value(self, "vehicle_config")
        )
        self.lidar = VirtualLidar(config["lidar"], seed=int(config["controller"]["seed"]))
        self.localizer = GridSearchLocalizer(
            self.lidar,
            xy_step_m=float(parameter_value(self, "localizer_xy_step_m")),
            theta_step_rad=float(parameter_value(self, "localizer_theta_step_rad")),
            search_xy_m=float(parameter_value(self, "localizer_search_xy_m")),
            search_theta_rad=float(parameter_value(self, "localizer_search_theta_rad")),
        )
        self.predicted_pose = None
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(parameter_value(self, "pose_topic")),
            10,
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(parameter_value(self, "localization_status_topic")),
            10,
        )
        self.create_subscription(
            Odometry,
            str(parameter_value(self, "odom_topic")),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(parameter_value(self, "scan_topic")),
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _on_odom(self, message):
        pose = message.pose.pose
        self.predicted_pose = np.array([
            quaternion_to_yaw(pose.orientation),
            float(pose.position.x),
            float(pose.position.y),
        ])

    def _on_scan(self, message):
        if self.predicted_pose is None:
            return
        ranges = np.asarray(message.ranges, dtype=float)
        expected_increment = float(self.lidar.angle_increment)
        geometry_mismatch = (
            ranges.shape != (self.lidar.num_beams,)
            or not np.isclose(float(message.angle_min), self.lidar.angle_min)
            or not np.isclose(float(message.angle_increment), expected_increment)
        )
        if geometry_mismatch:
            if not bool(parameter_value(self, "localizer_allow_scan_resample")):
                self.get_logger().error(
                    "scan geometry mismatch and resampling is disabled: "
                    f"expected {self.lidar.num_beams} beams, got {len(ranges)}"
                )
                return
            try:
                ranges = np.asarray(resample_periodic_scan(
                    ranges,
                    float(message.angle_min),
                    float(message.angle_increment),
                    self.lidar.angles,
                ))
            except ValueError as exc:
                self.get_logger().error(f"cannot resample LiDAR scan: {exc}")
                return
        scan = {
            "header": {
                "stamp_s": float(message.header.stamp.sec)
                + 1e-9 * float(message.header.stamp.nanosec),
                "frame_id": message.header.frame_id,
            },
            "angle_min": float(self.lidar.angle_min),
            "angle_max": float(
                self.lidar.angle_min
                + self.lidar.angle_increment * (self.lidar.num_beams - 1)
            ),
            "angle_increment": float(self.lidar.angle_increment),
            "range_min": float(message.range_min),
            "range_max": float(message.range_max),
            "ranges": ranges,
        }
        estimate, score = self.localizer.estimate(scan, self.predicted_pose)
        output = PoseWithCovarianceStamped()
        output.header = message.header
        output.header.frame_id = str(parameter_value(self, "map_frame"))
        output.pose.pose.position.x = float(estimate[1])
        output.pose.pose.position.y = float(estimate[2])
        half = 0.5 * float(estimate[0])
        output.pose.pose.orientation.z = float(np.sin(half))
        output.pose.pose.orientation.w = float(np.cos(half))
        variance = max(float(score), 1e-9)
        output.pose.covariance[0] = variance
        output.pose.covariance[7] = variance
        output.pose.covariance[35] = variance
        self.predicted_pose = estimate
        self.pose_publisher.publish(output)
        status_message = DiagnosticArray()
        status_message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "real_vehicle_localization"
        status.hardware_id = "known_map_lidar"
        status.level = (
            DiagnosticStatus.OK if self.localizer.last_success else DiagnosticStatus.ERROR
        )
        status.message = "localized" if self.localizer.last_success else "match_failed"
        valid = np.isfinite(ranges) & (ranges < self.lidar.range_max_m - 1e-9)
        scan_age_s = (
            self.get_clock().now().nanoseconds * 1e-9
            - scan["header"]["stamp_s"]
        )
        status.values = [
            KeyValue(key="match_score", value=f"{float(score):.9f}"),
            KeyValue(key="valid_beams", value=str(int(np.count_nonzero(valid)))),
            KeyValue(key="total_beams", value=str(len(ranges))),
            KeyValue(
                key="scan_age_s",
                value=f"{scan_age_s:.6f}",
            ),
        ]
        status_message.status = [status]
        self.status_publisher.publish(status_message)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
