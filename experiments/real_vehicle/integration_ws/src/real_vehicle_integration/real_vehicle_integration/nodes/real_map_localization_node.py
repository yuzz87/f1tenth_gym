"""Publish real-vehicle pose estimates by matching LiDAR scans to a saved map."""

import time

import numpy as np
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from ..configuration import declare_common_parameters, parameter_value
from ..real_map_localizer import (
    GridMapLocalizer,
    OccupancyMap,
    _compose_pose,
    _inverse_pose,
    odom_delta,
)
from ..ros_support import quaternion_to_yaw, stamp_to_seconds, yaw_to_quaternion


class RealMapLocalizationNode(Node):
    """Known-map localizer with a conservative invalid-data state."""

    def __init__(self):
        super().__init__("real_map_localization")
        declare_common_parameters(self)
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("initial_x_m", 0.0)
        self.declare_parameter("initial_y_m", 0.0)
        self.declare_parameter("initial_yaw_rad", 0.0)
        self.declare_parameter("localizer_xy_step_m", 0.15)
        self.declare_parameter("localizer_theta_step_rad", 0.10)
        self.declare_parameter("localizer_search_xy_m", 0.30)
        self.declare_parameter("localizer_search_theta_rad", 0.25)
        self.declare_parameter("localizer_minimum_valid_beams", 20)
        self.declare_parameter("localizer_maximum_match_score_m2", 0.25)
        self.declare_parameter("localizer_max_scan_beams", 90)
        self.declare_parameter("localizer_max_error_m", 0.40)
        self.declare_parameter("localizer_unknown_penalty_m", 0.25)
        self.declare_parameter("localizer_outside_penalty_m", 0.75)
        self.declare_parameter("localizer_position_prior_weight", 0.20)
        self.declare_parameter("localizer_yaw_prior_weight", 0.05)
        self.declare_parameter(
            "localizer_minimum_scan_score_improvement_m2",
            0.001,
        )
        self.declare_parameter(
            "localizer_minimum_objective_improvement_m2",
            0.001,
        )
        self.declare_parameter(
            "localizer_maximum_position_correction_m",
            0.22,
        )
        self.declare_parameter(
            "localizer_maximum_yaw_correction_rad",
            0.16,
        )
        self.declare_parameter(
            "localizer_correction_confirmation_scans",
            2,
        )
        self.declare_parameter(
            "localizer_correction_cooldown_scans",
            2,
        )
        self.declare_parameter(
            "localizer_correction_consistency_position_m",
            0.01,
        )
        self.declare_parameter(
            "localizer_correction_consistency_yaw_rad",
            0.01,
        )
        self.declare_parameter("localizer_rate_hz", 20.0)
        self.declare_parameter("localizer_map_publish_rate_hz", 1.0)
        self.declare_parameter("lidar_x_m", 0.25)
        self.declare_parameter("lidar_y_m", 0.0)
        self.declare_parameter("lidar_yaw_rad", 0.0)

        map_yaml_path = str(parameter_value(self, "map_yaml_path")).strip()
        if not map_yaml_path:
            raise ValueError(
                "map_yaml_path is required; pass the saved map YAML, for example "
                "small_test_area_03.yaml"
            )
        self.occupancy_map = OccupancyMap(map_yaml_path)
        self.localizer = GridMapLocalizer(
            self.occupancy_map,
            xy_step_m=float(parameter_value(self, "localizer_xy_step_m")),
            theta_step_rad=float(
                parameter_value(self, "localizer_theta_step_rad")
            ),
            search_xy_m=float(parameter_value(self, "localizer_search_xy_m")),
            search_theta_rad=float(
                parameter_value(self, "localizer_search_theta_rad")
            ),
            minimum_valid_beams=int(
                parameter_value(self, "localizer_minimum_valid_beams")
            ),
            maximum_match_score_m2=float(
                parameter_value(self, "localizer_maximum_match_score_m2")
            ),
            max_scan_beams=int(
                parameter_value(self, "localizer_max_scan_beams")
            ),
            max_error_m=float(parameter_value(self, "localizer_max_error_m")),
            unknown_penalty_m=float(
                parameter_value(self, "localizer_unknown_penalty_m")
            ),
            outside_penalty_m=float(
                parameter_value(self, "localizer_outside_penalty_m")
            ),
            lidar_x_m=float(parameter_value(self, "lidar_x_m")),
            lidar_y_m=float(parameter_value(self, "lidar_y_m")),
            lidar_yaw_rad=float(parameter_value(self, "lidar_yaw_rad")),
            position_prior_weight=float(
                parameter_value(self, "localizer_position_prior_weight")
            ),
            yaw_prior_weight=float(
                parameter_value(self, "localizer_yaw_prior_weight")
            ),
            minimum_scan_score_improvement_m2=float(parameter_value(
                self,
                "localizer_minimum_scan_score_improvement_m2",
            )),
            minimum_objective_improvement_m2=float(parameter_value(
                self,
                "localizer_minimum_objective_improvement_m2",
            )),
            maximum_position_correction_m=float(parameter_value(
                self,
                "localizer_maximum_position_correction_m",
            )),
            maximum_yaw_correction_rad=float(parameter_value(
                self,
                "localizer_maximum_yaw_correction_rad",
            )),
            correction_confirmation_scans=int(parameter_value(
                self,
                "localizer_correction_confirmation_scans",
            )),
            correction_cooldown_scans=int(parameter_value(
                self,
                "localizer_correction_cooldown_scans",
            )),
            correction_consistency_position_m=float(parameter_value(
                self,
                "localizer_correction_consistency_position_m",
            )),
            correction_consistency_yaw_rad=float(parameter_value(
                self,
                "localizer_correction_consistency_yaw_rad",
            )),
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.map_message = self._make_map_message()

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
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            LaserScan,
            str(parameter_value(self, "scan_topic")),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(parameter_value(self, "odom_topic")),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(parameter_value(self, "initial_pose_topic")),
            self._on_initial_pose,
            10,
        )

        self.latest_scan = None
        self.latest_scan_received_monotonic = None
        self.latest_odom_pose = None
        self.last_odom_pose = None
        self.estimated_pose = self._parameter_initial_pose()
        self.predicted_pose = self.estimated_pose.copy()
        self.last_scan_stamp_s = None
        self.last_processing_time_ms = 0.0
        self.last_state = "waiting_for_odom"
        self.last_match_score = float("inf")
        self.last_map_to_odom = None
        self.processed_scan_stamp_s = None
        self.processed_scan = None
        self.processed_scan_count = 0

        self.map_publisher.publish(self.map_message)
        localizer_rate = max(
            float(parameter_value(self, "localizer_rate_hz")),
            1.0,
        )
        map_rate = max(
            float(parameter_value(self, "localizer_map_publish_rate_hz")),
            0.1,
        )
        self.create_timer(1.0 / localizer_rate, self._process_scan)
        self.create_timer(1.0 / map_rate, self._publish_map)
        self.create_timer(0.10, self._publish_status)
        self.get_logger().info(
            "real map localization loaded "
            f"{self.occupancy_map.width}x{self.occupancy_map.height} cells "
            f"resolution={self.occupancy_map.resolution_m:.3f}m "
            f"map={self.occupancy_map.yaml_path}"
        )

    def _parameter_initial_pose(self):
        return self._make_pose(
            float(parameter_value(self, "initial_yaw_rad")),
            float(parameter_value(self, "initial_x_m")),
            float(parameter_value(self, "initial_y_m")),
        )

    @staticmethod
    def _make_pose(yaw_rad, x_m, y_m):
        return np.array(
            [float(yaw_rad), float(x_m), float(y_m)],
            dtype=float,
        )

    def _on_scan(self, message):
        self.latest_scan = message
        self.latest_scan_received_monotonic = time.monotonic()

    def _on_odom(self, message):
        pose = message.pose.pose
        self.latest_odom_pose = self._make_pose(
            quaternion_to_yaw(pose.orientation),
            float(pose.position.x),
            float(pose.position.y),
        )

    def _on_initial_pose(self, message):
        pose = message.pose.pose
        self.estimated_pose = self._make_pose(
            quaternion_to_yaw(pose.orientation),
            float(pose.position.x),
            float(pose.position.y),
        )
        self.predicted_pose = self.estimated_pose.copy()
        if self.latest_odom_pose is not None:
            self.last_odom_pose = self.latest_odom_pose.copy()
        else:
            self.last_odom_pose = None
        self.localizer.reset_correction_history(reset_counters=True)
        self.last_state = "initial_pose_received"
        self.get_logger().info(
            "received initial pose "
            f"x={self.estimated_pose[1]:.3f} "
            f"y={self.estimated_pose[2]:.3f} "
            f"yaw={self.estimated_pose[0]:.3f}"
        )

    def _make_map_message(self):
        message = OccupancyGrid()
        message.header.frame_id = str(parameter_value(self, "map_frame"))
        message.info = MapMetaData()
        message.info.resolution = float(self.occupancy_map.resolution_m)
        message.info.width = int(self.occupancy_map.width)
        message.info.height = int(self.occupancy_map.height)
        message.info.origin.position.x = float(self.occupancy_map.origin_x_m)
        message.info.origin.position.y = float(self.occupancy_map.origin_y_m)
        qx, qy, qz, qw = yaw_to_quaternion(
            self.occupancy_map.origin_yaw_rad
        )
        message.info.origin.orientation.x = qx
        message.info.origin.orientation.y = qy
        message.info.origin.orientation.z = qz
        message.info.origin.orientation.w = qw
        message.data = [
            int(value)
            for value in self.occupancy_map.occupancy_data_bottom_up.ravel()
        ]
        return message

    def _publish_map(self):
        self.map_message.header.stamp = self.get_clock().now().to_msg()
        self.map_message.info.map_load_time = self.map_message.header.stamp
        self.map_publisher.publish(self.map_message)

    @staticmethod
    def _scan_dict(message):
        return {
            "angle_min": float(message.angle_min),
            "angle_increment": float(message.angle_increment),
            "range_min": float(message.range_min),
            "range_max": float(message.range_max),
            "ranges": message.ranges,
        }

    def _process_scan(self):
        if self.latest_scan is None:
            self.last_state = "waiting_for_scan"
            return
        if self.latest_odom_pose is None:
            self.last_state = "waiting_for_odom"
            return
        stamp_s = stamp_to_seconds(self.latest_scan.header.stamp)
        if self.latest_scan is self.processed_scan:
            return
        self.processed_scan = self.latest_scan
        self.processed_scan_stamp_s = stamp_s
        self.processed_scan_count += 1
        current_odom = self.latest_odom_pose.copy()
        if self.last_odom_pose is None:
            self.last_odom_pose = current_odom.copy()
            self.predicted_pose = self.estimated_pose.copy()
        else:
            delta = odom_delta(self.last_odom_pose, current_odom)
            self.predicted_pose = _compose_pose(self.predicted_pose, delta)
            self.last_odom_pose = current_odom.copy()

        started = time.perf_counter()
        estimate, score = self.localizer.estimate(
            self._scan_dict(self.latest_scan),
            self.predicted_pose,
        )
        self.last_processing_time_ms = 1000.0 * (
            time.perf_counter() - started
        )
        self.last_scan_stamp_s = stamp_s
        self.last_match_score = float(score)
        if not self.localizer.last_success:
            self.last_state = "match_failed"
            return

        self.estimated_pose = estimate
        self.predicted_pose = estimate.copy()
        self.last_state = "localized"
        self._publish_pose(self.latest_scan, estimate, score)
        self._update_map_to_odom(estimate, current_odom)

    def _publish_pose(self, scan_message, pose, score):
        message = PoseWithCovarianceStamped()
        message.header = scan_message.header
        message.header.frame_id = str(parameter_value(self, "map_frame"))
        message.pose.pose.position.x = float(pose[1])
        message.pose.pose.position.y = float(pose[2])
        qx, qy, qz, qw = yaw_to_quaternion(float(pose[0]))
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        variance = max(float(score), 1e-6)
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[35] = variance
        self.pose_publisher.publish(message)

    def _update_map_to_odom(self, map_base, odom_base):
        self.last_map_to_odom = _compose_pose(
            map_base,
            _inverse_pose(odom_base),
        )
        self._publish_map_to_odom()

    def _publish_map_to_odom(self):
        if self.last_map_to_odom is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(parameter_value(self, "map_frame"))
        transform.child_frame_id = str(parameter_value(self, "odom_frame"))
        transform.transform.translation.x = float(self.last_map_to_odom[1])
        transform.transform.translation.y = float(self.last_map_to_odom[2])
        qx, qy, qz, qw = yaw_to_quaternion(float(self.last_map_to_odom[0]))
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _publish_status(self):
        self._publish_map_to_odom()
        now_s = self.get_clock().now().nanoseconds * 1e-9
        age_s = (
            float("inf")
            if self.last_scan_stamp_s is None
            else now_s - self.last_scan_stamp_s
        )
        timeout_s = float(parameter_value(self, "scan_timeout_s"))
        state = self.last_state
        if self.last_scan_stamp_s is not None and age_s > timeout_s:
            state = "scan_timeout"
        if state == "localized":
            level = DiagnosticStatus.OK
        elif state in ("waiting_for_scan", "waiting_for_odom"):
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "real_vehicle_localization"
        status.hardware_id = f"{self.occupancy_map.yaml_path.stem}_map"
        status.message = state
        status.values = [
            KeyValue(key="map_yaml", value=str(self.occupancy_map.yaml_path)),
            KeyValue(key="match_score_m2", value=f"{self.last_match_score:.9f}"),
            KeyValue(
                key="predicted_match_score_m2",
                value=f"{self.localizer.last_predicted_score:.9f}",
            ),
            KeyValue(
                key="objective_score_m2",
                value=f"{self.localizer.last_objective_score:.9f}",
            ),
            KeyValue(
                key="prior_cost_m2",
                value=f"{self.localizer.last_prior_cost:.9f}",
            ),
            KeyValue(
                key="scan_improvement_m2",
                value=f"{self.localizer.last_scan_improvement_m2:.9f}",
            ),
            KeyValue(
                key="objective_improvement_m2",
                value=(
                    f"{self.localizer.last_objective_improvement_m2:.9f}"
                ),
            ),
            KeyValue(
                key="selection_reason",
                value=self.localizer.last_selection_reason,
            ),
            KeyValue(
                key="position_correction_m",
                value=f"{self.localizer.last_position_correction_m:.6f}",
            ),
            KeyValue(
                key="yaw_correction_rad",
                value=f"{self.localizer.last_yaw_correction_rad:.6f}",
            ),
            KeyValue(
                key="correction_dx_m",
                value=f"{self.localizer.last_correction_dx_m:.6f}",
            ),
            KeyValue(
                key="correction_dy_m",
                value=f"{self.localizer.last_correction_dy_m:.6f}",
            ),
            KeyValue(
                key="correction_dyaw_rad",
                value=f"{self.localizer.last_correction_dyaw_rad:.6f}",
            ),
            KeyValue(
                key="proposed_correction_dx_m",
                value=(
                    f"{self.localizer.last_proposed_correction_dx_m:.6f}"
                ),
            ),
            KeyValue(
                key="proposed_correction_dy_m",
                value=(
                    f"{self.localizer.last_proposed_correction_dy_m:.6f}"
                ),
            ),
            KeyValue(
                key="proposed_correction_dyaw_rad",
                value=(
                    f"{self.localizer.last_proposed_correction_dyaw_rad:.6f}"
                ),
            ),
            KeyValue(
                key="correction_event_id",
                value=str(self.localizer.correction_event_id),
            ),
            KeyValue(
                key="correction_proposal_count",
                value=str(self.localizer.correction_proposal_count),
            ),
            KeyValue(
                key="correction_pending_count",
                value=str(self.localizer.correction_pending_count),
            ),
            KeyValue(
                key="correction_rejection_count",
                value=str(self.localizer.correction_rejection_count),
            ),
            KeyValue(
                key="pending_reset_count",
                value=str(self.localizer.pending_reset_count),
            ),
            KeyValue(
                key="confirmation_count",
                value=str(self.localizer.last_confirmation_count),
            ),
            KeyValue(
                key="confirmation_required",
                value=str(self.localizer.correction_confirmation_scans),
            ),
            KeyValue(
                key="cooldown_scans_remaining",
                value=str(self.localizer.last_cooldown_scans_remaining),
            ),
            KeyValue(
                key="pending_correction_dx_m",
                value=f"{self.localizer.pending_correction_dx_m:.6f}",
            ),
            KeyValue(
                key="pending_correction_dy_m",
                value=f"{self.localizer.pending_correction_dy_m:.6f}",
            ),
            KeyValue(
                key="pending_correction_dyaw_rad",
                value=(
                    f"{self.localizer.pending_correction_dyaw_rad:.6f}"
                ),
            ),
            KeyValue(
                key="valid_beams",
                value=str(self.localizer.last_valid_beams),
            ),
            KeyValue(key="candidate_count", value=str(
                self.localizer.last_candidate_count
            )),
            KeyValue(
                key="processed_scan_count",
                value=str(self.processed_scan_count),
            ),
            KeyValue(
                key="processed_scan_stamp_s",
                value=(
                    "nan"
                    if self.processed_scan_stamp_s is None
                    else f"{self.processed_scan_stamp_s:.9f}"
                ),
            ),
            KeyValue(key="scan_age_s", value=f"{age_s:.6f}"),
            KeyValue(
                key="processing_time_ms",
                value=f"{self.last_processing_time_ms:.3f}",
            ),
            KeyValue(
                key="estimated_pose_x_m",
                value=f"{self.estimated_pose[1]:.6f}",
            ),
            KeyValue(
                key="estimated_pose_y_m",
                value=f"{self.estimated_pose[2]:.6f}",
            ),
            KeyValue(
                key="estimated_pose_yaw_rad",
                value=f"{self.estimated_pose[0]:.6f}",
            ),
        ]
        message.status = [status]
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealMapLocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
