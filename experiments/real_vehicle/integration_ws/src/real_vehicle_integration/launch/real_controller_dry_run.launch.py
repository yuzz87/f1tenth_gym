"""Run real sensors through MPC/MPPI with no actuator output path."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from real_vehicle_integration.repository import find_repository_root


def generate_launch_description():
    share = Path(get_package_share_directory("real_vehicle_integration"))
    config = str(share / "config/connection.yaml")
    localization_launch = str(
        share / "launch/real_localization.launch.py"
    )
    repository_root = find_repository_root()
    default_map = os.environ.get(
        "REAL_MAP_YAML",
        str(
            repository_root
            / "experiments/real_vehicle/maps/small_test_area_03/"
            "small_test_area_03.yaml"
        ),
    )

    controller_type = LaunchConfiguration("controller_type")
    reference_type = LaunchConfiguration("reference_type")
    reference_speed_mps = LaunchConfiguration("reference_speed_mps")
    pose_timeout_s = LaunchConfiguration("pose_timeout_s")
    scan_timeout_s = LaunchConfiguration("scan_timeout_s")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=default_map),
        DeclareLaunchArgument(
            "lidar_udp_allowed_host",
            default_value="",
        ),
        DeclareLaunchArgument(
            "encoder_udp_allowed_host",
            default_value="",
        ),
        DeclareLaunchArgument("lidar_udp_port", default_value="5010"),
        DeclareLaunchArgument("encoder_udp_port", default_value="5011"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("controller_type", default_value="mppi"),
        DeclareLaunchArgument("reference_type", default_value="straight"),
        DeclareLaunchArgument(
            "reference_speed_mps",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "pose_timeout_s",
            # The real localizer follows a 7-8 Hz LiDAR stream.  Keep this
            # tolerance local to the dry-run; hardware safety remains 0.20 s.
            default_value="0.35",
        ),
        DeclareLaunchArgument(
            "scan_timeout_s",
            # The real A1 stream is about 7-8 Hz; allow Wi-Fi/jitter in the
            # dry-run while keeping the hardware safety setting unchanged.
            default_value="0.50",
        ),
        DeclareLaunchArgument("fault_mode", default_value="none"),
        DeclareLaunchArgument("auto_arm", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("initial_x_m", default_value="0.0"),
        DeclareLaunchArgument("initial_y_m", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw_rad", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "map_yaml": LaunchConfiguration("map_yaml"),
                "lidar_udp_allowed_host": LaunchConfiguration(
                    "lidar_udp_allowed_host"
                ),
                "encoder_udp_allowed_host": LaunchConfiguration(
                    "encoder_udp_allowed_host"
                ),
                "lidar_udp_port": LaunchConfiguration("lidar_udp_port"),
                "encoder_udp_port": LaunchConfiguration(
                    "encoder_udp_port"
                ),
                "start_rviz": LaunchConfiguration("start_rviz"),
                "start_direction_check": "false",
                "initial_x_m": LaunchConfiguration("initial_x_m"),
                "initial_y_m": LaunchConfiguration("initial_y_m"),
                "initial_yaw_rad": LaunchConfiguration(
                    "initial_yaw_rad"
                ),
            }.items(),
        ),
        Node(
            package="real_vehicle_integration",
            executable="dry_run_fault_injector",
            name="dry_run_fault_injector",
            output="screen",
            parameters=[{
                "fault_mode": LaunchConfiguration("fault_mode"),
                "scan_input_topic": "/scan",
                "scan_output_topic": "/dry_run/scan",
                "pose_input_topic": "/localization/pose",
                "pose_output_topic": "/dry_run/pose",
                "control_input_topic": "/dry_run/control_raw",
                "control_output_topic": "/dry_run/control_request",
                "emergency_stop_topic": "/dry_run/emergency_stop",
                "status_topic": "/dry_run/fault_injector_status",
            }],
        ),
        Node(
            package="real_vehicle_integration",
            executable="controller_node",
            name="controller",
            output="screen",
            parameters=[config, {
                "controller_type": controller_type,
                "reference_type": reference_type,
                "reference_speed_mps": reference_speed_mps,
                "pose_topic": "/dry_run/pose",
                "control_request_topic": "/dry_run/control_raw",
                "controller_status_topic": "/dry_run/controller_status",
            }],
        ),
        Node(
            package="real_vehicle_integration",
            executable="safety_node",
            name="safety",
            output="screen",
            parameters=[config, {
                "auto_arm": auto_arm,
                "auto_start": auto_start,
                "pose_timeout_s": pose_timeout_s,
                "scan_timeout_s": scan_timeout_s,
                "scan_topic": "/dry_run/scan",
                "pose_topic": "/dry_run/pose",
                "control_request_topic": "/dry_run/control_request",
                "control_safe_topic": "/dry_run/control_safe",
                "safety_status_topic": "/dry_run/safety_status",
                "emergency_stop_topic": "/dry_run/emergency_stop",
            }],
        ),
        Node(
            package="real_vehicle_integration",
            executable="dry_run_guard",
            name="dry_run_guard",
            output="screen",
            parameters=[config, {
                "control_safe_topic": "/dry_run/control_safe",
                "emergency_stop_topic": "/dry_run/emergency_stop",
                "status_topic": "/dry_run/hardware_guard_status",
            }],
        ),
    ])
