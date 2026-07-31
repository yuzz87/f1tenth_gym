"""Run the real LiDAR/encoder transport and a saved-map localizer."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from real_vehicle_integration.repository import find_repository_root


def generate_launch_description():
    share = Path(get_package_share_directory("real_vehicle_integration"))
    config = str(share / "config/connection.yaml")
    rviz_config = str(share / "rviz/real_localization.rviz")
    repository_root = find_repository_root()
    default_map = os.environ.get(
        "REAL_MAP_YAML",
        str(
            repository_root
            / "experiments/real_vehicle/maps/small_test_area_03/"
            "small_test_area_03.yaml"
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=default_map),
        DeclareLaunchArgument("lidar_udp_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("lidar_udp_allowed_host", default_value=""),
        DeclareLaunchArgument("lidar_udp_port", default_value="5010"),
        DeclareLaunchArgument("encoder_udp_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("encoder_udp_allowed_host", default_value=""),
        DeclareLaunchArgument("encoder_udp_port", default_value="5011"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_direction_check", default_value="false"),
        DeclareLaunchArgument("initial_x_m", default_value="0.0"),
        DeclareLaunchArgument("initial_y_m", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw_rad", default_value="0.0"),
        DeclareLaunchArgument("localizer_xy_step_m", default_value="0.15"),
        DeclareLaunchArgument(
            "localizer_theta_step_rad",
            default_value="0.10",
        ),
        DeclareLaunchArgument("localizer_search_xy_m", default_value="0.30"),
        DeclareLaunchArgument(
            "localizer_search_theta_rad",
            default_value="0.25",
        ),
        DeclareLaunchArgument("localizer_max_scan_beams", default_value="90"),
        DeclareLaunchArgument(
            "localizer_position_prior_weight",
            default_value="0.20",
        ),
        DeclareLaunchArgument(
            "localizer_yaw_prior_weight",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "localizer_minimum_scan_score_improvement_m2",
            default_value="0.001",
        ),
        DeclareLaunchArgument(
            "localizer_minimum_objective_improvement_m2",
            default_value="0.001",
        ),
        DeclareLaunchArgument(
            "localizer_maximum_position_correction_m",
            default_value="0.22",
        ),
        DeclareLaunchArgument(
            "localizer_maximum_yaw_correction_rad",
            default_value="0.16",
        ),
        DeclareLaunchArgument(
            "localizer_correction_confirmation_scans",
            default_value="2",
        ),
        DeclareLaunchArgument(
            "localizer_correction_cooldown_scans",
            default_value="2",
        ),
        DeclareLaunchArgument(
            "localizer_correction_consistency_position_m",
            default_value="0.01",
        ),
        DeclareLaunchArgument(
            "localizer_correction_consistency_yaw_rad",
            default_value="0.01",
        ),
        Node(
            package="real_vehicle_integration",
            executable="udp_lidar_bridge",
            name="udp_lidar_bridge",
            output="screen",
            parameters=[config, {
                "lidar_udp_bind_host": LaunchConfiguration(
                    "lidar_udp_bind_host"
                ),
                "lidar_udp_allowed_host": LaunchConfiguration(
                    "lidar_udp_allowed_host"
                ),
                "lidar_udp_port": LaunchConfiguration("lidar_udp_port"),
            }],
        ),
        Node(
            package="real_vehicle_integration",
            executable="encoder_odom_node",
            name="encoder_odom",
            output="screen",
            parameters=[config, {
                "encoder_udp_bind_host": LaunchConfiguration(
                    "encoder_udp_bind_host"
                ),
                "encoder_udp_allowed_host": LaunchConfiguration(
                    "encoder_udp_allowed_host"
                ),
                "encoder_udp_port": LaunchConfiguration("encoder_udp_port"),
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            output="screen",
            arguments=[
                "0.25",
                "0.0",
                "0.11",
                "0.0",
                "0.0",
                "0.0",
                "base_link",
                "laser",
            ],
        ),
        Node(
            package="real_vehicle_integration",
            executable="lidar_direction_check",
            name="lidar_direction_check",
            output="screen",
            condition=IfCondition(
                LaunchConfiguration("start_direction_check")
            ),
            parameters=[config],
        ),
        Node(
            package="real_vehicle_integration",
            executable="real_map_localization_node",
            name="real_map_localization",
            output="screen",
            parameters=[config, {
                "map_yaml_path": LaunchConfiguration("map_yaml"),
                "initial_x_m": LaunchConfiguration("initial_x_m"),
                "initial_y_m": LaunchConfiguration("initial_y_m"),
                "initial_yaw_rad": LaunchConfiguration("initial_yaw_rad"),
                "localizer_xy_step_m": LaunchConfiguration(
                    "localizer_xy_step_m"
                ),
                "localizer_theta_step_rad": LaunchConfiguration(
                    "localizer_theta_step_rad"
                ),
                "localizer_search_xy_m": LaunchConfiguration(
                    "localizer_search_xy_m"
                ),
                "localizer_search_theta_rad": LaunchConfiguration(
                    "localizer_search_theta_rad"
                ),
                "localizer_max_scan_beams": LaunchConfiguration(
                    "localizer_max_scan_beams"
                ),
                "localizer_position_prior_weight": LaunchConfiguration(
                    "localizer_position_prior_weight"
                ),
                "localizer_yaw_prior_weight": LaunchConfiguration(
                    "localizer_yaw_prior_weight"
                ),
                "localizer_minimum_scan_score_improvement_m2":
                    LaunchConfiguration(
                        "localizer_minimum_scan_score_improvement_m2"
                    ),
                "localizer_minimum_objective_improvement_m2":
                    LaunchConfiguration(
                        "localizer_minimum_objective_improvement_m2"
                    ),
                "localizer_maximum_position_correction_m":
                    LaunchConfiguration(
                        "localizer_maximum_position_correction_m"
                    ),
                "localizer_maximum_yaw_correction_rad":
                    LaunchConfiguration(
                        "localizer_maximum_yaw_correction_rad"
                    ),
                "localizer_correction_confirmation_scans":
                    LaunchConfiguration(
                        "localizer_correction_confirmation_scans"
                    ),
                "localizer_correction_cooldown_scans":
                    LaunchConfiguration(
                        "localizer_correction_cooldown_scans"
                    ),
                "localizer_correction_consistency_position_m":
                    LaunchConfiguration(
                        "localizer_correction_consistency_position_m"
                    ),
                "localizer_correction_consistency_yaw_rad":
                    LaunchConfiguration(
                        "localizer_correction_consistency_yaw_rad"
                    ),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="real_localization_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            arguments=["-d", rviz_config],
        ),
    ])
