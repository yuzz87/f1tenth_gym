"""Receive the standalone Pi LiDAR agent and optionally open RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("real_vehicle_integration"))
    config = str(share / "config/connection.yaml")
    rviz_config = str(share / "rviz/real_lidar.rviz")
    bind_host = LaunchConfiguration("lidar_udp_bind_host")
    allowed_host = LaunchConfiguration("lidar_udp_allowed_host")
    port = LaunchConfiguration("lidar_udp_port")
    start_direction_check = LaunchConfiguration("start_direction_check")
    start_rviz = LaunchConfiguration("start_rviz")
    publish_static_tf = LaunchConfiguration("publish_static_tf")
    start_localization = LaunchConfiguration("start_localization")
    start_encoder_odom = LaunchConfiguration("start_encoder_odom")
    encoder_bind_host = LaunchConfiguration("encoder_udp_bind_host")
    encoder_allowed_host = LaunchConfiguration("encoder_udp_allowed_host")
    encoder_port = LaunchConfiguration("encoder_udp_port")
    lidar_x = LaunchConfiguration("lidar_x_m")
    lidar_y = LaunchConfiguration("lidar_y_m")
    lidar_z = LaunchConfiguration("lidar_z_m")
    lidar_yaw = LaunchConfiguration("lidar_yaw_rad")
    return LaunchDescription([
        DeclareLaunchArgument("lidar_udp_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("lidar_udp_allowed_host", default_value=""),
        DeclareLaunchArgument("lidar_udp_port", default_value="5010"),
        DeclareLaunchArgument("start_direction_check", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("publish_static_tf", default_value="true"),
        DeclareLaunchArgument("start_localization", default_value="false"),
        DeclareLaunchArgument("start_encoder_odom", default_value="false"),
        DeclareLaunchArgument("encoder_udp_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("encoder_udp_allowed_host", default_value=""),
        DeclareLaunchArgument("encoder_udp_port", default_value="5011"),
        DeclareLaunchArgument("lidar_x_m", default_value="0.25"),
        DeclareLaunchArgument("lidar_y_m", default_value="0.0"),
        DeclareLaunchArgument("lidar_z_m", default_value="0.11"),
        DeclareLaunchArgument("lidar_yaw_rad", default_value="0.0"),
        Node(
            package="real_vehicle_integration",
            executable="udp_lidar_bridge",
            name="udp_lidar_bridge",
            output="screen",
            parameters=[config, {
                "lidar_udp_bind_host": bind_host,
                "lidar_udp_allowed_host": allowed_host,
                "lidar_udp_port": port,
            }],
        ),
        Node(
            package="real_vehicle_integration",
            executable="lidar_direction_check",
            name="lidar_direction_check",
            output="screen",
            condition=IfCondition(start_direction_check),
            parameters=[config],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            output="screen",
            condition=IfCondition(publish_static_tf),
            arguments=[
                lidar_x,
                lidar_y,
                lidar_z,
                lidar_yaw,
                "0.0",
                "0.0",
                "base_link",
                "laser",
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="real_lidar_rviz",
            output="screen",
            condition=IfCondition(start_rviz),
            arguments=["-d", rviz_config],
        ),
        Node(
            package="real_vehicle_integration",
            executable="localization_node",
            name="localization",
            output="screen",
            condition=IfCondition(start_localization),
            parameters=[config],
        ),
        Node(
            package="real_vehicle_integration",
            executable="encoder_odom_node",
            name="encoder_odom",
            output="screen",
            condition=IfCondition(start_encoder_odom),
            parameters=[config, {
                "encoder_udp_bind_host": encoder_bind_host,
                "encoder_udp_allowed_host": encoder_allowed_host,
                "encoder_udp_port": encoder_port,
            }],
        ),
    ])
