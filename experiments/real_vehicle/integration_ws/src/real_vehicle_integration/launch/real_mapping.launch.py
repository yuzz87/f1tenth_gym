"""Build an occupancy map from the real Pi LiDAR and encoder streams."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("real_vehicle_integration"))
    connection_config = str(share / "config/connection.yaml")
    default_slam_config = str(
        share / "config/mapper_params_real_vehicle.yaml"
    )
    default_rviz_config = str(share / "rviz/real_mapping.rviz")

    lidar_allowed_host = LaunchConfiguration("lidar_udp_allowed_host")
    encoder_allowed_host = LaunchConfiguration("encoder_udp_allowed_host")
    start_direction_check = LaunchConfiguration("start_direction_check")
    start_rviz = LaunchConfiguration("start_rviz")
    slam_params_file = LaunchConfiguration("slam_params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    return LaunchDescription([
        DeclareLaunchArgument("lidar_udp_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("lidar_udp_allowed_host", default_value=""),
        DeclareLaunchArgument("lidar_udp_port", default_value="5010"),
        DeclareLaunchArgument("encoder_udp_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("encoder_udp_allowed_host", default_value=""),
        DeclareLaunchArgument("encoder_udp_port", default_value="5011"),
        DeclareLaunchArgument("start_direction_check", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("lidar_x_m", default_value="0.25"),
        DeclareLaunchArgument("lidar_y_m", default_value="0.0"),
        DeclareLaunchArgument("lidar_z_m", default_value="0.11"),
        DeclareLaunchArgument("lidar_yaw_rad", default_value="0.0"),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_config,
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=default_rviz_config,
        ),
        Node(
            package="real_vehicle_integration",
            executable="udp_lidar_bridge",
            name="udp_lidar_bridge",
            output="screen",
            parameters=[connection_config, {
                "lidar_udp_bind_host": LaunchConfiguration(
                    "lidar_udp_bind_host"
                ),
                "lidar_udp_allowed_host": lidar_allowed_host,
                "lidar_udp_port": LaunchConfiguration("lidar_udp_port"),
            }],
        ),
        Node(
            package="real_vehicle_integration",
            executable="encoder_odom_node",
            name="encoder_odom",
            output="screen",
            parameters=[connection_config, {
                "encoder_udp_bind_host": LaunchConfiguration(
                    "encoder_udp_bind_host"
                ),
                "encoder_udp_allowed_host": encoder_allowed_host,
                "encoder_udp_port": LaunchConfiguration("encoder_udp_port"),
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            output="screen",
            arguments=[
                LaunchConfiguration("lidar_x_m"),
                LaunchConfiguration("lidar_y_m"),
                LaunchConfiguration("lidar_z_m"),
                LaunchConfiguration("lidar_yaw_rad"),
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
            condition=IfCondition(start_direction_check),
            parameters=[connection_config],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params_file, {"use_sim_time": False}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="real_mapping_rviz",
            output="screen",
            condition=IfCondition(start_rviz),
            arguments=["-d", rviz_config_file],
        ),
    ])
