"""Run the PC-side encoder UDP bridge and model-based odometry."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("real_vehicle_integration"))
    config = str(share / "config/connection.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("encoder_udp_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("encoder_udp_port", default_value="5011"),
        DeclareLaunchArgument("encoder_udp_allowed_host", default_value=""),
        Node(
            package="real_vehicle_integration",
            executable="encoder_odom_node",
            name="encoder_odom",
            output="screen",
            parameters=[config, {
                "encoder_udp_bind_host": LaunchConfiguration(
                    "encoder_udp_bind_host"
                ),
                "encoder_udp_port": LaunchConfiguration("encoder_udp_port"),
                "encoder_udp_allowed_host": LaunchConfiguration(
                    "encoder_udp_allowed_host"
                ),
            }],
        ),
    ])

