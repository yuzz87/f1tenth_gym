"""Launch controller integration without starting an RPLIDAR driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("real_vehicle_integration"))
    config = str(share / "config/connection.yaml")
    controller_type = LaunchConfiguration("controller_type")
    reference_type = LaunchConfiguration("reference_type")
    overrides = {
        "controller_type": controller_type,
        "reference_type": reference_type,
        "hardware_output_enabled": False,
    }
    return LaunchDescription([
        DeclareLaunchArgument("controller_type", default_value="mppi"),
        DeclareLaunchArgument("reference_type", default_value="straight"),
        Node(
            package="real_vehicle_integration",
            executable="localization_node",
            name="localization",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="real_vehicle_integration",
            executable="controller_node",
            name="controller",
            output="screen",
            parameters=[config, overrides],
        ),
        Node(
            package="real_vehicle_integration",
            executable="safety_node",
            name="safety",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="real_vehicle_integration",
            executable="actuator_node",
            name="actuator",
            output="screen",
            parameters=[config, overrides],
        ),
        Node(
            package="real_vehicle_integration",
            executable="integration_logger",
            name="integration_logger",
            output="screen",
            parameters=[config],
        ),
    ])
