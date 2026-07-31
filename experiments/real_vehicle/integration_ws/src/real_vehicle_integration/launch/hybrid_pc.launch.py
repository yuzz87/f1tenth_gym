"""Run ROS 2 on the PC and a standalone UDP actuator agent on the Pi."""

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
    auto_arm = LaunchConfiguration("auto_arm")
    auto_start = LaunchConfiguration("auto_start")
    controller_type = LaunchConfiguration("controller_type")
    reference_type = LaunchConfiguration("reference_type")
    reference_speed_mps = LaunchConfiguration("reference_speed_mps")
    pi_udp_host = LaunchConfiguration("pi_udp_host")
    allow_uncalibrated = LaunchConfiguration(
        "allow_uncalibrated_speed_to_duty"
    )
    start_mock_vehicle = LaunchConfiguration("start_mock_vehicle")
    common = {
        "auto_arm": auto_arm,
        "auto_start": auto_start,
        "controller_type": controller_type,
        "reference_type": reference_type,
        "reference_speed_mps": reference_speed_mps,
    }
    return LaunchDescription([
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("controller_type", default_value="mppi"),
        DeclareLaunchArgument("reference_type", default_value="straight"),
        DeclareLaunchArgument("reference_speed_mps", default_value="0.05"),
        DeclareLaunchArgument("pi_udp_host", default_value="raspberrypi.local"),
        DeclareLaunchArgument("start_mock_vehicle", default_value="true"),
        DeclareLaunchArgument(
            "allow_uncalibrated_speed_to_duty",
            default_value="false",
        ),
        Node(
            package="real_vehicle_integration",
            executable="mock_vehicle_node",
            name="mock_vehicle",
            output="screen",
            condition=IfCondition(start_mock_vehicle),
            parameters=[config, common],
        ),
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
            parameters=[config, common],
        ),
        Node(
            package="real_vehicle_integration",
            executable="safety_node",
            name="safety",
            output="screen",
            parameters=[config, common],
        ),
        Node(
            package="real_vehicle_integration",
            executable="udp_actuator_bridge",
            name="udp_actuator_bridge",
            output="screen",
            parameters=[config, {
                "pi_udp_host": pi_udp_host,
                "allow_uncalibrated_speed_to_duty": allow_uncalibrated,
            }],
        ),
        Node(
            package="real_vehicle_integration",
            executable="integration_logger",
            name="integration_logger",
            output="screen",
            parameters=[config],
        ),
    ])
