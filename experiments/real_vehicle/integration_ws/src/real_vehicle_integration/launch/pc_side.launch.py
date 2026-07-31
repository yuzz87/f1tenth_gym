"""Run localization and control on the PC side of the ROS 2 network."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("real_vehicle_integration"))
    config = str(share / "config/connection.yaml")
    controller_type = LaunchConfiguration("controller_type")
    reference_type = LaunchConfiguration("reference_type")
    return LaunchDescription([
        DeclareLaunchArgument("controller_type", default_value="mppi"),
        DeclareLaunchArgument("reference_type", default_value="straight"),
        DeclareLaunchArgument("scan_delay_s", default_value="0.0"),
        DeclareLaunchArgument("scan_delay_jitter_s", default_value="0.0"),
        DeclareLaunchArgument("scan_dropout_probability", default_value="0.0"),
        DeclareLaunchArgument("scan_burst_start_probability", default_value="0.0"),
        DeclareLaunchArgument("scan_burst_length_messages", default_value="1"),
        DeclareLaunchArgument("scan_outage_after_s", default_value="-1.0"),
        DeclareLaunchArgument("scan_outage_duration_s", default_value="0.0"),
        DeclareLaunchArgument("scan_minimum_publish_period_s", default_value="0.0"),
        DeclareLaunchArgument("odom_delay_s", default_value="0.0"),
        DeclareLaunchArgument("odom_dropout_probability", default_value="0.0"),
        DeclareLaunchArgument("fault_seed", default_value="123"),
        Node(
            package="real_vehicle_integration",
            executable="network_fault_injector",
            name="pc_sensor_fault_injector",
            output="screen",
            parameters=[{
                "relay_scan": True,
                "relay_odom": True,
                "scan_input_topic": "/transport/scan",
                "scan_output_topic": "/scan",
                "odom_input_topic": "/transport/odom",
                "odom_output_topic": "/odom",
                "scan_delay_s": LaunchConfiguration("scan_delay_s"),
                "scan_delay_jitter_s": LaunchConfiguration("scan_delay_jitter_s"),
                "scan_dropout_probability": LaunchConfiguration(
                    "scan_dropout_probability"
                ),
                "scan_burst_start_probability": LaunchConfiguration(
                    "scan_burst_start_probability"
                ),
                "scan_burst_length_messages": LaunchConfiguration(
                    "scan_burst_length_messages"
                ),
                "scan_outage_after_s": LaunchConfiguration("scan_outage_after_s"),
                "scan_outage_duration_s": LaunchConfiguration(
                    "scan_outage_duration_s"
                ),
                "scan_minimum_publish_period_s": LaunchConfiguration(
                    "scan_minimum_publish_period_s"
                ),
                "odom_delay_s": LaunchConfiguration("odom_delay_s"),
                "odom_dropout_probability": LaunchConfiguration(
                    "odom_dropout_probability"
                ),
                "fault_seed": LaunchConfiguration("fault_seed"),
            }],
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
            parameters=[config, {
                "controller_type": controller_type,
                "reference_type": reference_type,
                "control_request_topic": "/transport/control_request",
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
