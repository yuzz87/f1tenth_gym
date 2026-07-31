"""Run virtual sensors, safety, and dry-run actuator on the Raspberry Pi side."""

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
    reference_type = LaunchConfiguration("reference_type")
    return LaunchDescription([
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("reference_type", default_value="straight"),
        DeclareLaunchArgument("control_delay_s", default_value="0.0"),
        DeclareLaunchArgument("control_delay_jitter_s", default_value="0.0"),
        DeclareLaunchArgument("control_dropout_probability", default_value="0.0"),
        DeclareLaunchArgument("control_burst_start_probability", default_value="0.0"),
        DeclareLaunchArgument("control_burst_length_messages", default_value="1"),
        DeclareLaunchArgument("control_outage_after_s", default_value="-1.0"),
        DeclareLaunchArgument("control_outage_duration_s", default_value="0.0"),
        DeclareLaunchArgument("control_duplicate_probability", default_value="0.0"),
        DeclareLaunchArgument("control_stale_replay_probability", default_value="0.0"),
        DeclareLaunchArgument("control_minimum_publish_period_s", default_value="0.0"),
        DeclareLaunchArgument("fault_seed", default_value="123"),
        DeclareLaunchArgument("start_mock_vehicle", default_value="true"),
        Node(
            package="real_vehicle_integration",
            executable="mock_vehicle_node",
            name="mock_vehicle",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_mock_vehicle")),
            parameters=[config, {
                "scan_topic": "/transport/scan",
                "odom_topic": "/transport/odom",
                "reference_type": reference_type,
            }],
        ),
        Node(
            package="real_vehicle_integration",
            executable="network_fault_injector",
            name="pi_control_fault_injector",
            output="screen",
            parameters=[{
                "relay_control": True,
                "control_input_topic": "/transport/control_request",
                "control_output_topic": "/vehicle/control_request",
                "control_delay_s": LaunchConfiguration("control_delay_s"),
                "control_delay_jitter_s": LaunchConfiguration(
                    "control_delay_jitter_s"
                ),
                "control_dropout_probability": LaunchConfiguration(
                    "control_dropout_probability"
                ),
                "control_burst_start_probability": LaunchConfiguration(
                    "control_burst_start_probability"
                ),
                "control_burst_length_messages": LaunchConfiguration(
                    "control_burst_length_messages"
                ),
                "control_outage_after_s": LaunchConfiguration(
                    "control_outage_after_s"
                ),
                "control_outage_duration_s": LaunchConfiguration(
                    "control_outage_duration_s"
                ),
                "control_duplicate_probability": LaunchConfiguration(
                    "control_duplicate_probability"
                ),
                "control_stale_replay_probability": LaunchConfiguration(
                    "control_stale_replay_probability"
                ),
                "control_minimum_publish_period_s": LaunchConfiguration(
                    "control_minimum_publish_period_s"
                ),
                "fault_seed": LaunchConfiguration("fault_seed"),
            }],
        ),
        Node(
            package="real_vehicle_integration",
            executable="safety_node",
            name="safety",
            output="screen",
            parameters=[config, {"auto_arm": auto_arm, "auto_start": auto_start}],
        ),
        Node(
            package="real_vehicle_integration",
            executable="actuator_node",
            name="actuator",
            output="screen",
            parameters=[config, {
                "hardware_output_enabled": False,
                "pwm_backend": "mock",
            }],
        ),
    ])
