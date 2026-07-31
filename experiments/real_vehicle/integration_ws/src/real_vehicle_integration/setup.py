from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "real_vehicle_integration"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/config", glob("config/*.yaml")),
        ("share/" + PACKAGE_NAME + "/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE_NAME + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntuyuzz",
    maintainer_email="ubuntuyuzz@example.com",
    description="Safe ROS 2 integration for the Python RC-car controllers.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "actuator_node = real_vehicle_integration.nodes.actuator_node:main",
            "controller_node = real_vehicle_integration.nodes.controller_node:main",
            "diagnostic_check = real_vehicle_integration.nodes.diagnostic_check:main",
            "controller_dry_run_trial = "
            "real_vehicle_integration.nodes.controller_dry_run_trial:main",
            "controller_dry_run_compare = "
            "real_vehicle_integration.controller_dry_run_compare:main",
            "dry_run_fault_injector = "
            "real_vehicle_integration.nodes.dry_run_fault_injector:main",
            "dry_run_guard = "
            "real_vehicle_integration.nodes.dry_run_guard:main",
            "integration_logger = real_vehicle_integration.nodes.integration_logger:main",
            "lidar_direction_check = "
            "real_vehicle_integration.nodes.lidar_direction_check:main",
            "lidar_udp_test_sender = "
            "real_vehicle_integration.nodes.lidar_udp_test_sender:main",
            "localization_node = real_vehicle_integration.nodes.localization_node:main",
            "localization_motion_trial = "
            "real_vehicle_integration.nodes.localization_motion_trial:main",
            "localization_replay = "
            "real_vehicle_integration.localization_replay:main",
            "localization_replay_batch = "
            "real_vehicle_integration.localization_replay_batch:main",
            "localization_rotation_trial = "
            "real_vehicle_integration.nodes.localization_rotation_trial:main",
            "localization_static_trial = "
            "real_vehicle_integration.nodes.localization_static_trial:main",
            "real_map_localization_node = "
            "real_vehicle_integration.nodes.real_map_localization_node:main",
            "mapping_readiness_check = "
            "real_vehicle_integration.nodes.mapping_readiness_check:main",
            "mock_vehicle_node = real_vehicle_integration.nodes.mock_vehicle_node:main",
            "network_fault_injector = "
            "real_vehicle_integration.nodes.network_fault_injector:main",
            "network_fault_check = "
            "real_vehicle_integration.nodes.network_fault_check:main",
            "safety_node = real_vehicle_integration.nodes.safety_node:main",
            "smoke_check = real_vehicle_integration.nodes.smoke_check:main",
            "udp_actuator_bridge = "
            "real_vehicle_integration.nodes.udp_actuator_bridge:main",
            "udp_lidar_bridge = "
            "real_vehicle_integration.nodes.udp_lidar_bridge:main",
            "encoder_odom_node = "
            "real_vehicle_integration.nodes.encoder_odom:main",
        ],
    },
)
