from real_vehicle_integration.hardware_isolation import (
    find_forbidden_nodes,
    is_isolated_control_topic,
    node_basename,
)


def test_node_basename_handles_namespaces():
    assert node_basename("/vehicle/udp_actuator_bridge") == (
        "udp_actuator_bridge"
    )


def test_forbidden_nodes_are_detected_without_substring_false_positive():
    names = [
        "/controller",
        "/dry_run_guard",
        "/vehicle/udp_actuator_bridge",
        "/udp_actuator_bridge_monitor",
        "/mock_vehicle",
    ]

    assert find_forbidden_nodes(names) == [
        "/mock_vehicle",
        "/vehicle/udp_actuator_bridge",
    ]


def test_only_dry_run_control_topics_are_isolated():
    assert is_isolated_control_topic("/dry_run/control_safe")
    assert not is_isolated_control_topic("/vehicle/control_safe")
