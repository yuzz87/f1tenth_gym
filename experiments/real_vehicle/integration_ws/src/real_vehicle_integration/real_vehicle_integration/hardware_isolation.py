"""ROS-independent checks for the Phase 6 hardware isolation boundary."""

from pathlib import PurePosixPath


FORBIDDEN_NODE_BASENAMES = frozenset({
    "actuator",
    "actuator_node",
    "mock_vehicle",
    "real_vehicle_actuator",
    "udp_actuator_bridge",
})


def node_basename(name):
    """Return a normalized ROS node basename."""

    normalized = "/" + str(name).strip("/")
    return PurePosixPath(normalized).name


def find_forbidden_nodes(node_names):
    """Return sorted node names that could reach hardware or fake sensors."""

    return sorted({
        str(name)
        for name in node_names
        if node_basename(name) in FORBIDDEN_NODE_BASENAMES
    })


def is_isolated_control_topic(topic_name):
    """Return whether a topic is inside the dedicated dry-run namespace."""

    normalized = "/" + str(topic_name).strip("/")
    return normalized.startswith("/dry_run/")


__all__ = [
    "FORBIDDEN_NODE_BASENAMES",
    "find_forbidden_nodes",
    "is_isolated_control_topic",
    "node_basename",
]
