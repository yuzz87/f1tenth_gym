"""Wait for a diagnostic status message matching an expected value."""

import json

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node


class DiagnosticCheckNode(Node):
    def __init__(self):
        super().__init__("diagnostic_check")
        self.declare_parameter("topic", "/vehicle/actuator_status")
        self.declare_parameter("expected_message", "command_timeout")
        self.declare_parameter("duration_s", 3.0)
        self.expected = str(self.get_parameter("expected_message").value)
        self.seen_messages = set()
        self.success = False
        self.finished = False
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("topic").value),
            self._on_status,
            10,
        )
        self.create_timer(
            max(float(self.get_parameter("duration_s").value), 0.1),
            self._finish,
        )

    def _on_status(self, message):
        for status in message.status:
            self.seen_messages.add(status.message)
            values = {item.key: item.value for item in status.values}
            self.seen_messages.update(values.values())
        if self.expected in self.seen_messages:
            self.success = True
            self.finished = True

    def _finish(self):
        print(json.dumps({
            "success": self.success,
            "expected": self.expected,
            "seen": sorted(self.seen_messages),
        }, indent=2), flush=True)
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticCheckNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)
