"""Assert that a running two-side graph exercised its configured network profile."""

import json

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node


class NetworkFaultCheckNode(Node):
    def __init__(self):
        super().__init__("network_fault_check")
        self.declare_parameter("duration_s", 5.0)
        self.declare_parameter("expect_dropout", False)
        self.declare_parameter("expect_replay", False)
        self.received = 0
        self.relayed = 0
        self.dropped = 0
        self.duplicates = 0
        self.stale_replays = 0
        self.reordered = 0
        self.actuator_reasons = set()
        self.finished = False
        self.success = False
        self.create_subscription(
            DiagnosticArray,
            "/network_fault_injector/status",
            self._on_fault_status,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/vehicle/actuator_status",
            self._on_actuator_status,
            10,
        )
        duration_s = max(float(self.get_parameter("duration_s").value), 0.1)
        self.create_timer(duration_s, self._finish)

    def _on_fault_status(self, message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            self.received = max(self.received, int(values.get("received", 0)))
            self.relayed = max(self.relayed, int(values.get("relayed", 0)))
            self.dropped = max(self.dropped, int(values.get("dropped", 0)))
            self.duplicates = max(self.duplicates, int(values.get("duplicates", 0)))
            self.stale_replays = max(
                self.stale_replays,
                int(values.get("stale_replays", 0)),
            )
            self.reordered = max(self.reordered, int(values.get("reordered", 0)))

    def _on_actuator_status(self, message):
        self.actuator_reasons.update(status.message for status in message.status)

    def _finish(self):
        expect_dropout = bool(self.get_parameter("expect_dropout").value)
        expect_replay = bool(self.get_parameter("expect_replay").value)
        traffic_ok = self.received > 0 and self.relayed > 0
        dropout_ok = self.dropped > 0 if expect_dropout else self.dropped == 0
        replay_events = self.duplicates + self.stale_replays + self.reordered
        replay_ok = replay_events > 0 if expect_replay else True
        safe_response_seen = bool(
            self.actuator_reasons.intersection({
                "safe_stop",
                "command_timeout",
                "uncalibrated_speed_to_duty",
            })
        )
        self.success = traffic_ok and dropout_ok and replay_ok and safe_response_seen
        print(json.dumps({
            "success": self.success,
            "expect_dropout": expect_dropout,
            "received_max": self.received,
            "relayed_max": self.relayed,
            "dropped_max": self.dropped,
            "duplicates_max": self.duplicates,
            "stale_replays_max": self.stale_replays,
            "reordered_max": self.reordered,
            "actuator_reasons": sorted(self.actuator_reasons),
        }, indent=2), flush=True)
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = NetworkFaultCheckNode()
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
