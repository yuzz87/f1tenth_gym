"""Record and evaluate a stationary real-map localization trial."""

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from ..localization_static_metrics import (
    DEFAULT_THRESHOLDS,
    summarize_static_trial,
)
from ..repository import find_repository_root
from ..ros_support import quaternion_to_yaw, stamp_to_seconds


STATUS_FIELDS = (
    "elapsed_s",
    "stamp_s",
    "state",
    "match_score_m2",
    "predicted_match_score_m2",
    "objective_score_m2",
    "prior_cost_m2",
    "scan_improvement_m2",
    "objective_improvement_m2",
    "selection_reason",
    "position_correction_m",
    "yaw_correction_rad",
    "correction_dx_m",
    "correction_dy_m",
    "correction_dyaw_rad",
    "proposed_correction_dx_m",
    "proposed_correction_dy_m",
    "proposed_correction_dyaw_rad",
    "correction_event_id",
    "correction_proposal_count",
    "correction_pending_count",
    "correction_rejection_count",
    "pending_reset_count",
    "confirmation_count",
    "confirmation_required",
    "cooldown_scans_remaining",
    "pending_correction_dx_m",
    "pending_correction_dy_m",
    "pending_correction_dyaw_rad",
    "valid_beams",
    "candidate_count",
    "processed_scan_count",
    "processed_scan_stamp_s",
    "scan_age_s",
    "processing_time_ms",
    "estimated_pose_x_m",
    "estimated_pose_y_m",
    "estimated_pose_yaw_rad",
    "map_yaml",
)
POSE_FIELDS = (
    "elapsed_s",
    "stamp_s",
    "x_m",
    "y_m",
    "yaw_rad",
)


def _float_value(values, key):
    try:
        return float(values.get(key, "nan"))
    except (TypeError, ValueError):
        return float("nan")


def _write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class LocalizationStaticTrialNode(Node):
    """Collect typed pose/status messages without using the ROS 2 CLI daemon."""

    def __init__(self, pose_topic, status_topic):
        super().__init__("localization_static_trial")
        self.recording = False
        self.started_monotonic = None
        self.latest_pose_received_monotonic = None
        self.latest_status_received_monotonic = None
        self.latest_state = ""
        self.status_rows = []
        self.pose_rows = []
        self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._on_pose,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            status_topic,
            self._on_status,
            10,
        )

    def ready(self, maximum_age_s=0.5):
        now = time.monotonic()
        return bool(
            self.latest_state == "localized"
            and self.latest_pose_received_monotonic is not None
            and self.latest_status_received_monotonic is not None
            and now - self.latest_pose_received_monotonic <= maximum_age_s
            and now - self.latest_status_received_monotonic <= maximum_age_s
        )

    def start_recording(self):
        self.status_rows.clear()
        self.pose_rows.clear()
        self.started_monotonic = time.monotonic()
        self.recording = True

    def elapsed_s(self):
        if self.started_monotonic is None:
            return 0.0
        return time.monotonic() - self.started_monotonic

    def _on_pose(self, message):
        self.latest_pose_received_monotonic = time.monotonic()
        if not self.recording:
            return
        pose = message.pose.pose
        self.pose_rows.append({
            "elapsed_s": self.elapsed_s(),
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": quaternion_to_yaw(pose.orientation),
        })

    def _on_status(self, message):
        self.latest_status_received_monotonic = time.monotonic()
        if not message.status:
            return
        status = next(
            (
                item for item in message.status
                if item.name == "real_vehicle_localization"
            ),
            message.status[0],
        )
        self.latest_state = str(status.message)
        if not self.recording:
            return
        values = {item.key: item.value for item in status.values}
        row = {
            "elapsed_s": self.elapsed_s(),
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "state": self.latest_state,
            "selection_reason": values.get("selection_reason", "unknown"),
            "map_yaml": values.get("map_yaml", ""),
        }
        for key in (
            "match_score_m2",
            "predicted_match_score_m2",
            "objective_score_m2",
            "prior_cost_m2",
            "scan_improvement_m2",
            "objective_improvement_m2",
            "position_correction_m",
            "yaw_correction_rad",
            "correction_dx_m",
            "correction_dy_m",
            "correction_dyaw_rad",
            "proposed_correction_dx_m",
            "proposed_correction_dy_m",
            "proposed_correction_dyaw_rad",
            "correction_event_id",
            "correction_proposal_count",
            "correction_pending_count",
            "correction_rejection_count",
            "pending_reset_count",
            "confirmation_count",
            "confirmation_required",
            "cooldown_scans_remaining",
            "pending_correction_dx_m",
            "pending_correction_dy_m",
            "pending_correction_dyaw_rad",
            "valid_beams",
            "candidate_count",
            "processed_scan_count",
            "processed_scan_stamp_s",
            "scan_age_s",
            "processing_time_ms",
            "estimated_pose_x_m",
            "estimated_pose_y_m",
            "estimated_pose_yaw_rad",
        ):
            row[key] = _float_value(values, key)
        self.status_rows.append(row)


def _parse_args(args=None):
    raw_args = sys.argv if args is None else ["localization_static_trial"] + list(args)
    user_args = remove_ros_args(args=raw_args)[1:]
    root = find_repository_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--preflight-timeout-s", type=float, default=10.0)
    parser.add_argument("--pose-topic", default="/localization/pose")
    parser.add_argument("--status-topic", default="/localization/status")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            root
            / "experiments/real_vehicle/results/phase5_localization"
        ),
    )
    parser.add_argument("--trial-name")
    parser.add_argument(
        "--minimum-localized-rate",
        type=float,
        default=DEFAULT_THRESHOLDS["minimum_localized_rate"],
    )
    parser.add_argument(
        "--maximum-x-span-m",
        type=float,
        default=DEFAULT_THRESHOLDS["maximum_x_span_m"],
    )
    parser.add_argument(
        "--maximum-y-span-m",
        type=float,
        default=DEFAULT_THRESHOLDS["maximum_y_span_m"],
    )
    parser.add_argument(
        "--maximum-yaw-span-rad",
        type=float,
        default=DEFAULT_THRESHOLDS["maximum_yaw_span_rad"],
    )
    parser.add_argument(
        "--maximum-match-score-m2",
        type=float,
        default=DEFAULT_THRESHOLDS["maximum_match_score_m2"],
    )
    parser.add_argument(
        "--maximum-scan-age-s",
        type=float,
        default=DEFAULT_THRESHOLDS["maximum_scan_age_s"],
    )
    parser.add_argument(
        "--maximum-processing-time-ms",
        type=float,
        default=DEFAULT_THRESHOLDS["maximum_processing_time_ms"],
    )
    return parser.parse_args(user_args), raw_args[0]


def _output_directory(options):
    base = options.output_dir.expanduser().resolve()
    name = options.trial_name or datetime.now().strftime(
        "static_%Y%m%d_%H%M%S"
    )
    output = base / name
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"trial output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    return output


def main(args=None):
    options, program_name = _parse_args(args)
    if options.duration_s <= 0.0:
        raise SystemExit("--duration-s must be positive")
    output = _output_directory(options)
    rclpy.init(args=[program_name])
    node = LocalizationStaticTrialNode(
        options.pose_topic,
        options.status_topic,
    )
    interrupted = False
    preflight_failed = False
    try:
        preflight_started = time.monotonic()
        while (
            rclpy.ok()
            and not node.ready()
            and time.monotonic() - preflight_started
            < options.preflight_timeout_s
        ):
            rclpy.spin_once(node, timeout_sec=0.10)
        if not node.ready():
            preflight_failed = True
        else:
            print(
                f"localized preflight passed; recording {options.duration_s:.1f}s",
                flush=True,
            )
            node.start_recording()
            while rclpy.ok() and node.elapsed_s() < options.duration_s:
                rclpy.spin_once(node, timeout_sec=0.10)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        actual_duration_s = node.elapsed_s()
        node.recording = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    thresholds = {
        "minimum_localized_rate": options.minimum_localized_rate,
        "maximum_x_span_m": options.maximum_x_span_m,
        "maximum_y_span_m": options.maximum_y_span_m,
        "maximum_yaw_span_rad": options.maximum_yaw_span_rad,
        "maximum_match_score_m2": options.maximum_match_score_m2,
        "maximum_scan_age_s": options.maximum_scan_age_s,
        "maximum_processing_time_ms": options.maximum_processing_time_ms,
    }
    summary = summarize_static_trial(
        node.status_rows,
        node.pose_rows,
        requested_duration_s=options.duration_s,
        actual_duration_s=actual_duration_s,
        thresholds=thresholds,
        interrupted=interrupted,
    )
    if preflight_failed:
        summary["failures"].append(
            "preflight did not receive a fresh localized status and pose"
        )
        summary["status"] = "failed"
    _write_csv(output / "status.csv", STATUS_FIELDS, node.status_rows)
    _write_csv(output / "pose.csv", POSE_FIELDS, node.pose_rows)
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    print(f"output_dir: {output}", flush=True)
    return 0 if summary["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
