"""Record and evaluate a manually pushed localization motion trial."""

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan

from ..localization_motion_metrics import (
    DEFAULT_THRESHOLDS,
    summarize_motion_trial,
)
from ..repository import find_repository_root
from ..ros_support import quaternion_to_yaw, stamp_to_seconds


DIAGNOSTIC_ERROR_LEVEL = (
    DiagnosticStatus.ERROR[0]
    if isinstance(DiagnosticStatus.ERROR, (bytes, bytearray))
    else int(DiagnosticStatus.ERROR)
)
LOCALIZATION_STATUS_FIELDS = (
    "elapsed_s",
    "phase",
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
    "phase",
    "stamp_s",
    "x_m",
    "y_m",
    "yaw_rad",
)
ODOM_FIELDS = (
    "elapsed_s",
    "phase",
    "stamp_s",
    "x_m",
    "y_m",
    "yaw_rad",
    "linear_speed_mps",
    "angular_speed_rad_s",
)
SCAN_FIELDS = (
    "elapsed_s",
    "phase",
    "stamp_s",
    "frame_id",
    "angle_min_rad",
    "angle_max_rad",
    "angle_increment_rad",
    "time_increment_s",
    "scan_time_s",
    "range_min_m",
    "range_max_m",
    "beam_count",
    "ranges_json",
)
LIDAR_STATUS_FIELDS = (
    "elapsed_s",
    "phase",
    "stamp_s",
    "level",
    "state",
    "sender",
    "scan_age_s",
    "scan_rate_hz",
    "packet_count",
    "invalid_packet_count",
    "rejected_sender_count",
    "completed_scans",
    "incomplete_scans",
    "duplicate_packets",
    "sequence_gaps",
    "pending_scans",
    "source_clock_offset_s",
    "session_id",
    "scan_id",
    "raw_points",
    "valid_beams",
    "total_beams",
    "valid_ratio",
    "health_ok",
)
ENCODER_STATUS_FIELDS = (
    "elapsed_s",
    "phase",
    "stamp_s",
    "level",
    "state",
    "sender",
    "packet_age_s",
    "packet_count",
    "invalid_packet_count",
    "rejected_sender_count",
    "duplicate_packets",
    "sequence_gaps",
    "session_changes",
    "session_id",
    "distance_per_count_m",
    "calibration",
    "invalid_transition_count",
    "odom_x_m",
    "odom_y_m",
    "odom_yaw_rad",
    "measured_speed_mps",
    "steering_angle_rad",
)


def _float_value(values, key):
    try:
        value = float(values.get(key, "nan"))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _int_value(values, key):
    try:
        return int(values.get(key, ""))
    except (TypeError, ValueError):
        return None


def _diagnostic_level(status):
    level = status.level
    if isinstance(level, (bytes, bytearray)):
        return int(level[0])
    return int(level)


def _write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _find_status(message, name):
    return next(
        (status for status in message.status if status.name == name),
        None,
    )


class LocalizationMotionTrialNode(Node):
    """Collect localizer, odometry, and transport messages by ROS type."""

    def __init__(
        self,
        pose_topic,
        localization_status_topic,
        scan_topic,
        odom_topic,
        lidar_status_topic,
        encoder_status_topic,
        node_name="localization_motion_trial",
    ):
        super().__init__(node_name)
        self.recording = False
        self.phase = "preflight"
        self.started_monotonic = None
        self.latest_received = {}
        self.latest_pose = None
        self.latest_localization_status = None
        self.latest_scan = None
        self.latest_odom = None
        self.latest_lidar_status = None
        self.latest_encoder_status = None
        self.pose_rows = []
        self.localization_status_rows = []
        self.scan_rows = []
        self.odom_rows = []
        self.lidar_status_rows = []
        self.encoder_status_rows = []
        self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._on_pose,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            localization_status_topic,
            self._on_localization_status,
            10,
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DiagnosticArray,
            lidar_status_topic,
            self._on_lidar_status,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            encoder_status_topic,
            self._on_encoder_status,
            10,
        )

    def elapsed_s(self):
        if self.started_monotonic is None:
            return 0.0
        return time.monotonic() - self.started_monotonic

    def readiness_failures(self, maximum_age_s=0.5):
        now = time.monotonic()
        failures = []
        for name in (
            "pose",
            "localization_status",
            "scan",
            "odom",
            "lidar_status",
            "encoder_status",
        ):
            received = self.latest_received.get(name)
            if received is None:
                failures.append(f"{name} has not been received")
            elif now - received > maximum_age_s:
                failures.append(
                    f"{name} is stale by {now - received:.3f} s"
                )
        if (
            self.latest_localization_status is None
            or self.latest_localization_status.get("state") != "localized"
        ):
            state = (
                "missing"
                if self.latest_localization_status is None
                else self.latest_localization_status.get("state", "")
            )
            failures.append(
                f"localization state is {state!r}, not 'localized'"
            )
        if (
            self.latest_lidar_status is None
            or int(self.latest_lidar_status.get("level", 2))
            >= DIAGNOSTIC_ERROR_LEVEL
        ):
            state = (
                "missing"
                if self.latest_lidar_status is None
                else self.latest_lidar_status.get("state", "")
            )
            failures.append(f"LiDAR transport is not healthy: {state}")
        if (
            self.latest_encoder_status is None
            or int(self.latest_encoder_status.get("level", 2))
            >= DIAGNOSTIC_ERROR_LEVEL
        ):
            state = (
                "missing"
                if self.latest_encoder_status is None
                else self.latest_encoder_status.get("state", "")
            )
            failures.append(f"encoder transport is not healthy: {state}")
        return failures

    def ready(self, maximum_age_s=0.5):
        return not self.readiness_failures(maximum_age_s)

    def start_recording(self, phase):
        self.pose_rows.clear()
        self.localization_status_rows.clear()
        self.scan_rows.clear()
        self.odom_rows.clear()
        self.lidar_status_rows.clear()
        self.encoder_status_rows.clear()
        self.started_monotonic = time.monotonic()
        self.recording = True
        self.phase = str(phase)
        self._append_snapshots()

    def set_phase(self, phase):
        self.phase = str(phase)
        if self.recording:
            self._append_snapshots()

    def _record(self, target, row):
        if not self.recording:
            return
        recorded = dict(row)
        recorded["elapsed_s"] = self.elapsed_s()
        recorded["phase"] = self.phase
        target.append(recorded)

    def _append_snapshots(self):
        pairs = (
            (self.pose_rows, self.latest_pose),
            (
                self.localization_status_rows,
                self.latest_localization_status,
            ),
            (self.scan_rows, self.latest_scan),
            (self.odom_rows, self.latest_odom),
            (self.lidar_status_rows, self.latest_lidar_status),
            (self.encoder_status_rows, self.latest_encoder_status),
        )
        for target, row in pairs:
            if row is not None:
                self._record(target, row)

    def _on_pose(self, message):
        self.latest_received["pose"] = time.monotonic()
        pose = message.pose.pose
        self.latest_pose = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": quaternion_to_yaw(pose.orientation),
        }
        self._record(self.pose_rows, self.latest_pose)

    def _on_localization_status(self, message):
        status = _find_status(message, "real_vehicle_localization")
        if status is None:
            return
        self.latest_received["localization_status"] = time.monotonic()
        values = {item.key: item.value for item in status.values}
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "state": str(status.message),
            "map_yaml": values.get("map_yaml", ""),
            "selection_reason": values.get(
                "selection_reason",
                "unknown",
            ),
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
            "pending_correction_dx_m",
            "pending_correction_dy_m",
            "pending_correction_dyaw_rad",
            "valid_beams",
            "candidate_count",
            "processed_scan_stamp_s",
            "scan_age_s",
            "processing_time_ms",
            "estimated_pose_x_m",
            "estimated_pose_y_m",
            "estimated_pose_yaw_rad",
        ):
            row[key] = _float_value(values, key)
        for key in (
            "correction_event_id",
            "correction_proposal_count",
            "correction_pending_count",
            "correction_rejection_count",
            "pending_reset_count",
            "confirmation_count",
            "confirmation_required",
            "cooldown_scans_remaining",
            "processed_scan_count",
        ):
            row[key] = _int_value(values, key)
        self.latest_localization_status = row
        self._record(self.localization_status_rows, row)

    def _on_scan(self, message):
        self.latest_received["scan"] = time.monotonic()
        ranges = [
            float(value) if math.isfinite(float(value)) else None
            for value in message.ranges
        ]
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "frame_id": str(message.header.frame_id),
            "angle_min_rad": float(message.angle_min),
            "angle_max_rad": float(message.angle_max),
            "angle_increment_rad": float(message.angle_increment),
            "time_increment_s": float(message.time_increment),
            "scan_time_s": float(message.scan_time),
            "range_min_m": float(message.range_min),
            "range_max_m": float(message.range_max),
            "beam_count": len(message.ranges),
            "ranges_json": json.dumps(ranges, separators=(",", ":")),
        }
        self.latest_scan = row
        self._record(self.scan_rows, row)

    def _on_odom(self, message):
        self.latest_received["odom"] = time.monotonic()
        pose = message.pose.pose
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": quaternion_to_yaw(pose.orientation),
            "linear_speed_mps": float(message.twist.twist.linear.x),
            "angular_speed_rad_s": float(message.twist.twist.angular.z),
        }
        self.latest_odom = row
        self._record(self.odom_rows, row)

    def _on_lidar_status(self, message):
        status = _find_status(message, "real_vehicle_lidar_transport")
        if status is None:
            return
        self.latest_received["lidar_status"] = time.monotonic()
        values = {item.key: item.value for item in status.values}
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "level": _diagnostic_level(status),
            "state": str(status.message),
            "sender": values.get("sender", ""),
            "session_id": values.get("session_id", ""),
            "health_ok": values.get("health_ok", ""),
        }
        for key in (
            "scan_age_s",
            "scan_rate_hz",
            "source_clock_offset_s",
            "valid_ratio",
        ):
            row[key] = _float_value(values, key)
        for key in (
            "packet_count",
            "invalid_packet_count",
            "rejected_sender_count",
            "completed_scans",
            "incomplete_scans",
            "duplicate_packets",
            "sequence_gaps",
            "pending_scans",
            "scan_id",
            "raw_points",
            "valid_beams",
            "total_beams",
        ):
            row[key] = _int_value(values, key)
        self.latest_lidar_status = row
        self._record(self.lidar_status_rows, row)

    def _on_encoder_status(self, message):
        status = _find_status(message, "real_vehicle_encoder_transport")
        if status is None:
            return
        self.latest_received["encoder_status"] = time.monotonic()
        values = {item.key: item.value for item in status.values}
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "level": _diagnostic_level(status),
            "state": str(status.message),
            "sender": values.get("sender", ""),
            "session_id": values.get("session_id", ""),
            "calibration": values.get("calibration", ""),
        }
        for key in (
            "packet_age_s",
            "distance_per_count_m",
            "odom_x_m",
            "odom_y_m",
            "odom_yaw_rad",
            "measured_speed_mps",
            "steering_angle_rad",
        ):
            row[key] = _float_value(values, key)
        for key in (
            "packet_count",
            "invalid_packet_count",
            "rejected_sender_count",
            "duplicate_packets",
            "sequence_gaps",
            "session_changes",
            "invalid_transition_count",
        ):
            row[key] = _int_value(values, key)
        self.latest_encoder_status = row
        self._record(self.encoder_status_rows, row)


def _parse_args(args=None):
    raw_args = (
        sys.argv
        if args is None
        else ["localization_motion_trial"] + list(args)
    )
    user_args = remove_ros_args(args=raw_args)[1:]
    root = find_repository_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-distance-m", type=float, default=0.50)
    parser.add_argument("--baseline-duration-s", type=float, default=5.0)
    parser.add_argument("--motion-duration-s", type=float, default=15.0)
    parser.add_argument("--settle-duration-s", type=float, default=5.0)
    parser.add_argument("--reference-window-s", type=float, default=2.0)
    parser.add_argument("--preflight-timeout-s", type=float, default=10.0)
    parser.add_argument("--pose-topic", default="/localization/pose")
    parser.add_argument(
        "--localization-status-topic",
        default="/localization/status",
    )
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument(
        "--lidar-status-topic",
        default="/lidar/transport_status",
    )
    parser.add_argument(
        "--encoder-status-topic",
        default="/encoder/transport_status",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            root
            / "experiments/real_vehicle/results/"
            "phase5_localization/manual_motion"
        ),
    )
    parser.add_argument("--trial-name")
    parser.add_argument(
        "--minimum-localized-rate",
        type=float,
        default=DEFAULT_THRESHOLDS["minimum_localized_rate"],
    )
    parser.add_argument(
        "--maximum-distance-error-m",
        type=float,
        default=DEFAULT_THRESHOLDS[
            "maximum_expected_distance_error_m"
        ],
    )
    parser.add_argument(
        "--maximum-odom-distance-error-m",
        type=float,
        default=DEFAULT_THRESHOLDS[
            "maximum_odom_expected_distance_error_m"
        ],
    )
    parser.add_argument(
        "--maximum-pose-odom-difference-m",
        type=float,
        default=DEFAULT_THRESHOLDS[
            "maximum_pose_odom_distance_difference_m"
        ],
    )
    parser.add_argument(
        "--maximum-lateral-displacement-m",
        type=float,
        default=DEFAULT_THRESHOLDS[
            "maximum_lateral_displacement_m"
        ],
    )
    parser.add_argument(
        "--maximum-yaw-change-rad",
        type=float,
        default=DEFAULT_THRESHOLDS["maximum_yaw_change_rad"],
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
        "--maximum-nonlocalized-duration-s",
        type=float,
        default=DEFAULT_THRESHOLDS[
            "maximum_nonlocalized_duration_s"
        ],
    )
    return parser.parse_args(user_args), raw_args[0]


def _output_directory(options):
    base = options.output_dir.expanduser().resolve()
    name = options.trial_name or datetime.now().strftime(
        "manual_50cm_%Y%m%d_%H%M%S"
    )
    output = base / name
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"trial output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    return output


def _spin_until(node, end_elapsed_s):
    while rclpy.ok() and node.elapsed_s() < end_elapsed_s:
        remaining_s = end_elapsed_s - node.elapsed_s()
        rclpy.spin_once(node, timeout_sec=min(0.10, remaining_s))


def _thresholds(options):
    return {
        "minimum_localized_rate": options.minimum_localized_rate,
        "maximum_expected_distance_error_m": (
            options.maximum_distance_error_m
        ),
        "maximum_odom_expected_distance_error_m": (
            options.maximum_odom_distance_error_m
        ),
        "maximum_pose_odom_distance_difference_m": (
            options.maximum_pose_odom_difference_m
        ),
        "maximum_lateral_displacement_m": (
            options.maximum_lateral_displacement_m
        ),
        "maximum_yaw_change_rad": options.maximum_yaw_change_rad,
        "maximum_match_score_m2": options.maximum_match_score_m2,
        "maximum_scan_age_s": options.maximum_scan_age_s,
        "maximum_nonlocalized_duration_s": (
            options.maximum_nonlocalized_duration_s
        ),
    }


def _validate_options(options):
    positive_values = {
        "--expected-distance-m": options.expected_distance_m,
        "--baseline-duration-s": options.baseline_duration_s,
        "--motion-duration-s": options.motion_duration_s,
        "--settle-duration-s": options.settle_duration_s,
        "--reference-window-s": options.reference_window_s,
        "--preflight-timeout-s": options.preflight_timeout_s,
    }
    for name, value in positive_values.items():
        if value <= 0.0:
            raise SystemExit(f"{name} must be positive")


def main(args=None):
    options, program_name = _parse_args(args)
    _validate_options(options)
    try:
        output = _output_directory(options)
    except FileExistsError as exc:
        print(f"試験を開始できません: {exc}", file=sys.stderr)
        print(
            "既存結果は上書きしません。"
            "--trial-nameに新しい名前を指定してください。"
            " 例: --trial-name manual_50cm_trial1_retry1",
            file=sys.stderr,
        )
        return 2
    rclpy.init(args=[program_name])
    node = LocalizationMotionTrialNode(
        options.pose_topic,
        options.localization_status_topic,
        options.scan_topic,
        options.odom_topic,
        options.lidar_status_topic,
        options.encoder_status_topic,
    )
    interrupted = False
    preflight_failures = []
    phases = (
        (
            "baseline",
            options.baseline_duration_s,
            "フェーズ1/3 静止: 車体を動かさないでください。",
        ),
        (
            "motion",
            options.motion_duration_s,
            (
                "フェーズ2/3 手動移動: 車体を前へ"
                f"{options.expected_distance_m:.2f} m手で押し、"
                "終了線で止めてください。停止後もCtrl+Cを押さず、"
                "その場で待ってください。"
            ),
        ),
        (
            "settle",
            options.settle_duration_s,
            "フェーズ3/3 停止: 車体から手を離してください。",
        ),
    )
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
            preflight_failures = node.readiness_failures()
        else:
            total_end_s = 0.0
            for index, (phase, duration_s, prompt) in enumerate(phases):
                if index == 0:
                    node.start_recording(phase)
                else:
                    node.set_phase(phase)
                total_end_s += duration_s
                print(
                    f"{prompt} ({duration_s:.1f} s)",
                    flush=True,
                )
                _spin_until(node, total_end_s)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        actual_duration_s = node.elapsed_s()
        node.recording = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    requested_duration_s = sum(phase[1] for phase in phases)
    summary = summarize_motion_trial(
        node.localization_status_rows,
        node.pose_rows,
        node.odom_rows,
        node.lidar_status_rows,
        expected_distance_m=options.expected_distance_m,
        requested_duration_s=requested_duration_s,
        actual_duration_s=actual_duration_s,
        thresholds=_thresholds(options),
        interrupted=interrupted,
        reference_window_s=options.reference_window_s,
        encoder_rows=node.encoder_status_rows,
    )
    summary["phase_durations_s"] = {
        phase: duration_s for phase, duration_s, _ in phases
    }
    summary["preflight_ready"] = not preflight_failures
    summary["preflight_failures"] = preflight_failures
    summary["recording_format_version"] = 3
    summary["scan_samples"] = len(node.scan_rows)
    if preflight_failures:
        summary["failures"].append(
            "preflight failed: " + "; ".join(preflight_failures)
        )
        summary["status"] = "failed"

    _write_csv(
        output / "localization_status.csv",
        LOCALIZATION_STATUS_FIELDS,
        node.localization_status_rows,
    )
    _write_csv(output / "pose.csv", POSE_FIELDS, node.pose_rows)
    _write_csv(output / "scan.csv", SCAN_FIELDS, node.scan_rows)
    _write_csv(output / "odom.csv", ODOM_FIELDS, node.odom_rows)
    _write_csv(
        output / "lidar_status.csv",
        LIDAR_STATUS_FIELDS,
        node.lidar_status_rows,
    )
    _write_csv(
        output / "encoder_status.csv",
        ENCODER_STATUS_FIELDS,
        node.encoder_status_rows,
    )
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    print(f"output_dir: {output}", flush=True)
    return 0 if summary["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
