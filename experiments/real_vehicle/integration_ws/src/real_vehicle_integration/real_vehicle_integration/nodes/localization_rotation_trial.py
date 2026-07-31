"""Record and evaluate a known-angle manual localization rotation trial."""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args

from ..localization_rotation_metrics import (
    DEFAULT_THRESHOLDS,
    summarize_rotation_trial,
)
from ..repository import find_repository_root
from .localization_motion_trial import (
    ENCODER_STATUS_FIELDS,
    LIDAR_STATUS_FIELDS,
    LOCALIZATION_STATUS_FIELDS,
    ODOM_FIELDS,
    POSE_FIELDS,
    SCAN_FIELDS,
    LocalizationMotionTrialNode,
    _spin_until,
    _write_csv,
)


def _parse_args(args=None):
    raw_args = (
        sys.argv
        if args is None
        else ["localization_rotation_trial"] + list(args)
    )
    user_args = remove_ros_args(args=raw_args)[1:]
    root = find_repository_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-yaw-deg", type=float, default=30.0)
    parser.add_argument("--baseline-duration-s", type=float, default=5.0)
    parser.add_argument("--rotation-duration-s", type=float, default=15.0)
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
            "phase5_localization/manual_rotation"
        ),
    )
    parser.add_argument("--trial-name")
    parser.add_argument(
        "--minimum-localized-rate",
        type=float,
        default=DEFAULT_THRESHOLDS["minimum_localized_rate"],
    )
    parser.add_argument(
        "--maximum-yaw-error-deg",
        type=float,
        default=math.degrees(
            DEFAULT_THRESHOLDS["maximum_expected_yaw_error_rad"]
        ),
    )
    parser.add_argument(
        "--maximum-position-drift-m",
        type=float,
        default=DEFAULT_THRESHOLDS["maximum_position_drift_m"],
    )
    parser.add_argument(
        "--minimum-correction-events",
        type=int,
        default=DEFAULT_THRESHOLDS["minimum_correction_event_delta"],
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
    direction = "left" if options.expected_yaw_deg > 0.0 else "right"
    angle = abs(options.expected_yaw_deg)
    name = options.trial_name or (
        f"manual_{direction}_{angle:g}deg_"
        + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    output = base / name
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(
            f"trial output directory is not empty: {output}"
        )
    output.mkdir(parents=True, exist_ok=True)
    return output


def _thresholds(options):
    return {
        "minimum_localized_rate": options.minimum_localized_rate,
        "maximum_expected_yaw_error_rad": math.radians(
            options.maximum_yaw_error_deg
        ),
        "maximum_position_drift_m": options.maximum_position_drift_m,
        "minimum_correction_event_delta": (
            options.minimum_correction_events
        ),
        "maximum_match_score_m2": options.maximum_match_score_m2,
        "maximum_scan_age_s": options.maximum_scan_age_s,
        "maximum_nonlocalized_duration_s": (
            options.maximum_nonlocalized_duration_s
        ),
    }


def _validate_options(options):
    positive_values = {
        "--baseline-duration-s": options.baseline_duration_s,
        "--rotation-duration-s": options.rotation_duration_s,
        "--settle-duration-s": options.settle_duration_s,
        "--reference-window-s": options.reference_window_s,
        "--preflight-timeout-s": options.preflight_timeout_s,
        "--maximum-yaw-error-deg": options.maximum_yaw_error_deg,
        "--maximum-position-drift-m": options.maximum_position_drift_m,
    }
    for name, value in positive_values.items():
        if value <= 0.0:
            raise SystemExit(f"{name} must be positive")
    if not 0.0 < abs(options.expected_yaw_deg) < 180.0:
        raise SystemExit(
            "--expected-yaw-deg must be non-zero and less than 180 degrees"
        )
    if options.minimum_correction_events < 0:
        raise SystemExit("--minimum-correction-events cannot be negative")


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
            " 例: --trial-name guarded_left_30deg_trial1_retry1",
            file=sys.stderr,
        )
        return 2

    expected_yaw_rad = math.radians(options.expected_yaw_deg)
    direction_text = (
        "左（反時計回り）"
        if expected_yaw_rad > 0.0
        else "右（時計回り）"
    )
    phases = (
        (
            "baseline",
            options.baseline_duration_s,
            "フェーズ1/3 静止: 車体を動かさないでください。",
        ),
        (
            "rotation",
            options.rotation_duration_s,
            (
                "フェーズ2/3 手動旋回: 後輪軸中央を床の基準点に保ち、"
                f"車体を{direction_text}へ"
                f"{abs(options.expected_yaw_deg):g}度ゆっくり回してください。"
                "角度線で止めた後もCtrl+Cを押さず、その場で待ってください。"
            ),
        ),
        (
            "settle",
            options.settle_duration_s,
            "フェーズ3/3 停止: 車体から手を離してください。",
        ),
    )

    rclpy.init(args=[program_name])
    node = LocalizationMotionTrialNode(
        options.pose_topic,
        options.localization_status_topic,
        options.scan_topic,
        options.odom_topic,
        options.lidar_status_topic,
        options.encoder_status_topic,
        node_name="localization_rotation_trial",
    )
    interrupted = False
    preflight_failures = []
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
            print("事前確認に失敗しました:", flush=True)
            for failure in preflight_failures:
                print(f"  - {failure}", flush=True)
        else:
            print(
                "事前確認完了: localization、LiDAR、encoderは正常です。",
                flush=True,
            )
            total_end_s = 0.0
            for index, (phase, duration_s, prompt) in enumerate(phases):
                if index == 0:
                    node.start_recording(phase)
                else:
                    node.set_phase(phase)
                total_end_s += duration_s
                print(f"{prompt} ({duration_s:.1f} s)", flush=True)
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
    summary = summarize_rotation_trial(
        node.localization_status_rows,
        node.pose_rows,
        node.odom_rows,
        node.lidar_status_rows,
        expected_yaw_rad=expected_yaw_rad,
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
    summary["trial_type"] = "manual_rotation"
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
