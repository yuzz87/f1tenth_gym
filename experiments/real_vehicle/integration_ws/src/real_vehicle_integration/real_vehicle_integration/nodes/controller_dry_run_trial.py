"""Record and evaluate a real-sensor controller dry-run trial."""

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger

from ..controller_dry_run_metrics import (
    is_recoverable_preflight_fault,
    summarize_controller_dry_run,
)
from ..dry_run_faults import SUPPORTED_FAULT_MODES
from ..hardware_isolation import find_forbidden_nodes
from ..repository import find_repository_root
from ..ros_support import (
    AckermannDriveStamped,
    require_ackermann_messages,
    stamp_to_seconds,
)


COMMAND_FIELDS = (
    "elapsed_s",
    "stamp_s",
    "speed_mps",
    "steering_angle_rad",
    "steering_rate_rad_s",
)
CONTROLLER_FIELDS = (
    "elapsed_s",
    "stamp_s",
    "state",
    "controller_type",
    "solve_time_ms",
    "period_ms",
    "overrun",
    "deadline_exceeded",
    "optimizer_success",
    "function_evaluations",
    "cost",
)
SAFETY_FIELDS = (
    "elapsed_s",
    "stamp_s",
    "state",
    "fault_reason",
    "scan_age_s",
    "pose_age_s",
    "control_age_s",
    "output_enabled",
)
LOCALIZATION_FIELDS = (
    "elapsed_s",
    "stamp_s",
    "state",
    "match_score_m2",
    "valid_beams",
    "scan_age_s",
    "processing_time_ms",
)
GUARD_FIELDS = (
    "elapsed_s",
    "stamp_s",
    "state",
    "hardware_path_absent",
    "topic_isolated",
    "violation_latched",
    "forbidden_nodes",
    "command_count",
    "non_neutral_count",
    "constraint_violation_count",
)
FAULT_FIELDS = (
    "elapsed_s",
    "stamp_s",
    "state",
    "mode",
    "active",
    "emergency_stop_active",
    "trigger_count",
    "clear_count",
    "scan_received",
    "scan_relayed",
    "scan_dropped",
    "pose_received",
    "pose_relayed",
    "pose_dropped",
    "control_received",
    "control_relayed",
    "control_dropped",
)
TRANSPORT_FIELDS = (
    "elapsed_s",
    "stamp_s",
    "name",
    "level",
    "state",
    "packet_count",
    "invalid_packet_count",
    "sequence_gaps",
    "incomplete_scans",
    "session_changes",
)


def _diagnostic_level(status):
    level = status.level
    if isinstance(level, (bytes, bytearray)):
        return int(level[0])
    return int(level)


def _find_status(message, name):
    return next(
        (status for status in message.status if status.name == name),
        None,
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


def _write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class ControllerDryRunTrialNode(Node):
    """Collect the isolated controller path and its real-sensor diagnostics."""

    def __init__(self):
        require_ackermann_messages()
        super().__init__("controller_dry_run_trial")
        self.recording = False
        self.started_monotonic = None
        self.latest_received = {}
        self.latest = {}
        self.request_rows = []
        self.safe_rows = []
        self.controller_rows = []
        self.safety_rows = []
        self.localization_rows = []
        self.guard_rows = []
        self.fault_rows = []
        self.transport_rows = []

        self.create_subscription(
            AckermannDriveStamped,
            "/dry_run/control_raw",
            lambda message: self._on_command(
                "request",
                self.request_rows,
                message,
            ),
            10,
        )
        self.create_subscription(
            AckermannDriveStamped,
            "/dry_run/control_safe",
            lambda message: self._on_command(
                "safe",
                self.safe_rows,
                message,
            ),
            10,
        )
        subscriptions = (
            (
                "/dry_run/controller_status",
                "controller",
                self._on_controller,
            ),
            (
                "/dry_run/safety_status",
                "safety",
                self._on_safety,
            ),
            (
                "/localization/status",
                "localization",
                self._on_localization,
            ),
            (
                "/dry_run/hardware_guard_status",
                "guard",
                self._on_guard,
            ),
            (
                "/dry_run/fault_injector_status",
                "fault",
                self._on_fault,
            ),
            (
                "/lidar/transport_status",
                "lidar_transport",
                self._on_transport,
            ),
            (
                "/encoder/transport_status",
                "encoder_transport",
                self._on_transport,
            ),
        )
        for topic, _key, callback in subscriptions:
            self.create_subscription(
                DiagnosticArray,
                topic,
                callback,
                10,
            )

        self.trigger_client = self.create_client(
            Trigger,
            "/dry_run_fault_injector/trigger",
        )
        self.clear_client = self.create_client(
            Trigger,
            "/dry_run_fault_injector/clear",
        )
        self.safety_reset_client = self.create_client(
            Trigger,
            "/safety/reset_fault",
        )

    def elapsed_s(self):
        if self.started_monotonic is None:
            return 0.0
        return time.monotonic() - self.started_monotonic

    def _record(self, target, row):
        if not self.recording:
            return
        recorded = dict(row)
        recorded["elapsed_s"] = self.elapsed_s()
        target.append(recorded)

    def _remember(self, key, row):
        self.latest[key] = row
        self.latest_received[key] = time.monotonic()

    def _on_command(self, key, target, message):
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "speed_mps": float(message.drive.speed),
            "steering_angle_rad": float(message.drive.steering_angle),
            "steering_rate_rad_s": float(
                message.drive.steering_angle_velocity
            ),
        }
        self._remember(key, row)
        self._record(target, row)

    def _on_controller(self, message):
        status = _find_status(message, "real_vehicle_controller")
        if status is None:
            return
        values = {item.key: item.value for item in status.values}
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "state": str(status.message),
            "controller_type": values.get("controller_type", ""),
            "overrun": values.get("overrun", "false"),
            "deadline_exceeded": values.get(
                "deadline_exceeded",
                "false",
            ),
            "optimizer_success": values.get(
                "optimizer_success",
                "false",
            ),
            "solve_time_ms": _float_value(values, "solve_time_ms"),
            "period_ms": _float_value(values, "period_ms"),
            "function_evaluations": _int_value(
                values,
                "function_evaluations",
            ),
            "cost": _float_value(values, "cost"),
        }
        self._remember("controller", row)
        self._record(self.controller_rows, row)

    def _on_safety(self, message):
        status = _find_status(message, "real_vehicle_safety")
        if status is None:
            return
        values = {item.key: item.value for item in status.values}
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "state": values.get("state", str(status.message)),
            "fault_reason": values.get("fault_reason", ""),
            "output_enabled": values.get("output_enabled", "false"),
        }
        for key in ("scan_age_s", "pose_age_s", "control_age_s"):
            row[key] = _float_value(values, key)
        self._remember("safety", row)
        self._record(self.safety_rows, row)

    def _on_localization(self, message):
        status = _find_status(message, "real_vehicle_localization")
        if status is None:
            return
        values = {item.key: item.value for item in status.values}
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "state": str(status.message),
        }
        for key in (
            "match_score_m2",
            "valid_beams",
            "scan_age_s",
            "processing_time_ms",
        ):
            row[key] = _float_value(values, key)
        self._remember("localization", row)
        self._record(self.localization_rows, row)

    def _on_guard(self, message):
        status = _find_status(
            message,
            "real_vehicle_dry_run_guard",
        )
        if status is None:
            return
        values = {item.key: item.value for item in status.values}
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "state": str(status.message),
            "hardware_path_absent": values.get(
                "hardware_path_absent",
                "false",
            ),
            "topic_isolated": values.get("topic_isolated", "false"),
            "violation_latched": values.get(
                "violation_latched",
                "true",
            ),
            "forbidden_nodes": values.get("forbidden_nodes", ""),
        }
        for key in (
            "command_count",
            "non_neutral_count",
            "constraint_violation_count",
        ):
            row[key] = _int_value(values, key)
        self._remember("guard", row)
        self._record(self.guard_rows, row)

    def _on_fault(self, message):
        status = _find_status(
            message,
            "real_vehicle_dry_run_fault_injector",
        )
        if status is None:
            return
        values = {item.key: item.value for item in status.values}
        row = {
            "stamp_s": stamp_to_seconds(message.header.stamp),
            "state": str(status.message),
            "mode": values.get("mode", ""),
            "active": values.get("active", "false"),
            "emergency_stop_active": values.get(
                "emergency_stop_active",
                "false",
            ),
        }
        for key in (
            "trigger_count",
            "clear_count",
            "scan_received",
            "scan_relayed",
            "scan_dropped",
            "pose_received",
            "pose_relayed",
            "pose_dropped",
            "control_received",
            "control_relayed",
            "control_dropped",
        ):
            row[key] = _int_value(values, key)
        self._remember("fault", row)
        self._record(self.fault_rows, row)

    def _on_transport(self, message):
        for status_name, key in (
            ("real_vehicle_lidar_transport", "lidar_transport"),
            ("real_vehicle_encoder_transport", "encoder_transport"),
        ):
            status = _find_status(message, status_name)
            if status is None:
                continue
            values = {item.key: item.value for item in status.values}
            row = {
                "stamp_s": stamp_to_seconds(message.header.stamp),
                "name": key,
                "level": _diagnostic_level(status),
                "state": str(status.message),
            }
            for field in (
                "packet_count",
                "invalid_packet_count",
                "sequence_gaps",
                "incomplete_scans",
                "session_changes",
            ):
                row[field] = _int_value(values, field)
            self._remember(key, row)
            self._record(self.transport_rows, row)

    def readiness_failures(
        self,
        controller_type,
        fault_mode,
        maximum_age_s=0.5,
        require_safety_running=True,
    ):
        failures = []
        now = time.monotonic()
        required = (
            "request",
            "safe",
            "controller",
            "safety",
            "localization",
            "guard",
            "fault",
            "lidar_transport",
            "encoder_transport",
        )
        for key in required:
            received = self.latest_received.get(key)
            if received is None:
                failures.append(f"{key} has not been received")
            elif now - received > maximum_age_s:
                failures.append(
                    f"{key} is stale by {now - received:.3f} s"
                )
        if (
            require_safety_running
            and self.latest.get("safety", {}).get("state") != "RUNNING"
        ):
            failures.append(
                "safety state is not RUNNING: "
                f"{self.latest.get('safety', {}).get('state', 'missing')}"
            )
        if (
            self.latest.get("localization", {}).get("state")
            != "localized"
        ):
            failures.append(
                "localization state is not localized: "
                f"{self.latest.get('localization', {}).get('state', 'missing')}"
            )
        if (
            self.latest.get("guard", {}).get("hardware_path_absent")
            != "true"
        ):
            failures.append("hardware isolation guard is not healthy")
        observed_controller = self.latest.get(
            "controller",
            {},
        ).get("controller_type", "")
        if observed_controller and observed_controller != controller_type:
            failures.append(
                f"controller type is {observed_controller}, "
                f"expected {controller_type}"
            )
        observed_fault = self.latest.get("fault", {}).get("mode", "")
        if observed_fault and observed_fault != fault_mode:
            failures.append(
                f"fault mode is {observed_fault}, expected {fault_mode}"
            )
        if self.latest.get("fault", {}).get("active") == "true":
            failures.append("fault injector is already active")
        error_level = (
            DiagnosticStatus.ERROR[0]
            if isinstance(DiagnosticStatus.ERROR, (bytes, bytearray))
            else int(DiagnosticStatus.ERROR)
        )
        for key in ("lidar_transport", "encoder_transport"):
            level = self.latest.get(key, {}).get("level", error_level)
            if int(level) >= error_level:
                failures.append(
                    f"{key} is unhealthy: "
                    f"{self.latest.get(key, {}).get('state', 'missing')}"
                )
        forbidden = find_forbidden_nodes(self.get_node_names())
        if forbidden:
            failures.append(
                "forbidden ROS nodes are running: " + ", ".join(forbidden)
            )
        return failures

    def start_recording(self):
        for rows in (
            self.request_rows,
            self.safe_rows,
            self.controller_rows,
            self.safety_rows,
            self.localization_rows,
            self.guard_rows,
            self.fault_rows,
            self.transport_rows,
        ):
            rows.clear()
        self.started_monotonic = time.monotonic()
        self.recording = True

    def call_trigger_service(self, client, timeout_s=3.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"service is unavailable: {client.srv_name}")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_s,
        )
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service timed out: {client.srv_name}")
        response = future.result()
        if not response.success:
            raise RuntimeError(
                f"service rejected request: {client.srv_name}: "
                f"{response.message}"
            )
        return response.message


def _parse_args(args=None):
    raw_args = (
        sys.argv
        if args is None
        else ["controller_dry_run_trial"] + list(args)
    )
    user_args = remove_ros_args(args=raw_args)[1:]
    root = find_repository_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--controller-type",
        choices=("mpc", "mppi"),
        default="mppi",
    )
    parser.add_argument(
        "--fault-mode",
        choices=SUPPORTED_FAULT_MODES,
        default="none",
    )
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--fault-trigger-after-s", type=float, default=5.0)
    parser.add_argument("--preflight-timeout-s", type=float, default=15.0)
    parser.add_argument("--preflight-stable-s", type=float, default=2.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            root
            / "experiments/real_vehicle/results/"
            "phase6_controller_dry_run/trials"
        ),
    )
    parser.add_argument("--trial-name")
    return parser.parse_args(user_args), raw_args[0]


def _output_directory(options):
    base = options.output_dir.expanduser().resolve()
    name = options.trial_name or (
        f"{options.controller_type}_{options.fault_mode}_"
        + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    output = base / name
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(
            f"trial output directory is not empty: {output}"
        )
    output.mkdir(parents=True, exist_ok=True)
    return output


def _validate_options(options):
    if options.duration_s <= 0.0:
        raise SystemExit("--duration-s must be positive")
    if options.preflight_timeout_s <= 0.0:
        raise SystemExit("--preflight-timeout-s must be positive")
    if options.preflight_stable_s < 0.0:
        raise SystemExit("--preflight-stable-s must be non-negative")
    if options.preflight_stable_s >= options.preflight_timeout_s:
        raise SystemExit(
            "--preflight-stable-s must be less than "
            "--preflight-timeout-s"
        )
    if options.fault_mode != "none":
        if not 0.0 < options.fault_trigger_after_s < options.duration_s:
            raise SystemExit(
                "--fault-trigger-after-s must be inside the trial duration"
            )


def main(args=None):
    options, program_name = _parse_args(args)
    _validate_options(options)
    try:
        output = _output_directory(options)
    except FileExistsError as exc:
        print(f"試験を開始できません: {exc}", file=sys.stderr)
        print(
            "--trial-nameに未使用の名前を指定してください。",
            file=sys.stderr,
        )
        return 2

    rclpy.init(args=[program_name])
    node = ControllerDryRunTrialNode()
    preflight_failures = []
    interrupted = False
    runtime_failure = ""
    fault_trigger_elapsed_s = None
    actual_duration_s = 0.0
    fault_triggered = False
    preflight_reset_count = 0
    preflight_reset_errors = []
    preflight_stable_started = None
    last_preflight_reset = None
    try:
        preflight_started = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            failures = node.readiness_failures(
                options.controller_type,
                options.fault_mode,
            )
            non_safety_failures = node.readiness_failures(
                options.controller_type,
                options.fault_mode,
                require_safety_running=False,
            )
            safety = node.latest.get("safety", {})
            reset_due = (
                last_preflight_reset is None
                or now - last_preflight_reset >= 1.0
            )
            if (
                not non_safety_failures
                and reset_due
                and is_recoverable_preflight_fault(
                    safety.get("state", ""),
                    safety.get("fault_reason", ""),
                )
            ):
                try:
                    message = node.call_trigger_service(
                        node.safety_reset_client
                    )
                    preflight_reset_count += 1
                    last_preflight_reset = time.monotonic()
                    preflight_stable_started = None
                    print(
                        "開始前の回復可能FAULTを解除: "
                        f"{safety.get('fault_reason')} ({message})",
                        flush=True,
                    )
                except RuntimeError as exc:
                    preflight_reset_errors.append(str(exc))
                    last_preflight_reset = time.monotonic()
            elif not failures:
                if preflight_stable_started is None:
                    preflight_stable_started = now
                    print(
                        "開始前の通信安定確認: "
                        f"{options.preflight_stable_s:.1f} s",
                        flush=True,
                    )
                elif (
                    now - preflight_stable_started
                    >= options.preflight_stable_s
                ):
                    break
            else:
                preflight_stable_started = None
            if (
                now - preflight_started
                >= options.preflight_timeout_s
            ):
                preflight_failures = failures or [
                    "healthy preflight window did not reach "
                    f"{options.preflight_stable_s:.3f} s"
                ]
                preflight_failures.extend(preflight_reset_errors[-1:])
                break
            rclpy.spin_once(node, timeout_sec=0.10)

        if not preflight_failures:
            print(
                "ドライラン開始: 車体を動かさず、モーターバッテリーは"
                "接続しないでください。",
                flush=True,
            )
            node.start_recording()
            while rclpy.ok() and node.elapsed_s() < options.duration_s:
                if (
                    options.fault_mode != "none"
                    and not fault_triggered
                    and node.elapsed_s()
                    >= options.fault_trigger_after_s
                ):
                    trigger_started_s = node.elapsed_s()
                    message = node.call_trigger_service(
                        node.trigger_client
                    )
                    fault_trigger_elapsed_s = trigger_started_s
                    fault_triggered = True
                    print(
                        f"故障注入: {message} at "
                        f"{fault_trigger_elapsed_s:.3f} s",
                        flush=True,
                    )
                remaining = options.duration_s - node.elapsed_s()
                rclpy.spin_once(
                    node,
                    timeout_sec=min(0.05, max(remaining, 0.0)),
                )
    except KeyboardInterrupt:
        interrupted = True
    except RuntimeError as exc:
        runtime_failure = str(exc)
    finally:
        actual_duration_s = node.elapsed_s()
        node.recording = False
        if fault_triggered:
            try:
                node.call_trigger_service(node.clear_client)
                cleanup_end = time.monotonic() + 0.10
                while rclpy.ok() and time.monotonic() < cleanup_end:
                    rclpy.spin_once(node, timeout_sec=0.02)
            except RuntimeError as exc:
                if not runtime_failure:
                    runtime_failure = f"fault cleanup failed: {exc}"
            try:
                node.call_trigger_service(node.safety_reset_client)
            except RuntimeError as exc:
                if not runtime_failure:
                    runtime_failure = f"safety reset failed: {exc}"

    summary = summarize_controller_dry_run(
        node.request_rows,
        node.safe_rows,
        node.controller_rows,
        node.safety_rows,
        node.localization_rows,
        node.guard_rows,
        requested_duration_s=options.duration_s,
        actual_duration_s=actual_duration_s,
        controller_type=options.controller_type,
        fault_mode=options.fault_mode,
        fault_trigger_elapsed_s=fault_trigger_elapsed_s,
        interrupted=interrupted,
    )
    summary["preflight_ready"] = not preflight_failures
    summary["preflight_failures"] = preflight_failures
    summary["preflight_reset_count"] = preflight_reset_count
    summary["preflight_reset_errors"] = preflight_reset_errors
    summary["preflight_stable_s"] = options.preflight_stable_s
    summary["runtime_failure"] = runtime_failure
    summary["recording_format_version"] = 2
    summary["transport_status_samples"] = len(node.transport_rows)
    summary["fault_status_samples"] = len(node.fault_rows)
    summary["final_forbidden_nodes"] = find_forbidden_nodes(
        node.get_node_names()
    )
    if preflight_failures:
        summary["failures"].append(
            "preflight failed: " + "; ".join(preflight_failures)
        )
    if runtime_failure:
        summary["failures"].append(runtime_failure)
    if summary["final_forbidden_nodes"]:
        summary["failures"].append(
            "forbidden ROS nodes were present at trial end: "
            + ", ".join(summary["final_forbidden_nodes"])
        )
    if summary["failures"]:
        summary["status"] = "failed"

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    _write_csv(
        output / "control_request.csv",
        COMMAND_FIELDS,
        node.request_rows,
    )
    _write_csv(
        output / "control_safe.csv",
        COMMAND_FIELDS,
        node.safe_rows,
    )
    _write_csv(
        output / "controller_status.csv",
        CONTROLLER_FIELDS,
        node.controller_rows,
    )
    _write_csv(
        output / "safety_status.csv",
        SAFETY_FIELDS,
        node.safety_rows,
    )
    _write_csv(
        output / "localization_status.csv",
        LOCALIZATION_FIELDS,
        node.localization_rows,
    )
    _write_csv(
        output / "hardware_guard_status.csv",
        GUARD_FIELDS,
        node.guard_rows,
    )
    _write_csv(
        output / "fault_injector_status.csv",
        FAULT_FIELDS,
        node.fault_rows,
    )
    _write_csv(
        output / "transport_status.csv",
        TRANSPORT_FIELDS,
        node.transport_rows,
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
