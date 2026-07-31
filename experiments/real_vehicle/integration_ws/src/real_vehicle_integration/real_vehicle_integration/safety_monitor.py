"""Watchdogs and command authorization independent of ROS 2."""

from dataclasses import dataclass

from .contracts import ContractLimits, neutral_control, validate_control
from .state_machine import SafetyState, SafetyStateMachine


@dataclass(frozen=True)
class SafetyConfig:
    command_timeout_s: float = 0.10
    pose_timeout_s: float = 0.20
    scan_timeout_s: float = 0.30
    limits: ContractLimits = ContractLimits()


class SafetyMonitor:
    def __init__(self, config=SafetyConfig()):
        self.config = config
        self.machine = SafetyStateMachine()
        self.last_scan_stamp_s = None
        self.last_pose_stamp_s = None
        self.last_command_stamp_s = None
        self.last_sequence_id = -1
        self.emergency_stop = False

    def update_scan(self, stamp_s):
        self.last_scan_stamp_s = float(stamp_s)

    def update_pose(self, stamp_s):
        self.last_pose_stamp_s = float(stamp_s)

    def set_emergency_stop(self, active):
        self.emergency_stop = bool(active)
        if self.emergency_stop:
            self.machine.fault("emergency_stop")

    def _sensor_problem(self, now_s):
        now_s = float(now_s)
        if self.last_scan_stamp_s is None:
            return "scan_missing"
        if (
            self.last_scan_stamp_s - now_s
            > self.config.limits.future_tolerance_s
        ):
            return "scan_timestamp_future"
        if now_s - self.last_scan_stamp_s > self.config.scan_timeout_s:
            return "scan_timeout"
        if self.last_pose_stamp_s is None:
            return "pose_missing"
        if (
            self.last_pose_stamp_s - now_s
            > self.config.limits.future_tolerance_s
        ):
            return "pose_timestamp_future"
        if now_s - self.last_pose_stamp_s > self.config.pose_timeout_s:
            return "pose_timeout"
        return ""

    def arm(self, now_s):
        problem = self._sensor_problem(now_s)
        if problem:
            raise RuntimeError(problem)
        if self.emergency_stop:
            raise RuntimeError("emergency_stop")
        self.machine.arm()

    def start(self, now_s):
        problem = self._sensor_problem(now_s)
        if problem:
            raise RuntimeError(problem)
        if self.emergency_stop:
            raise RuntimeError("emergency_stop")
        self.machine.start()

    def stop(self):
        self.machine.stop()

    def reset(self):
        self.machine.reset()

    def process(self, command, now_s):
        now_s = float(now_s)
        if self.emergency_stop:
            self.machine.fault("emergency_stop")
            return neutral_control(now_s, command.sequence_id, command.frame_id)
        try:
            command = validate_control(
                command,
                now_s,
                self.config.command_timeout_s,
                self.config.limits,
            )
        except (TypeError, ValueError) as exc:
            self.machine.fault(f"invalid_command:{exc}")
            return neutral_control(now_s, command.sequence_id, command.frame_id)
        if command.sequence_id <= self.last_sequence_id:
            self.machine.fault("non_increasing_sequence")
            return neutral_control(now_s, command.sequence_id, command.frame_id)
        if (
            self.last_command_stamp_s is not None
            and command.stamp_s <= self.last_command_stamp_s
        ):
            self.machine.fault("non_increasing_command_stamp")
            return neutral_control(now_s, command.sequence_id, command.frame_id)
        self.last_command_stamp_s = command.stamp_s
        self.last_sequence_id = command.sequence_id
        problem = self._sensor_problem(now_s)
        if problem and self.machine.state in (SafetyState.READY, SafetyState.RUNNING):
            self.machine.fault(problem)
        if not self.machine.output_enabled:
            return neutral_control(now_s, command.sequence_id, command.frame_id)
        return command

    def watchdog(self, now_s):
        now_s = float(now_s)
        if self.machine.state not in (SafetyState.READY, SafetyState.RUNNING):
            return self.machine.fault_reason
        problem = self._sensor_problem(now_s)
        if not problem and self.machine.state == SafetyState.RUNNING:
            if self.last_command_stamp_s is None:
                problem = "command_missing"
            elif now_s - self.last_command_stamp_s > self.config.command_timeout_s:
                problem = "command_timeout"
        if problem:
            self.machine.fault(problem)
        return problem
