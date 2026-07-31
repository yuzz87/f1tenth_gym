"""Explicit safety state machine for vehicle output authorization."""

from enum import Enum


class SafetyState(str, Enum):
    DISARMED = "DISARMED"
    READY = "READY"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    FAULT = "FAULT"


class SafetyStateMachine:
    def __init__(self):
        self.state = SafetyState.DISARMED
        self.fault_reason = ""

    def arm(self):
        if self.state not in (SafetyState.DISARMED, SafetyState.STOPPED):
            raise RuntimeError(f"cannot arm from {self.state.value}")
        self.state = SafetyState.READY
        self.fault_reason = ""

    def start(self):
        if self.state != SafetyState.READY:
            raise RuntimeError(f"cannot start from {self.state.value}")
        self.state = SafetyState.RUNNING

    def stop(self):
        if self.state in (SafetyState.READY, SafetyState.RUNNING):
            self.state = SafetyState.STOPPING

    def mark_stopped(self):
        if self.state not in (SafetyState.STOPPING, SafetyState.FAULT):
            raise RuntimeError(f"cannot mark stopped from {self.state.value}")
        self.state = SafetyState.STOPPED

    def fault(self, reason):
        self.fault_reason = str(reason) or "unknown_fault"
        self.state = SafetyState.FAULT

    def reset(self):
        if self.state not in (SafetyState.FAULT, SafetyState.STOPPED):
            raise RuntimeError(f"cannot reset from {self.state.value}")
        self.state = SafetyState.DISARMED
        self.fault_reason = ""

    @property
    def output_enabled(self):
        return self.state == SafetyState.RUNNING
