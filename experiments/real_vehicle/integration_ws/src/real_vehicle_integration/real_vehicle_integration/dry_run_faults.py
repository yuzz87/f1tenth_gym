"""Deterministic gate used to inject Phase 6 controller safety faults."""

from collections import Counter


SUPPORTED_FAULT_MODES = (
    "none",
    "scan_timeout",
    "pose_timeout",
    "control_timeout",
    "emergency_stop",
)

FAULT_STREAM_BY_MODE = {
    "scan_timeout": "scan",
    "pose_timeout": "pose",
    "control_timeout": "control",
}


class DryRunFaultGate:
    """Pass messages until an explicitly configured fault is triggered."""

    def __init__(self, mode="none"):
        mode = str(mode).strip().lower()
        if mode not in SUPPORTED_FAULT_MODES:
            choices = ", ".join(SUPPORTED_FAULT_MODES)
            raise ValueError(f"fault mode must be one of: {choices}")
        self.mode = mode
        self.active = False
        self.trigger_count = 0
        self.clear_count = 0
        self.received = Counter()
        self.relayed = Counter()
        self.dropped = Counter()

    @property
    def emergency_stop_active(self):
        return self.active and self.mode == "emergency_stop"

    def trigger(self):
        if self.mode == "none":
            raise RuntimeError("fault mode is none")
        self.active = True
        self.trigger_count += 1

    def clear(self):
        self.active = False
        self.clear_count += 1

    def should_relay(self, stream):
        stream = str(stream)
        if stream not in ("scan", "pose", "control"):
            raise ValueError(f"unsupported stream: {stream}")
        self.received[stream] += 1
        blocked_stream = FAULT_STREAM_BY_MODE.get(self.mode)
        allowed = not (self.active and stream == blocked_stream)
        if allowed:
            self.relayed[stream] += 1
        else:
            self.dropped[stream] += 1
        return allowed

    def statistics(self):
        values = {
            "mode": self.mode,
            "active": self.active,
            "emergency_stop_active": self.emergency_stop_active,
            "trigger_count": self.trigger_count,
            "clear_count": self.clear_count,
        }
        for stream in ("scan", "pose", "control"):
            values[f"{stream}_received"] = self.received[stream]
            values[f"{stream}_relayed"] = self.relayed[stream]
            values[f"{stream}_dropped"] = self.dropped[stream]
        return values


__all__ = [
    "DryRunFaultGate",
    "FAULT_STREAM_BY_MODE",
    "SUPPORTED_FAULT_MODES",
]
