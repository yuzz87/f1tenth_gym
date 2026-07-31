"""ROS-independent safety policy for UDP actuator commands."""

from collections import deque
from dataclasses import dataclass
import math

try:
    from .protocol import ProtocolError, decode_command
except ImportError:
    from protocol import ProtocolError, decode_command


@dataclass(frozen=True)
class AgentLimits:
    esc_neutral_duty_percent: float = 10.30
    esc_start_duty_percent: float = 10.16
    esc_forward_min_duty_percent: float = 10.10
    steering_neutral_duty_percent: float = 10.895
    steering_min_duty_percent: float = 9.05
    steering_max_duty_percent: float = 12.42
    message_max_age_s: float = 0.10
    future_tolerance_s: float = 0.10


@dataclass(frozen=True)
class AgentDecision:
    accepted: bool
    enable_output: bool
    esc_duty_percent: float
    steering_duty_percent: float
    reason: str
    session_id: str
    sequence_id: int


class SafeCommandPolicy:
    def __init__(self, limits=None):
        self.limits = limits or AgentLimits()
        self.active_session_id = None
        self.active_sender_ip = None
        self.last_sequence_id = -1
        self.closed_session_ids = deque(maxlen=16)

    def neutral(self, reason, session_id="", sequence_id=0, accepted=False):
        return AgentDecision(
            accepted=bool(accepted),
            enable_output=False,
            esc_duty_percent=self.limits.esc_neutral_duty_percent,
            steering_duty_percent=self.limits.steering_neutral_duty_percent,
            reason=str(reason),
            session_id=str(session_id),
            sequence_id=int(sequence_id),
        )

    def release_session(self):
        if self.active_session_id is not None:
            self.closed_session_ids.append(self.active_session_id)
        self.active_session_id = None
        self.active_sender_ip = None
        self.last_sequence_id = -1

    def process(self, packet, sender_ip, now_unix_s):
        try:
            message = decode_command(packet)
            self._validate_transport(message, str(sender_ip), float(now_unix_s))
            decision = self._validate_output(message)
        except (ProtocolError, TypeError, ValueError) as exc:
            return self.neutral(f"invalid_command:{exc}")

        self.active_session_id = message["session_id"]
        self.active_sender_ip = str(sender_ip)
        self.last_sequence_id = message["sequence_id"]
        return decision

    def _validate_transport(self, message, sender_ip, now_unix_s):
        age_s = now_unix_s - message["sent_at_unix_s"]
        if age_s > self.limits.message_max_age_s:
            raise ProtocolError("message is stale")
        if age_s < -self.limits.future_tolerance_s:
            raise ProtocolError("message is from the future")
        if message["session_id"] in self.closed_session_ids:
            raise ProtocolError("session is already closed")
        if self.active_sender_ip is not None and sender_ip != self.active_sender_ip:
            raise ProtocolError("sender conflict")
        if (
            self.active_session_id is not None
            and message["session_id"] != self.active_session_id
        ):
            raise ProtocolError("session conflict")
        if message["sequence_id"] <= self.last_sequence_id:
            raise ProtocolError("sequence is not increasing")

    def _validate_output(self, message):
        session_id = message["session_id"]
        sequence_id = message["sequence_id"]
        if not message["enable_output"]:
            return self.neutral(
                message["reason"],
                session_id,
                sequence_id,
                accepted=True,
            )

        esc_duty = message["esc_duty_percent"]
        esc_is_neutral = math.isclose(
            esc_duty,
            self.limits.esc_neutral_duty_percent,
            abs_tol=1e-6,
        )
        esc_is_motion = (
            self.limits.esc_forward_min_duty_percent
            <= esc_duty
            <= self.limits.esc_start_duty_percent
        )
        if not esc_is_neutral and not esc_is_motion:
            raise ProtocolError("ESC duty is outside neutral and motion limits")
        steering_duty = message["steering_duty_percent"]
        if not (
            self.limits.steering_min_duty_percent
            <= steering_duty
            <= self.limits.steering_max_duty_percent
        ):
            raise ProtocolError("steering duty is outside limits")
        return AgentDecision(
            accepted=True,
            enable_output=True,
            esc_duty_percent=esc_duty,
            steering_duty_percent=steering_duty,
            reason=message["reason"],
            session_id=session_id,
            sequence_id=sequence_id,
        )
