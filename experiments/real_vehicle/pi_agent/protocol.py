"""Versioned JSON protocol shared by the PC bridge and Raspberry Pi agent."""

import json
import math


PROTOCOL_VERSION = 1
MAX_PACKET_BYTES = 2048
COMMAND_KIND = "actuator_command"
STATUS_KIND = "actuator_status"


class ProtocolError(ValueError):
    """Raised when a UDP packet violates the actuator protocol."""


def _finite_float(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ProtocolError(f"{name} must be finite")
    return result


def _non_negative_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ProtocolError(f"{name} must be a non-negative integer")
    return int(value)


def _short_text(value, name, maximum=128):
    if not isinstance(value, str) or not value or len(value) > maximum:
        raise ProtocolError(f"{name} must be a non-empty string <= {maximum} chars")
    return value


def _optional_short_text(value, name, maximum=64):
    if value == "":
        return ""
    return _short_text(value, name, maximum)


def _boolean(value, name):
    if not isinstance(value, bool):
        raise ProtocolError(f"{name} must be a boolean")
    return value


def _decode_json(packet):
    if not isinstance(packet, (bytes, bytearray)):
        raise ProtocolError("packet must be bytes")
    if len(packet) > MAX_PACKET_BYTES:
        raise ProtocolError("packet is too large")
    try:
        message = json.loads(bytes(packet).decode("ascii"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"invalid JSON: {exc}") from exc
    if not isinstance(message, dict):
        raise ProtocolError("message must be a JSON object")
    return message


def _encode_json(message):
    packet = json.dumps(
        message,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("ascii")
    if len(packet) > MAX_PACKET_BYTES:
        raise ProtocolError("encoded packet is too large")
    return packet


def make_command(
    session_id,
    sequence_id,
    sent_at_unix_s,
    esc_duty_percent,
    steering_duty_percent,
    enable_output,
    reason,
):
    return {
        "version": PROTOCOL_VERSION,
        "kind": COMMAND_KIND,
        "session_id": _short_text(session_id, "session_id", 64),
        "sequence_id": _non_negative_int(sequence_id, "sequence_id"),
        "sent_at_unix_s": _finite_float(sent_at_unix_s, "sent_at_unix_s"),
        "esc_duty_percent": _finite_float(
            esc_duty_percent,
            "esc_duty_percent",
        ),
        "steering_duty_percent": _finite_float(
            steering_duty_percent,
            "steering_duty_percent",
        ),
        "enable_output": _boolean(enable_output, "enable_output"),
        "reason": _short_text(reason, "reason"),
    }


def encode_command(message):
    return _encode_json(decode_command(_encode_json(message)))


def decode_command(packet):
    message = _decode_json(packet)
    if message.get("version") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    if message.get("kind") != COMMAND_KIND:
        raise ProtocolError("unexpected message kind")
    return make_command(
        message.get("session_id"),
        message.get("sequence_id"),
        message.get("sent_at_unix_s"),
        message.get("esc_duty_percent"),
        message.get("steering_duty_percent"),
        message.get("enable_output"),
        message.get("reason"),
    )


def make_status(
    session_id,
    sequence_id,
    sent_at_unix_s,
    state,
    reason,
    esc_duty_percent,
    steering_duty_percent,
    hardware_output_enabled,
    output_armed,
    accepted_packets,
    rejected_packets,
    watchdog_count,
):
    return {
        "version": PROTOCOL_VERSION,
        "kind": STATUS_KIND,
        "session_id": _optional_short_text(session_id, "session_id"),
        "sequence_id": _non_negative_int(sequence_id, "sequence_id"),
        "sent_at_unix_s": _finite_float(sent_at_unix_s, "sent_at_unix_s"),
        "state": _short_text(state, "state"),
        "reason": _short_text(reason, "reason"),
        "esc_duty_percent": _finite_float(
            esc_duty_percent,
            "esc_duty_percent",
        ),
        "steering_duty_percent": _finite_float(
            steering_duty_percent,
            "steering_duty_percent",
        ),
        "hardware_output_enabled": _boolean(
            hardware_output_enabled,
            "hardware_output_enabled",
        ),
        "output_armed": _boolean(output_armed, "output_armed"),
        "accepted_packets": _non_negative_int(
            accepted_packets,
            "accepted_packets",
        ),
        "rejected_packets": _non_negative_int(
            rejected_packets,
            "rejected_packets",
        ),
        "watchdog_count": _non_negative_int(watchdog_count, "watchdog_count"),
    }


def encode_status(message):
    return _encode_json(decode_status(_encode_json(message)))


def decode_status(packet):
    message = _decode_json(packet)
    if message.get("version") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    if message.get("kind") != STATUS_KIND:
        raise ProtocolError("unexpected message kind")
    return make_status(
        message.get("session_id", ""),
        message.get("sequence_id", 0),
        message.get("sent_at_unix_s"),
        message.get("state"),
        message.get("reason"),
        message.get("esc_duty_percent"),
        message.get("steering_duty_percent"),
        message.get("hardware_output_enabled"),
        message.get("output_armed"),
        message.get("accepted_packets"),
        message.get("rejected_packets"),
        message.get("watchdog_count"),
    )
