"""CRC-protected UDP protocol for encoder samples."""

import json
import math
import zlib


PROTOCOL_VERSION = 1
ENCODER_SAMPLE_KIND = "encoder_sample"
MAX_PACKET_BYTES = 4096


class ProtocolError(ValueError):
    """Raised when an encoder packet violates the data contract."""


def _finite_float(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ProtocolError(f"{name} must be finite")
    if minimum is not None and result < minimum:
        raise ProtocolError(f"{name} must be >= {minimum}")
    return result


def _integer(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(f"{name} must be an integer")
    if minimum is not None and value < minimum:
        raise ProtocolError(f"{name} must be >= {minimum}")
    return int(value)


def _short_text(value, name, maximum=128):
    if not isinstance(value, str) or not value or len(value) > maximum:
        raise ProtocolError(f"{name} must be a non-empty string <= {maximum} chars")
    return value


def _boolean(value, name):
    if not isinstance(value, bool):
        raise ProtocolError(f"{name} must be a boolean")
    return value


def _gpio_level(value, name):
    value = _integer(value, name)
    if value not in (0, 1):
        raise ProtocolError(f"{name} must be 0 or 1")
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


def _sample_without_crc(message):
    return {
        "version": PROTOCOL_VERSION,
        "kind": ENCODER_SAMPLE_KIND,
        "session_id": _short_text(message.get("session_id"), "session_id"),
        "sequence": _integer(message.get("sequence"), "sequence", 0),
        "source_monotonic_ns": _integer(
            message.get("source_monotonic_ns"),
            "source_monotonic_ns",
            0,
        ),
        "cumulative_count": _integer(
            message.get("cumulative_count"),
            "cumulative_count",
        ),
        "delta_count": _integer(message.get("delta_count"), "delta_count"),
        "direction": _integer(message.get("direction"), "direction"),
        "sample_period_s": _finite_float(
            message.get("sample_period_s"),
            "sample_period_s",
            0.0,
        ),
        "pulse_age_s": _finite_float(
            message.get("pulse_age_s"),
            "pulse_age_s",
            0.0,
        ),
        "invalid_transition_count": _integer(
            message.get("invalid_transition_count"),
            "invalid_transition_count",
            0,
        ),
        "gpio_a": _gpio_level(message.get("gpio_a"), "gpio_a"),
        "gpio_b": _gpio_level(message.get("gpio_b"), "gpio_b"),
        "encoder_healthy": _boolean(
            message.get("encoder_healthy"),
            "encoder_healthy",
        ),
    }


def make_sample(
    session_id,
    sequence,
    source_monotonic_ns,
    cumulative_count,
    delta_count,
    direction,
    sample_period_s,
    pulse_age_s,
    invalid_transition_count,
    gpio_a,
    gpio_b,
    encoder_healthy,
):
    message = {
        "session_id": session_id,
        "sequence": sequence,
        "source_monotonic_ns": source_monotonic_ns,
        "cumulative_count": cumulative_count,
        "delta_count": delta_count,
        "direction": direction,
        "sample_period_s": sample_period_s,
        "pulse_age_s": pulse_age_s,
        "invalid_transition_count": invalid_transition_count,
        "gpio_a": gpio_a,
        "gpio_b": gpio_b,
        "encoder_healthy": encoder_healthy,
    }
    validated = _sample_without_crc(message)
    if validated["direction"] not in (-1, 0, 1):
        raise ProtocolError("direction must be -1, 0, or 1")
    return validated


def _checksum(message):
    return f"{zlib.crc32(_encode_json(message)) & 0xFFFFFFFF:08x}"


def encode_sample(message):
    """Encode and append the CRC32 checksum."""

    base = make_sample(
        session_id=message.get("session_id"),
        sequence=message.get("sequence"),
        source_monotonic_ns=message.get("source_monotonic_ns"),
        cumulative_count=message.get("cumulative_count"),
        delta_count=message.get("delta_count"),
        direction=message.get("direction"),
        sample_period_s=message.get("sample_period_s"),
        pulse_age_s=message.get("pulse_age_s"),
        invalid_transition_count=message.get("invalid_transition_count"),
        gpio_a=message.get("gpio_a"),
        gpio_b=message.get("gpio_b"),
        encoder_healthy=message.get("encoder_healthy"),
    )
    encoded = dict(base)
    encoded["crc32"] = _checksum(base)
    return _encode_json(encoded)


def decode_sample(packet):
    """Validate JSON fields and CRC32, then return the decoded sample."""

    message = _decode_json(packet)
    crc = message.get("crc32")
    if not isinstance(crc, str) or len(crc) != 8:
        raise ProtocolError("crc32 must be an 8-character hexadecimal string")
    base = _sample_without_crc(message)
    expected = _checksum(base)
    if crc.lower() != expected:
        raise ProtocolError("CRC32 mismatch")
    base["crc32"] = crc.lower()
    return base

