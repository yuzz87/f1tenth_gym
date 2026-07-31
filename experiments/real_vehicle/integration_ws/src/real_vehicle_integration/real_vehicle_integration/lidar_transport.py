"""ROS-independent transport and geometry helpers for the real LiDAR."""

from dataclasses import dataclass
import math
import struct
import time
from typing import Dict, Iterable, Optional, Sequence, Tuple
import zlib


MAGIC = b"RVLD"
PROTOCOL_VERSION = 1
MESSAGE_TYPE_SCAN = 1
FLAG_HEALTH_OK = 1 << 0
FLAG_NORMALIZED = 1 << 1
FLAG_ANGLE_INVERTED = 1 << 2
HEADER = struct.Struct("!4sBBHIIHHHHHHQIHHffffI")
HEADER_SIZE = HEADER.size
CRC_OFFSET = HEADER_SIZE - 4
DEFAULT_BEAMS_PER_PACKET = 120


class LidarProtocolError(ValueError):
    """Raised when a LiDAR UDP datagram violates the protocol contract."""


@dataclass(frozen=True)
class ScanPacket:
    session_id: int
    scan_id: int
    chunk_index: int
    chunk_count: int
    beam_start: int
    total_beams: int
    flags: int
    capture_time_ns: int
    scan_period_us: int
    valid_beams: int
    raw_points: int
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    ranges: Tuple[float, ...]


@dataclass(frozen=True)
class ReassembledScan:
    session_id: int
    scan_id: int
    flags: int
    capture_time_ns: int
    scan_period_us: int
    valid_beams: int
    raw_points: int
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    ranges: Tuple[float, ...]
    received_at_monotonic_s: float


def _validate_packet(packet):
    if not 0 <= int(packet.session_id) <= 0xFFFFFFFF:
        raise LidarProtocolError("session_id is outside uint32")
    if not 0 <= int(packet.scan_id) <= 0xFFFFFFFF:
        raise LidarProtocolError("scan_id is outside uint32")
    if packet.chunk_count < 1:
        raise LidarProtocolError("chunk_count must be positive")
    if not 0 <= packet.chunk_index < packet.chunk_count:
        raise LidarProtocolError("chunk_index is outside chunk_count")
    if packet.total_beams < 1 or packet.total_beams > 8192:
        raise LidarProtocolError("total_beams is outside supported range")
    if not packet.ranges:
        raise LidarProtocolError("scan packet contains no ranges")
    if packet.beam_start < 0:
        raise LidarProtocolError("beam_start must not be negative")
    if packet.beam_start + len(packet.ranges) > packet.total_beams:
        raise LidarProtocolError("scan packet extends beyond total_beams")
    if packet.valid_beams < 0 or packet.valid_beams > packet.total_beams:
        raise LidarProtocolError("valid_beams is outside total_beams")
    if packet.raw_points < 0 or packet.raw_points > 0xFFFF:
        raise LidarProtocolError("raw_points is outside uint16")
    geometry = (
        packet.angle_min_rad,
        packet.angle_increment_rad,
        packet.range_min_m,
        packet.range_max_m,
    )
    if not all(math.isfinite(float(value)) for value in geometry):
        raise LidarProtocolError("scan geometry must be finite")
    if packet.angle_increment_rad <= 0.0:
        raise LidarProtocolError("angle_increment must be positive")
    if packet.range_min_m < 0.0 or packet.range_min_m >= packet.range_max_m:
        raise LidarProtocolError("invalid LiDAR range limits")


def _header_values(packet, checksum):
    return (
        MAGIC,
        PROTOCOL_VERSION,
        MESSAGE_TYPE_SCAN,
        HEADER_SIZE,
        int(packet.session_id),
        int(packet.scan_id),
        int(packet.chunk_index),
        int(packet.chunk_count),
        int(packet.beam_start),
        len(packet.ranges),
        int(packet.total_beams),
        int(packet.flags),
        int(packet.capture_time_ns),
        int(packet.scan_period_us),
        int(packet.valid_beams),
        int(packet.raw_points),
        float(packet.angle_min_rad),
        float(packet.angle_increment_rad),
        float(packet.range_min_m),
        float(packet.range_max_m),
        int(checksum),
    )


def encode_scan_packet(packet):
    """Encode one scan chunk using network byte order and packet CRC32."""

    _validate_packet(packet)
    payload = struct.pack("!%df" % len(packet.ranges), *packet.ranges)
    header = HEADER.pack(*_header_values(packet, 0))
    checksum = zlib.crc32(header + payload) & 0xFFFFFFFF
    return HEADER.pack(*_header_values(packet, checksum)) + payload


def decode_scan_packet(datagram):
    """Validate and decode one LiDAR UDP datagram."""

    if len(datagram) < HEADER_SIZE:
        raise LidarProtocolError("datagram is shorter than the header")
    values = HEADER.unpack(datagram[:HEADER_SIZE])
    (
        magic,
        version,
        message_type,
        header_size,
        session_id,
        scan_id,
        chunk_index,
        chunk_count,
        beam_start,
        beam_count,
        total_beams,
        flags,
        capture_time_ns,
        scan_period_us,
        valid_beams,
        raw_points,
        angle_min_rad,
        angle_increment_rad,
        range_min_m,
        range_max_m,
        checksum,
    ) = values
    if magic != MAGIC:
        raise LidarProtocolError("invalid LiDAR packet magic")
    if version != PROTOCOL_VERSION:
        raise LidarProtocolError("unsupported LiDAR protocol version")
    if message_type != MESSAGE_TYPE_SCAN:
        raise LidarProtocolError("unsupported LiDAR message type")
    if header_size != HEADER_SIZE:
        raise LidarProtocolError("unexpected LiDAR header size")
    expected_size = HEADER_SIZE + 4 * beam_count
    if len(datagram) != expected_size:
        raise LidarProtocolError("datagram length does not match beam_count")
    checksum_input = bytearray(datagram)
    checksum_input[CRC_OFFSET:HEADER_SIZE] = b"\x00\x00\x00\x00"
    actual_checksum = zlib.crc32(checksum_input) & 0xFFFFFFFF
    if checksum != actual_checksum:
        raise LidarProtocolError("LiDAR packet CRC mismatch")
    ranges = struct.unpack("!%df" % beam_count, datagram[HEADER_SIZE:])
    packet = ScanPacket(
        session_id=session_id,
        scan_id=scan_id,
        chunk_index=chunk_index,
        chunk_count=chunk_count,
        beam_start=beam_start,
        total_beams=total_beams,
        flags=flags,
        capture_time_ns=capture_time_ns,
        scan_period_us=scan_period_us,
        valid_beams=valid_beams,
        raw_points=raw_points,
        angle_min_rad=angle_min_rad,
        angle_increment_rad=angle_increment_rad,
        range_min_m=range_min_m,
        range_max_m=range_max_m,
        ranges=tuple(ranges),
    )
    _validate_packet(packet)
    return packet


def make_scan_packets(
    ranges,
    session_id,
    scan_id,
    capture_time_ns,
    scan_period_us,
    raw_points,
    angle_min_rad=-math.pi,
    angle_increment_rad=2.0 * math.pi / 360.0,
    range_min_m=0.15,
    range_max_m=12.0,
    flags=FLAG_HEALTH_OK | FLAG_NORMALIZED,
    beams_per_packet=DEFAULT_BEAMS_PER_PACKET,
):
    """Split one normalized scan into independently validated packets."""

    ranges = tuple(float(value) for value in ranges)
    if not ranges:
        raise LidarProtocolError("cannot packetize an empty scan")
    beams_per_packet = int(beams_per_packet)
    if beams_per_packet < 1 or beams_per_packet > 300:
        raise LidarProtocolError("beams_per_packet must be in [1, 300]")
    chunk_count = int(math.ceil(len(ranges) / float(beams_per_packet)))
    valid_beams = sum(
        math.isfinite(value) and range_min_m <= value <= range_max_m
        for value in ranges
    )
    packets = []
    for chunk_index in range(chunk_count):
        beam_start = chunk_index * beams_per_packet
        chunk = ranges[beam_start:beam_start + beams_per_packet]
        packets.append(ScanPacket(
            session_id=int(session_id),
            scan_id=int(scan_id),
            chunk_index=chunk_index,
            chunk_count=chunk_count,
            beam_start=beam_start,
            total_beams=len(ranges),
            flags=int(flags),
            capture_time_ns=int(capture_time_ns),
            scan_period_us=int(scan_period_us),
            valid_beams=int(valid_beams),
            raw_points=int(raw_points),
            angle_min_rad=float(angle_min_rad),
            angle_increment_rad=float(angle_increment_rad),
            range_min_m=float(range_min_m),
            range_max_m=float(range_max_m),
            ranges=tuple(chunk),
        ))
    return tuple(packets)


class _PendingScan:
    def __init__(self, packet, now_s):
        self.created_s = now_s
        self.last_update_s = now_s
        self.metadata = (
            packet.chunk_count,
            packet.total_beams,
            packet.flags,
            packet.capture_time_ns,
            packet.scan_period_us,
            packet.valid_beams,
            packet.raw_points,
            packet.angle_min_rad,
            packet.angle_increment_rad,
            packet.range_min_m,
            packet.range_max_m,
        )
        self.chunks = {}


class ScanReassembler:
    """Reassemble out-of-order scan chunks and account for transport faults."""

    def __init__(self, timeout_s=0.25, maximum_pending_scans=32):
        self.timeout_s = float(timeout_s)
        self.maximum_pending_scans = int(maximum_pending_scans)
        if self.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if self.maximum_pending_scans < 1:
            raise ValueError("maximum_pending_scans must be positive")
        self.pending = {}
        self.completed_scans = 0
        self.incomplete_scans = 0
        self.duplicate_packets = 0
        self.metadata_errors = 0
        self.sequence_gaps = 0
        self._last_completed_by_session = {}

    def expire(self, now_s=None):
        now_s = time.monotonic() if now_s is None else float(now_s)
        expired = [
            key for key, state in self.pending.items()
            if now_s - state.last_update_s > self.timeout_s
        ]
        for key in expired:
            del self.pending[key]
            self.incomplete_scans += 1
        return len(expired)

    def add(self, packet, now_s=None):
        now_s = time.monotonic() if now_s is None else float(now_s)
        self.expire(now_s)
        key = (packet.session_id, packet.scan_id)
        state = self.pending.get(key)
        if state is None:
            if len(self.pending) >= self.maximum_pending_scans:
                oldest = min(
                    self.pending,
                    key=lambda candidate: self.pending[candidate].last_update_s,
                )
                del self.pending[oldest]
                self.incomplete_scans += 1
            state = _PendingScan(packet, now_s)
            self.pending[key] = state
        metadata = (
            packet.chunk_count,
            packet.total_beams,
            packet.flags,
            packet.capture_time_ns,
            packet.scan_period_us,
            packet.valid_beams,
            packet.raw_points,
            packet.angle_min_rad,
            packet.angle_increment_rad,
            packet.range_min_m,
            packet.range_max_m,
        )
        if metadata != state.metadata:
            self.metadata_errors += 1
            del self.pending[key]
            raise LidarProtocolError("inconsistent metadata across scan chunks")
        state.last_update_s = now_s
        if packet.chunk_index in state.chunks:
            if state.chunks[packet.chunk_index] != packet:
                self.metadata_errors += 1
                del self.pending[key]
                raise LidarProtocolError("conflicting duplicate LiDAR chunk")
            self.duplicate_packets += 1
            return None
        state.chunks[packet.chunk_index] = packet
        if len(state.chunks) != packet.chunk_count:
            return None

        output = [None] * packet.total_beams
        for chunk in state.chunks.values():
            for offset, value in enumerate(chunk.ranges):
                index = chunk.beam_start + offset
                if output[index] is not None:
                    del self.pending[key]
                    self.metadata_errors += 1
                    raise LidarProtocolError("overlapping LiDAR chunks")
                output[index] = float(value)
        if any(value is None for value in output):
            del self.pending[key]
            self.incomplete_scans += 1
            raise LidarProtocolError("complete chunk set leaves missing beams")
        del self.pending[key]
        previous = self._last_completed_by_session.get(packet.session_id)
        if previous is not None and packet.scan_id > previous + 1:
            self.sequence_gaps += packet.scan_id - previous - 1
        if previous is None or packet.scan_id > previous:
            self._last_completed_by_session[packet.session_id] = packet.scan_id
        self.completed_scans += 1
        return ReassembledScan(
            session_id=packet.session_id,
            scan_id=packet.scan_id,
            flags=packet.flags,
            capture_time_ns=packet.capture_time_ns,
            scan_period_us=packet.scan_period_us,
            valid_beams=packet.valid_beams,
            raw_points=packet.raw_points,
            angle_min_rad=packet.angle_min_rad,
            angle_increment_rad=packet.angle_increment_rad,
            range_min_m=packet.range_min_m,
            range_max_m=packet.range_max_m,
            ranges=tuple(output),
            received_at_monotonic_s=now_s,
        )


def normalize_measurements(
    measurements,
    num_beams=360,
    range_min_m=0.15,
    range_max_m=12.0,
    angle_inverted=False,
    angle_offset_rad=0.0,
):
    """Convert raw ``(angle_deg, distance_mm)`` pairs into periodic bins."""

    num_beams = int(num_beams)
    if num_beams < 1:
        raise ValueError("num_beams must be positive")
    range_min_m = float(range_min_m)
    range_max_m = float(range_max_m)
    if range_min_m < 0.0 or range_min_m >= range_max_m:
        raise ValueError("invalid range limits")
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / num_beams
    ranges = [math.inf] * num_beams
    direction = -1.0 if angle_inverted else 1.0
    for angle_deg, distance_mm in measurements:
        distance_m = float(distance_mm) / 1000.0
        if not math.isfinite(distance_m):
            continue
        if distance_m < range_min_m or distance_m > range_max_m:
            continue
        angle = direction * math.radians(float(angle_deg)) + float(angle_offset_rad)
        angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
        position = (angle - angle_min) / angle_increment
        index = int(math.floor(position + 0.5)) % num_beams
        ranges[index] = min(ranges[index], distance_m)
    return tuple(ranges)


def resample_periodic_scan(
    ranges,
    source_angle_min_rad,
    source_angle_increment_rad,
    target_angles_rad,
):
    """Nearest-neighbor resample of a full-circle scan onto target angles."""

    ranges = tuple(float(value) for value in ranges)
    if not ranges:
        raise ValueError("source scan is empty")
    increment = float(source_angle_increment_rad)
    if not math.isfinite(increment) or increment == 0.0:
        raise ValueError("source angle increment must be finite and non-zero")
    source_min = float(source_angle_min_rad)
    output = []
    for target in target_angles_rad:
        position = (float(target) - source_min) / increment
        position %= len(ranges)
        index = int(math.floor(position + 0.5)) % len(ranges)
        output.append(ranges[index])
    return tuple(output)
