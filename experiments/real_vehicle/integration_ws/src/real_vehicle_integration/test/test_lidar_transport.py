import math

import pytest

from real_vehicle_integration.lidar_analysis import (
    mask_near_field_sector,
    sector_minima,
)
from real_vehicle_integration.lidar_transport import (
    FLAG_HEALTH_OK,
    FLAG_NORMALIZED,
    LidarProtocolError,
    ScanReassembler,
    decode_scan_packet,
    encode_scan_packet,
    make_scan_packets,
    normalize_measurements,
    resample_periodic_scan,
)


def _packets(scan_id=7):
    ranges = [math.inf] * 360
    ranges[0] = 1.0
    ranges[180] = 2.0
    return make_scan_packets(
        ranges,
        session_id=11,
        scan_id=scan_id,
        capture_time_ns=123456789,
        scan_period_us=181818,
        raw_points=1047,
    )


def test_scan_packet_round_trip_and_crc_rejection():
    packet = _packets()[0]
    encoded = encode_scan_packet(packet)
    restored = decode_scan_packet(encoded)
    assert restored.session_id == packet.session_id
    assert restored.scan_id == packet.scan_id
    assert restored.ranges == packet.ranges
    assert restored.angle_min_rad == pytest.approx(packet.angle_min_rad)
    assert restored.angle_increment_rad == pytest.approx(
        packet.angle_increment_rad
    )
    assert restored.range_min_m == pytest.approx(packet.range_min_m)
    assert restored.range_max_m == pytest.approx(packet.range_max_m)
    corrupted = bytearray(encoded)
    corrupted[-1] ^= 1
    with pytest.raises(LidarProtocolError, match="CRC"):
        decode_scan_packet(corrupted)


def test_reassembler_accepts_out_of_order_chunks():
    reassembler = ScanReassembler(timeout_s=0.2)
    result = None
    for packet in reversed(_packets()):
        result = reassembler.add(packet, now_s=1.0) or result
    assert result is not None
    assert len(result.ranges) == 360
    assert result.ranges[0] == pytest.approx(1.0)
    assert result.ranges[180] == pytest.approx(2.0)
    assert result.valid_beams == 2
    assert result.raw_points == 1047
    assert result.flags == FLAG_HEALTH_OK | FLAG_NORMALIZED


def test_reassembler_counts_duplicates_timeouts_and_sequence_gaps():
    reassembler = ScanReassembler(timeout_s=0.1)
    first = _packets(scan_id=2)[0]
    assert reassembler.add(first, now_s=1.0) is None
    assert reassembler.add(first, now_s=1.01) is None
    assert reassembler.duplicate_packets == 1
    assert reassembler.expire(now_s=1.2) == 1
    assert reassembler.incomplete_scans == 1
    for packet in _packets(scan_id=3):
        reassembler.add(packet, now_s=2.0)
    for packet in _packets(scan_id=5):
        result = reassembler.add(packet, now_s=3.0)
    assert result.scan_id == 5
    assert reassembler.sequence_gaps == 1


def test_normalization_uses_nearest_bin_and_nearest_obstacle():
    ranges = normalize_measurements([
        (0.0, 3000.0),
        (0.2, 1000.0),
        (90.0, 2000.0),
        (180.0, 100.0),
        (270.0, 13000.0),
        (45.0, 0.0),
    ])
    assert len(ranges) == 360
    assert ranges[180] == pytest.approx(1.0)
    assert ranges[270] == pytest.approx(2.0)
    assert math.isinf(ranges[0])
    assert math.isinf(ranges[90])


def test_normalization_supports_angle_inversion_and_offset():
    inverted = normalize_measurements([(90.0, 1000.0)], angle_inverted=True)
    offset = normalize_measurements(
        [(0.0, 1000.0)],
        angle_offset_rad=math.pi / 2.0,
    )
    assert inverted[90] == pytest.approx(1.0)
    assert offset[270] == pytest.approx(1.0)


def test_periodic_resampling_maps_positive_pi_to_negative_pi():
    source = list(range(360))
    target = resample_periodic_scan(
        source,
        -math.pi,
        2.0 * math.pi / 360.0,
        (-math.pi, 0.0, math.pi),
    )
    assert target == (0.0, 180.0, 0.0)


def test_sector_minima_uses_ros_counterclockwise_convention():
    ranges = [math.inf] * 360
    ranges[180] = 1.0
    ranges[270] = 2.0
    ranges[0] = 3.0
    ranges[90] = 4.0
    minima = sector_minima(
        ranges,
        -math.pi,
        2.0 * math.pi / 360.0,
        0.15,
        12.0,
        half_width_rad=math.radians(2.0),
    )
    assert minima == {
        "front": 1.0,
        "left": 2.0,
        "rear": 3.0,
        "right": 4.0,
    }


def test_near_field_sector_mask_removes_only_close_rear_returns():
    ranges = [math.inf] * 360
    ranges[0] = 0.205
    ranges[10] = 0.25
    ranges[20] = 1.0
    ranges[180] = 0.20
    ranges[350] = 0.20
    masked, count = mask_near_field_sector(
        ranges,
        -math.pi,
        2.0 * math.pi / 360.0,
        -math.pi,
        math.radians(30.0),
        0.30,
    )
    assert count == 3
    assert math.isinf(masked[0])
    assert math.isinf(masked[10])
    assert masked[20] == pytest.approx(1.0)
    assert masked[180] == pytest.approx(0.20)
    assert math.isinf(masked[350])


def test_full_rear_sector_mask_preserves_front_and_side_ranges():
    ranges = [math.inf] * 360
    ranges[0] = 11.0
    ranges[20] = 4.0
    ranges[90] = 2.0
    ranges[180] = 1.0
    ranges[270] = 3.0
    ranges[340] = 5.0
    masked, count = mask_near_field_sector(
        ranges,
        -math.pi,
        2.0 * math.pi / 360.0,
        -math.pi,
        math.radians(30.0),
        12.0,
    )
    assert count == 3
    assert math.isinf(masked[0])
    assert math.isinf(masked[20])
    assert math.isinf(masked[340])
    assert masked[90] == pytest.approx(2.0)
    assert masked[180] == pytest.approx(1.0)
    assert masked[270] == pytest.approx(3.0)
