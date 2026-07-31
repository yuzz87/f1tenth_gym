"""Send deterministic synthetic LiDAR UDP packets without ROS or hardware."""

import argparse
import math
import random
import socket
import time

from ..lidar_transport import encode_scan_packet, make_scan_packets


SECTOR_INDEX = {
    "rear": 0,
    "right": 90,
    "front": 180,
    "left": 270,
}


def build_ranges(sector, distance_m):
    ranges = [math.inf] * 360
    center = SECTOR_INDEX[sector]
    for offset in range(-2, 3):
        ranges[(center + offset) % 360] = distance_m + 0.05 * abs(offset)
    return ranges


def parse_args(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("host")
    parser.add_argument("--port", type=int, default=5010)
    parser.add_argument("--scans", type=int, default=20)
    parser.add_argument("--rate-hz", type=float, default=5.5)
    parser.add_argument(
        "--sector",
        choices=tuple(SECTOR_INDEX),
        default="front",
    )
    parser.add_argument("--distance-m", type=float, default=1.0)
    parser.add_argument("--reverse-chunks", action="store_true")
    parser.add_argument("--drop-chunk", type=int, default=-1)
    parser.add_argument("--corrupt-scan", type=int, default=-1)
    return parser.parse_args(args)


def main(args=None):
    options = parse_args(args)
    if options.scans < 1 or options.rate_hz <= 0.0:
        raise SystemExit("--scans and --rate-hz must be positive")
    destination = (socket.gethostbyname(options.host), options.port)
    session_id = random.SystemRandom().randrange(1, 0xFFFFFFFF)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period_s = 1.0 / options.rate_hz
    ranges = build_ranges(options.sector, options.distance_m)
    try:
        for scan_id in range(1, options.scans + 1):
            packets = list(make_scan_packets(
                ranges,
                session_id=session_id,
                scan_id=scan_id,
                capture_time_ns=time.time_ns(),
                scan_period_us=round(period_s * 1e6),
                raw_points=1047,
            ))
            if options.reverse_chunks:
                packets.reverse()
            for packet in packets:
                if packet.chunk_index == options.drop_chunk:
                    continue
                datagram = bytearray(encode_scan_packet(packet))
                if scan_id == options.corrupt_scan and packet.chunk_index == 0:
                    datagram[-1] ^= 1
                udp_socket.sendto(datagram, destination)
            print(
                f"scan={scan_id} session={session_id} "
                f"sector={options.sector} destination={destination[0]}:{destination[1]}"
            )
            time.sleep(period_s)
    finally:
        udp_socket.close()
