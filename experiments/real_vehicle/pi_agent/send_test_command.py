#!/usr/bin/env python3
"""Send safe dry-run packets to a Raspberry Pi UDP actuator agent."""

import argparse
import socket
import time
import uuid

try:
    from .protocol import (
        ProtocolError,
        decode_status,
        encode_command,
        make_command,
    )
except ImportError:
    from protocol import ProtocolError, decode_status, encode_command, make_command


def receive_matching_status(sock, session_id, sequence_id, timeout_s=0.5):
    """Ignore periodic status packets until this command's reply arrives."""

    deadline = time.monotonic() + float(timeout_s)
    while True:
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0.0:
            return None
        sock.settimeout(remaining_s)
        try:
            status_packet, _sender = sock.recvfrom(2048)
        except socket.timeout:
            return None
        try:
            status = decode_status(status_packet)
        except ProtocolError:
            continue
        if (
            status["session_id"] == session_id
            and int(status["sequence_id"]) == int(sequence_id)
        ):
            return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--stop-count", type=int, default=5)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--motion-candidate", action="store_true")
    mode.add_argument("--steering-candidate", action="store_true")
    parser.add_argument("--esc-duty", type=float, default=10.16)
    parser.add_argument("--steering-duty", type=float, default=10.895)
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    if args.rate_hz <= 0.0:
        parser.error("--rate-hz must be positive")
    if args.stop_count <= 0:
        parser.error("--stop-count must be positive")
    session_id = uuid.uuid4().hex
    enable_output = bool(args.motion_candidate or args.steering_candidate)
    esc_duty = args.esc_duty if args.motion_candidate else 10.30
    steering_duty = args.steering_duty if enable_output else 10.895
    if args.motion_candidate:
        reason = "test_motion"
    elif args.steering_candidate:
        reason = "test_steering"
    else:
        reason = "safe_stop"
    received = 0
    sent = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("0.0.0.0", 0))
        sock.settimeout(0.5)
        sequence_id = 0
        try:
            for sequence_id in range(1, args.count + 1):
                packet = encode_command(make_command(
                    session_id=session_id,
                    sequence_id=sequence_id,
                    sent_at_unix_s=time.time(),
                    esc_duty_percent=esc_duty,
                    steering_duty_percent=steering_duty,
                    enable_output=enable_output,
                    reason=reason,
                ))
                sock.sendto(packet, (args.host, args.port))
                sent += 1
                try:
                    status = receive_matching_status(
                        sock,
                        session_id,
                        sequence_id,
                    )
                    if status is None:
                        raise socket.timeout
                    received += 1
                    print(
                        f"seq={status['sequence_id']} state={status['state']} "
                        f"reason={status['reason']} "
                        f"hardware={status['hardware_output_enabled']}",
                        flush=True,
                    )
                except socket.timeout:
                    print(f"seq={sequence_id} status_timeout", flush=True)
                time.sleep(1.0 / args.rate_hz)
        finally:
            for _index in range(args.stop_count):
                sequence_id += 1
                packet = encode_command(make_command(
                    session_id=session_id,
                    sequence_id=sequence_id,
                    sent_at_unix_s=time.time(),
                    esc_duty_percent=10.30,
                    steering_duty_percent=10.895,
                    enable_output=False,
                    reason="test_complete_stop",
                ))
                sock.sendto(packet, (args.host, args.port))
                sent += 1
                try:
                    status = receive_matching_status(
                        sock,
                        session_id,
                        sequence_id,
                    )
                    if status is None:
                        raise socket.timeout
                    received += 1
                    print(
                        f"stop_seq={status['sequence_id']} "
                        f"state={status['state']} reason={status['reason']}",
                        flush=True,
                    )
                except socket.timeout:
                    print(f"stop_seq={sequence_id} status_timeout", flush=True)
                time.sleep(1.0 / args.rate_hz)
    print(f"sent={sent} received={received}")
    if received == 0:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
