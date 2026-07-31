#!/usr/bin/env python3
"""PC-side UDP receiver and monitor for Phase 1 encoder samples."""

import argparse
import csv
import os
import socket
import sys
import time

try:
    from .protocol import ProtocolError, decode_sample
except ImportError:  # Direct execution from the Raspberry Pi/PC copy.
    from protocol import ProtocolError, decode_sample


class EncoderPacketMonitor:
    """Track session changes, sequence gaps, and invalid packets."""

    def __init__(self, allowed_sender=""):
        self.allowed_sender = self._resolve_allowed_sender(allowed_sender)
        self.session_id = None
        self.last_sequence = None
        self.accepted_packets = 0
        self.invalid_packet_count = 0
        self.rejected_sender_count = 0
        self.sequence_gaps = 0
        self.duplicate_packets = 0
        self.session_changes = 0
        self.last_packet_received_monotonic = None
        self.last_packet = None

    @staticmethod
    def _resolve_allowed_sender(host):
        host = str(host).strip()
        if not host:
            return ""
        try:
            return socket.gethostbyname(host)
        except socket.gaierror as exc:
            raise ValueError(
                f"cannot resolve encoder allowed sender: {host}"
            ) from exc

    def accept(self, packet, sender, received_monotonic=None):
        if self.allowed_sender and sender[0] != self.allowed_sender:
            self.rejected_sender_count += 1
            return None
        try:
            sample = decode_sample(packet)
        except ProtocolError:
            self.invalid_packet_count += 1
            return None

        session_id = sample["session_id"]
        sequence = sample["sequence"]
        if self.session_id != session_id:
            if self.session_id is not None:
                self.session_changes += 1
            self.session_id = session_id
            self.last_sequence = None
        elif self.last_sequence is not None:
            if sequence <= self.last_sequence:
                self.duplicate_packets += 1
                return None
            self.sequence_gaps += max(0, sequence - self.last_sequence - 1)

        self.last_sequence = sequence
        self.accepted_packets += 1
        self.last_packet_received_monotonic = (
            time.monotonic()
            if received_monotonic is None
            else received_monotonic
        )
        self.last_packet = sample
        return sample


def _write_csv_row(writer, sample, sender, received_monotonic):
    if writer is None:
        return
    writer.writerow({
        "received_monotonic_s": f"{received_monotonic:.9f}",
        "sender": f"{sender[0]}:{sender[1]}",
        **sample,
    })


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=int(os.environ.get("ENCODER_UDP_PORT", 5011)))
    parser.add_argument("--allowed-sender", default="")
    parser.add_argument("--print-period", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--csv", default="")
    parser.add_argument("--max-packets", type=int, default=0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    monitor = EncoderPacketMonitor(allowed_sender=args.allowed_sender)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    sock.settimeout(min(0.2, max(0.01, args.timeout)))
    csv_file = None
    writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "received_monotonic_s", "sender", "version", "kind",
                "session_id", "sequence",
                "source_monotonic_ns", "cumulative_count", "delta_count",
                "direction", "sample_period_s", "pulse_age_s",
                "invalid_transition_count", "gpio_a", "gpio_b",
                "encoder_healthy", "crc32",
            ],
        )
        writer.writeheader()

    print(f"encoder receiver listening host={args.host} port={args.port}", flush=True)
    last_print = time.monotonic()
    last_received = None
    try:
        while args.max_packets <= 0 or monitor.accepted_packets < args.max_packets:
            now = time.monotonic()
            try:
                packet, sender = sock.recvfrom(4096)
            except socket.timeout:
                if monitor.last_packet_received_monotonic is not None:
                    age = now - monitor.last_packet_received_monotonic
                    if age > args.timeout and now - last_print >= args.print_period:
                        print(
                            f"encoder_timeout age_s={age:.3f} "
                            f"accepted={monitor.accepted_packets}",
                            flush=True,
                        )
                        last_print = now
                continue

            received = time.monotonic()
            sample = monitor.accept(packet, sender, received)
            if sample is not None:
                _write_csv_row(writer, sample, sender, received)
                last_received = sample
            now = time.monotonic()
            if sample is not None and now - last_print >= args.print_period:
                print(
                    f"session={sample['session_id']} seq={sample['sequence']} "
                    f"count={sample['cumulative_count']} "
                    f"delta={sample['delta_count']} direction={sample['direction']} "
                    f"pulse_age_s={sample['pulse_age_s']:.3f} "
                    f"invalid={sample['invalid_transition_count']} "
                    f"gaps={monitor.sequence_gaps}",
                    flush=True,
                )
                last_print = now
    except KeyboardInterrupt:
        print("Encoder receiver stopped cleanly", flush=True)
    finally:
        sock.close()
        if csv_file is not None:
            csv_file.close()

    last_count = None if last_received is None else last_received["cumulative_count"]
    print(
        f"summary accepted={monitor.accepted_packets} "
        f"invalid={monitor.invalid_packet_count} "
        f"sender_rejected={monitor.rejected_sender_count} "
        f"duplicates={monitor.duplicate_packets} "
        f"sequence_gaps={monitor.sequence_gaps} "
        f"session_changes={monitor.session_changes} "
        f"last_count={last_count}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
