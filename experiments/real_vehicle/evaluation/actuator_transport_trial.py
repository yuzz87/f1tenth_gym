#!/usr/bin/env python3
"""Measure fixed-rate PC-to-Pi actuator UDP transport in dry-run mode."""

import argparse
import csv
import json
import math
from pathlib import Path
import select
import socket
import statistics
import time
import uuid

from ..pi_agent.protocol import (
    ProtocolError,
    decode_status,
    encode_command,
    make_command,
)


COMMAND_FIELDS = (
    "sequence_id",
    "scheduled_monotonic_s",
    "sent_monotonic_s",
    "schedule_lateness_s",
    "send_interval_s",
    "status_received_monotonic_s",
    "status_rtt_s",
    "status_state",
    "status_reason",
)

STATUS_FIELDS = (
    "received_monotonic_s",
    "phase",
    "session_id",
    "sequence_id",
    "state",
    "reason",
    "hardware_output_enabled",
    "output_armed",
    "accepted_packets",
    "rejected_packets",
    "watchdog_count",
)


def percentile(values, percentile_value):
    """Return a linearly interpolated percentile for a non-empty sequence."""

    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(percentile_value) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def evaluate_transport_run(
    command_rows,
    status_rows,
    session_id,
    baseline_status,
    watchdog_timeout_s,
    minimum_status_coverage,
    decode_errors=0,
):
    """Evaluate one fixed-rate neutral-command stream."""

    command_by_sequence = {
        int(row["sequence_id"]): row
        for row in command_rows
    }
    matching_status = {}
    invalid_status_sequences = set()
    hardware_status_seen = False
    for row in status_rows:
        hardware_status_seen = hardware_status_seen or bool(
            row["hardware_output_enabled"]
        )
        sequence_id = int(row["sequence_id"])
        if (
            row["session_id"] != session_id
            or sequence_id not in command_by_sequence
        ):
            continue
        matching_status.setdefault(sequence_id, row)
        if row["state"] != "stopped" or row["reason"] != "transport_keepalive":
            invalid_status_sequences.add(sequence_id)

    command_count = len(command_rows)
    coverage = len(matching_status) / command_count if command_count else 0.0
    send_intervals = [
        float(row["send_interval_s"])
        for row in command_rows
        if row["send_interval_s"] not in (None, "")
    ]
    schedule_lateness = [
        float(row["schedule_lateness_s"])
        for row in command_rows
    ]
    status_rtts = []
    matching_receive_times = []
    for sequence_id, status in matching_status.items():
        sent_at = float(command_by_sequence[sequence_id]["sent_monotonic_s"])
        received_at = float(status["received_monotonic_s"])
        status_rtts.append(max(0.0, received_at - sent_at))
        matching_receive_times.append(received_at)
    matching_receive_times.sort()
    status_gaps = [
        later - earlier
        for earlier, later in zip(
            matching_receive_times,
            matching_receive_times[1:],
        )
    ]

    baseline_accepted = int(baseline_status["accepted_packets"])
    baseline_rejected = int(baseline_status["rejected_packets"])
    baseline_watchdog = int(baseline_status["watchdog_count"])
    maximum_accepted = max(
        [baseline_accepted]
        + [int(row["accepted_packets"]) for row in status_rows]
    )
    maximum_rejected = max(
        [baseline_rejected]
        + [int(row["rejected_packets"]) for row in status_rows]
    )
    maximum_watchdog = max(
        [baseline_watchdog]
        + [int(row["watchdog_count"]) for row in status_rows]
    )
    accepted_change = maximum_accepted - baseline_accepted
    rejected_change = maximum_rejected - baseline_rejected
    watchdog_during_stream = any(
        row["phase"] == "stream"
        and int(row["watchdog_count"]) > baseline_watchdog
        for row in status_rows
    )
    post_stream_watchdog = any(
        row["phase"] == "post_stream"
        and row["reason"] == "command_timeout"
        and int(row["watchdog_count"]) > baseline_watchdog
        for row in status_rows
    )
    max_send_interval = max(send_intervals) if send_intervals else 0.0
    send_interval_overruns = sum(
        interval >= float(watchdog_timeout_s)
        for interval in send_intervals
    )

    failures = []
    if command_count == 0:
        failures.append("no commands were sent")
    if accepted_change != command_count:
        failures.append(
            "accepted packet change does not match sent commands: "
            f"accepted={accepted_change} sent={command_count}"
        )
    if rejected_change != 0:
        failures.append(f"agent rejected {rejected_change} commands")
    if watchdog_during_stream:
        failures.append("watchdog fired during the command stream")
    if not post_stream_watchdog:
        failures.append("post-stream watchdog neutral was not observed")
    if coverage < float(minimum_status_coverage):
        failures.append(
            f"status coverage {coverage:.6f} is below "
            f"{float(minimum_status_coverage):.6f}"
        )
    if invalid_status_sequences:
        failures.append("matching statuses were not neutral keepalive statuses")
    if hardware_status_seen:
        failures.append("Pi agent was not running in dry-run mode")
    if send_interval_overruns:
        failures.append(
            f"{send_interval_overruns} send intervals reached the watchdog timeout"
        )
    if int(decode_errors) != 0:
        failures.append(f"received {int(decode_errors)} invalid status packets")

    return {
        "status": "passed" if not failures else "failed",
        "failures": failures,
        "sent_commands": command_count,
        "matching_status_sequences": len(matching_status),
        "status_coverage": coverage,
        "minimum_status_coverage": float(minimum_status_coverage),
        "accepted_packets_baseline": baseline_accepted,
        "accepted_packets_during_stream": accepted_change,
        "rejected_packets_baseline": baseline_rejected,
        "rejected_packets_during_stream": rejected_change,
        "watchdog_count_baseline": baseline_watchdog,
        "watchdog_count_maximum": maximum_watchdog,
        "watchdog_during_stream": watchdog_during_stream,
        "post_stream_watchdog_observed": post_stream_watchdog,
        "invalid_status_sequences": sorted(invalid_status_sequences),
        "hardware_output_status_seen": hardware_status_seen,
        "status_decode_errors": int(decode_errors),
        "max_send_interval_s": max_send_interval,
        "p95_send_interval_s": percentile(send_intervals, 95.0),
        "mean_send_interval_s": (
            statistics.mean(send_intervals) if send_intervals else 0.0
        ),
        "send_interval_overruns": send_interval_overruns,
        "max_schedule_lateness_s": max(schedule_lateness, default=0.0),
        "p95_schedule_lateness_s": percentile(schedule_lateness, 95.0),
        "max_status_rtt_s": max(status_rtts, default=None),
        "p95_status_rtt_s": percentile(status_rtts, 95.0),
        "max_matching_status_gap_s": max(status_gaps, default=None),
    }


class StatusCollector:
    def __init__(self, pi_host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((pi_host, int(port)))
        self.socket.setblocking(False)
        self.rows = []
        self.decode_errors = 0

    def close(self):
        self.socket.close()

    def send(self, session_id, sequence_id, reason):
        packet = encode_command(make_command(
            session_id=session_id,
            sequence_id=int(sequence_id),
            sent_at_unix_s=time.time(),
            esc_duty_percent=10.30,
            steering_duty_percent=10.895,
            enable_output=False,
            reason=reason,
        ))
        self.socket.send(packet)

    def drain(self, timeout_s, phase):
        readable, _writable, _exceptional = select.select(
            [self.socket],
            [],
            [],
            max(0.0, float(timeout_s)),
        )
        if not readable:
            return
        while True:
            try:
                packet = self.socket.recv(4096)
            except BlockingIOError:
                break
            received_at = time.monotonic()
            try:
                status = decode_status(packet)
            except ProtocolError:
                self.decode_errors += 1
                continue
            self.rows.append({
                "received_monotonic_s": received_at,
                "phase": phase,
                **status,
            })

    def wait(self, duration_s, phase):
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            self.drain(min(0.02, deadline - time.monotonic()), phase)

    def wait_for(self, predicate, timeout_s, phase):
        start_index = len(self.rows)
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            self.drain(min(0.02, deadline - time.monotonic()), phase)
            for row in self.rows[start_index:]:
                if predicate(row):
                    return row
        return None


def make_output_directory(base_directory):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    candidate = Path(base_directory) / timestamp
    suffix = 1
    while candidate.exists():
        candidate = Path(base_directory) / f"{timestamp}_{suffix}"
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(rows)


def run_trial(args):
    output_directory = make_output_directory(args.output_dir)
    collector = StatusCollector(args.pi_host, args.port)
    command_rows = []
    try:
        preflight_session = uuid.uuid4().hex
        collector.send(preflight_session, 1, "transport_preflight")
        preflight_status = collector.wait_for(
            lambda row: (
                row["session_id"] == preflight_session
                and int(row["sequence_id"]) == 1
            ),
            args.ready_timeout_s,
            "preflight",
        )
        if preflight_status is None:
            raise RuntimeError("Pi agent did not answer the neutral preflight")
        if preflight_status["hardware_output_enabled"]:
            raise RuntimeError("Pi agent must be started without --enable-hardware")

        preflight_watchdog = collector.wait_for(
            lambda row: (
                row["reason"] == "command_timeout"
                and int(row["watchdog_count"])
                > int(preflight_status["watchdog_count"])
            ),
            args.watchdog_timeout_s + args.ready_timeout_s,
            "preflight_release",
        )
        if preflight_watchdog is None:
            raise RuntimeError("preflight watchdog release was not observed")

        baseline_status = preflight_watchdog
        session_id = uuid.uuid4().hex
        command_count = max(2, int(round(args.duration_s * args.rate_hz)))
        period_s = 1.0 / args.rate_hz
        start_at = time.monotonic() + 0.05
        previous_sent_at = None
        for sequence_id in range(1, command_count + 1):
            scheduled_at = start_at + (sequence_id - 1) * period_s
            while time.monotonic() < scheduled_at:
                collector.drain(
                    min(0.01, scheduled_at - time.monotonic()),
                    "stream",
                )
            sent_at = time.monotonic()
            collector.send(session_id, sequence_id, "transport_keepalive")
            command_rows.append({
                "sequence_id": sequence_id,
                "scheduled_monotonic_s": scheduled_at,
                "sent_monotonic_s": sent_at,
                "schedule_lateness_s": max(0.0, sent_at - scheduled_at),
                "send_interval_s": (
                    "" if previous_sent_at is None else sent_at - previous_sent_at
                ),
            })
            previous_sent_at = sent_at

        collector.wait(
            args.watchdog_timeout_s + args.post_watchdog_wait_s,
            "post_stream",
        )
        summary = evaluate_transport_run(
            command_rows,
            collector.rows,
            session_id,
            baseline_status,
            args.watchdog_timeout_s,
            args.minimum_status_coverage,
            collector.decode_errors,
        )
        first_status_by_sequence = {}
        for row in collector.rows:
            sequence_id = int(row["sequence_id"])
            if row["session_id"] == session_id and sequence_id in range(
                1,
                command_count + 1,
            ):
                first_status_by_sequence.setdefault(sequence_id, row)
        for row in command_rows:
            status = first_status_by_sequence.get(int(row["sequence_id"]))
            if status is None:
                row.update({
                    "status_received_monotonic_s": "",
                    "status_rtt_s": "",
                    "status_state": "",
                    "status_reason": "",
                })
                continue
            received_at = float(status["received_monotonic_s"])
            row.update({
                "status_received_monotonic_s": received_at,
                "status_rtt_s": max(
                    0.0,
                    received_at - float(row["sent_monotonic_s"]),
                ),
                "status_state": status["state"],
                "status_reason": status["reason"],
            })
        summary.update({
            "pi_host": args.pi_host,
            "port": args.port,
            "duration_s": args.duration_s,
            "command_rate_hz": args.rate_hz,
            "watchdog_timeout_s": args.watchdog_timeout_s,
            "note": args.note,
            "session_id": session_id,
            "preflight_watchdog_observed": True,
            "dry_run_confirmed": True,
        })
    except (OSError, RuntimeError, ValueError) as exc:
        summary = {
            "status": "failed",
            "failures": [str(exc)],
            "pi_host": args.pi_host,
            "port": args.port,
            "duration_s": args.duration_s,
            "command_rate_hz": args.rate_hz,
            "watchdog_timeout_s": args.watchdog_timeout_s,
            "note": args.note,
            "status_decode_errors": collector.decode_errors,
        }
    finally:
        collector.close()

    write_csv(output_directory / "commands.csv", COMMAND_FIELDS, command_rows)
    write_csv(output_directory / "status.csv", STATUS_FIELDS, collector.rows)
    with (output_directory / "summary.json").open("w", encoding="utf-8") as file:
        json.dump(summary, file, indent=2, sort_keys=True)
        file.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"output_dir: {output_directory.resolve()}")
    return 0 if summary["status"] == "passed" else 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pi_host")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--watchdog-timeout-s", type=float, default=0.20)
    parser.add_argument("--ready-timeout-s", type=float, default=1.0)
    parser.add_argument("--post-watchdog-wait-s", type=float, default=0.20)
    parser.add_argument("--minimum-status-coverage", type=float, default=0.99)
    parser.add_argument("--note", default="")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(
            "experiments/real_vehicle/results/actuator_transport"
        ),
    )
    args = parser.parse_args(argv)
    if args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    if not 1.0 <= args.rate_hz <= 100.0:
        parser.error("--rate-hz must be in [1, 100]")
    if not 0.05 <= args.watchdog_timeout_s <= 1.0:
        parser.error("--watchdog-timeout-s must be in [0.05, 1.0]")
    if args.ready_timeout_s <= 0.0:
        parser.error("--ready-timeout-s must be positive")
    if args.post_watchdog_wait_s <= 0.0:
        parser.error("--post-watchdog-wait-s must be positive")
    if not 0.0 < args.minimum_status_coverage <= 1.0:
        parser.error("--minimum-status-coverage must be in (0, 1]")
    return args


def main(argv=None):
    return run_trial(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
