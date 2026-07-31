"""Run one bounded hardware Duty trial while recording encoder motion."""

import argparse
import csv
import json
import math
from pathlib import Path
import select
import socket
import time
import uuid

from ..pi_agent.protocol import (
    ProtocolError as ActuatorProtocolError,
    decode_status,
    encode_command,
    make_command,
)
from ..pi_encoder_agent.receiver import EncoderPacketMonitor
from .common import DEFAULT_CONFIG, load_real_config


ENCODER_FIELDS = (
    "received_monotonic_s",
    "phase",
    "sender",
    "session_id",
    "sequence",
    "source_monotonic_ns",
    "cumulative_count",
    "delta_count",
    "direction",
    "sample_period_s",
    "pulse_age_s",
    "invalid_transition_count",
    "gpio_a",
    "gpio_b",
    "encoder_healthy",
    "speed_mps",
)

ACTUATOR_FIELDS = (
    "received_monotonic_s",
    "phase",
    "session_id",
    "sequence_id",
    "state",
    "reason",
    "esc_duty_percent",
    "steering_duty_percent",
    "hardware_output_enabled",
    "output_armed",
    "accepted_packets",
    "rejected_packets",
    "watchdog_count",
)


class TrialError(RuntimeError):
    """Raised when a Stage C safety or transport condition is not met."""


def validate_stage_c_settings(
    esc_duty,
    steering_duty,
    duration_s,
    rate_hz,
):
    """Limit each floor-test Duty to its currently approved duration."""

    approved_durations = {
        10.160: 5.0,
        10.140: 2.0,
        10.120: 1.0,
        10.100: 3.0,
    }
    selected_duty = next(
        (
            approved_duty
            for approved_duty in approved_durations
            if math.isclose(
                float(esc_duty),
                approved_duty,
                abs_tol=1e-6,
            )
        ),
        None,
    )
    if selected_duty is None:
        approved = ", ".join(
            f"{duty:.3f}%" for duty in sorted(approved_durations, reverse=True)
        )
        raise ValueError(f"ESC Duty must be one of the approved values: {approved}")
    if not math.isclose(float(steering_duty), 10.895, abs_tol=1e-6):
        raise ValueError("initial Stage C trial requires neutral steering 10.895%")
    maximum_duration_s = approved_durations[selected_duty]
    if not 0.0 < float(duration_s) <= maximum_duration_s:
        raise ValueError(
            f"ESC Duty {selected_duty:.3f}% requires duration_s in "
            f"(0, {maximum_duration_s:.1f}]"
        )
    if not 20.0 <= float(rate_hz) <= 50.0:
        raise ValueError("rate_hz must be in [20, 50]")


def calculate_trial_metrics(
    start_count,
    stop_count,
    final_count,
    distance_per_count_m,
):
    """Calculate powered, coast, and total signed encoder distances."""

    distance_per_count_m = float(distance_per_count_m)
    if distance_per_count_m <= 0.0:
        raise ValueError("distance_per_count_m must be positive")
    powered_count = int(stop_count) - int(start_count)
    coast_count = int(final_count) - int(stop_count)
    total_count = int(final_count) - int(start_count)
    return {
        "start_count": int(start_count),
        "stop_command_count": int(stop_count),
        "final_count": int(final_count),
        "powered_count_delta": powered_count,
        "coast_count_delta": coast_count,
        "total_count_delta": total_count,
        "powered_distance_m": powered_count * distance_per_count_m,
        "coast_distance_m": coast_count * distance_per_count_m,
        "total_distance_m": total_count * distance_per_count_m,
    }


def count_at_or_before(rows, timestamp_s):
    """Return the newest encoder count no later than a command timestamp."""

    candidates = [
        row for row in rows
        if float(row["received_monotonic_s"]) <= float(timestamp_s)
    ]
    if candidates:
        return int(candidates[-1]["cumulative_count"])
    if rows:
        return int(rows[0]["cumulative_count"])
    raise TrialError("no encoder samples were received")


def calculate_invalid_transition_change(rows):
    """Return cumulative transition-counter values and their change."""

    if not rows:
        raise ValueError("at least one encoder row is required")
    first_count = int(rows[0]["invalid_transition_count"])
    final_count = int(rows[-1]["invalid_transition_count"])
    return {
        "baseline": first_count,
        "final": final_count,
        "change": final_count - first_count,
    }


def calculate_cumulative_counter_change(first_status, final_status, field):
    """Calculate a per-trial change from a cumulative Agent counter."""

    baseline = int(first_status[field])
    final = int(final_status[field])
    return {
        "baseline": baseline,
        "final": final,
        "change": final - baseline,
    }


def evaluate_status_range(
    rows,
    session_id,
    first_sequence_id,
    last_sequence_id,
    expected_state,
    expected_reason,
):
    """Summarize delayed or duplicated statuses for one command range."""

    first_sequence_id = int(first_sequence_id)
    last_sequence_id = int(last_sequence_id)
    if first_sequence_id <= 0 or last_sequence_id < first_sequence_id:
        raise ValueError("invalid actuator sequence range")

    statuses_by_sequence = {}
    invalid_sequence_ids = set()
    for row in rows:
        sequence_id = int(row["sequence_id"])
        if (
            row["session_id"] != session_id
            or sequence_id < first_sequence_id
            or sequence_id > last_sequence_id
        ):
            continue
        statuses_by_sequence[sequence_id] = row
        if (
            row["state"] != expected_state
            or row["reason"] != expected_reason
        ):
            invalid_sequence_ids.add(sequence_id)

    valid_sequence_ids = {
        sequence_id
        for sequence_id, row in statuses_by_sequence.items()
        if (
            row["state"] == expected_state
            and row["reason"] == expected_reason
        )
    }
    command_count = last_sequence_id - first_sequence_id + 1
    return {
        "command_count": command_count,
        "received_sequence_count": len(statuses_by_sequence),
        "received_coverage": len(statuses_by_sequence) / command_count,
        "valid_sequence_count": len(valid_sequence_ids),
        "valid_coverage": len(valid_sequence_ids) / command_count,
        "invalid_sequence_ids": sorted(invalid_sequence_ids),
    }


def session_counter_increased(rows, session_id, field, baseline):
    """Return whether a cumulative counter rose within one command session."""

    baseline = int(baseline)
    return any(
        row["session_id"] == session_id and int(row[field]) > baseline
        for row in rows
    )


def watchdog_fired_during_motion(rows, session_id, baseline):
    """Detect a watchdog event even after the agent releases its session."""

    baseline = int(baseline)
    return any(
        int(row["watchdog_count"]) > baseline
        and (
            row["session_id"] == session_id
            or row.get("phase") == "motion"
        )
        for row in rows
    )


class TrialTransport:
    def __init__(
        self,
        pi_host,
        actuator_port,
        encoder_bind_host,
        encoder_port,
        encoder_allowed_sender,
        distance_per_count_m,
    ):
        self.distance_per_count_m = float(distance_per_count_m)
        self.encoder_monitor = EncoderPacketMonitor(
            allowed_sender=encoder_allowed_sender
        )
        self.encoder_rows = []
        self.actuator_rows = []
        self.actuator_decode_errors = 0
        self.last_sent_session_id = None
        self.last_sent_sequence_id = 0
        self.encoder_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.encoder_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.encoder_socket.bind((encoder_bind_host, int(encoder_port)))
        self.encoder_socket.setblocking(False)
        self.actuator_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.actuator_socket.connect((pi_host, int(actuator_port)))
        self.actuator_socket.setblocking(False)

    def close(self):
        self.encoder_socket.close()
        self.actuator_socket.close()

    def drain(self, timeout_s, phase):
        readable, _writable, _exceptional = select.select(
            [self.encoder_socket, self.actuator_socket],
            [],
            [],
            max(0.0, float(timeout_s)),
        )
        for source in readable:
            while True:
                try:
                    packet, sender = source.recvfrom(4096)
                except BlockingIOError:
                    break
                received = time.monotonic()
                if source is self.encoder_socket:
                    sample = self.encoder_monitor.accept(packet, sender, received)
                    if sample is None:
                        continue
                    period_s = max(float(sample["sample_period_s"]), 1e-9)
                    row = {
                        "received_monotonic_s": received,
                        "phase": phase,
                        "sender": f"{sender[0]}:{sender[1]}",
                        **sample,
                        "speed_mps": (
                            float(sample["delta_count"])
                            * self.distance_per_count_m
                            / period_s
                        ),
                    }
                    self.encoder_rows.append(row)
                    continue
                try:
                    status = decode_status(packet)
                except ActuatorProtocolError:
                    self.actuator_decode_errors += 1
                    continue
                self.actuator_rows.append({
                    "received_monotonic_s": received,
                    "phase": phase,
                    **status,
                })

    def wait(self, duration_s, phase):
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            self.drain(min(0.02, deadline - time.monotonic()), phase)

    def send_command(
        self,
        session_id,
        sequence_id,
        esc_duty_percent,
        steering_duty_percent,
        enable_output,
        reason,
    ):
        packet = encode_command(make_command(
            session_id=session_id,
            sequence_id=sequence_id,
            sent_at_unix_s=time.time(),
            esc_duty_percent=esc_duty_percent,
            steering_duty_percent=steering_duty_percent,
            enable_output=enable_output,
            reason=reason,
        ))
        self.actuator_socket.send(packet)
        self.last_sent_session_id = session_id
        self.last_sent_sequence_id = int(sequence_id)

    def send_and_wait_status(
        self,
        session_id,
        sequence_id,
        esc_duty_percent,
        steering_duty_percent,
        enable_output,
        reason,
        phase,
        timeout_s,
    ):
        start_index = len(self.actuator_rows)
        self.send_command(
            session_id,
            sequence_id,
            esc_duty_percent,
            steering_duty_percent,
            enable_output,
            reason,
        )
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            self.drain(min(0.01, deadline - time.monotonic()), phase)
            for row in self.actuator_rows[start_index:]:
                if (
                    row["session_id"] == session_id
                    and int(row["sequence_id"]) == int(sequence_id)
                ):
                    return row
        raise TrialError(
            f"actuator status timeout at sequence {sequence_id}"
        )


def _write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _make_output_directory(base_directory):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    candidate = Path(base_directory) / timestamp
    suffix = 1
    while candidate.exists():
        candidate = Path(base_directory) / f"{timestamp}_{suffix}"
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pi_host")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--actuator-port", type=int, default=5005)
    parser.add_argument("--encoder-bind-host", default="0.0.0.0")
    parser.add_argument("--encoder-port", type=int, default=5011)
    parser.add_argument("--encoder-allowed-sender", default="")
    parser.add_argument("--esc-duty", type=float, default=10.160)
    parser.add_argument("--steering-duty", type=float, default=10.895)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--ready-timeout-s", type=float, default=5.0)
    parser.add_argument("--pre-settle-s", type=float, default=1.0)
    parser.add_argument("--post-settle-s", type=float, default=2.0)
    parser.add_argument("--battery-voltage-v", type=float)
    parser.add_argument("--note", default="")
    parser.add_argument("--allow-dry-run", action="store_true")
    parser.add_argument("--confirm-track-clear", action="store_true")
    parser.add_argument("--confirm-power-cutoff-ready", action="store_true")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("experiments/real_vehicle/results/phase3_stage_c"),
    )
    args = parser.parse_args(argv)
    validate_stage_c_settings(
        args.esc_duty,
        args.steering_duty,
        args.duration_s,
        args.rate_hz,
    )
    if args.battery_voltage_v is not None and args.battery_voltage_v <= 0.0:
        parser.error("--battery-voltage-v must be positive")
    if not args.confirm_track_clear or not args.confirm_power_cutoff_ready:
        parser.error(
            "hardware trial requires --confirm-track-clear and "
            "--confirm-power-cutoff-ready"
        )
    return args


def run_trial(args):
    config, _parameters, _limits = load_real_config(args.config)
    encoder_config = config["calibration"]["encoder"]
    distance_per_count_m = float(encoder_config["distance_per_count_m"])
    output_directory = _make_output_directory(args.output_dir)
    transport = TrialTransport(
        pi_host=args.pi_host,
        actuator_port=args.actuator_port,
        encoder_bind_host=args.encoder_bind_host,
        encoder_port=args.encoder_port,
        encoder_allowed_sender=args.encoder_allowed_sender,
        distance_per_count_m=distance_per_count_m,
    )
    failure = ""
    motion_session = uuid.uuid4().hex
    sequence_id = 0
    motion_start_s = None
    stop_command_s = None
    invalid_transition_baseline = None
    motion_started = False
    explicit_stop_completed = False
    motion_command_count = 0
    motion_first_sequence_id = None
    motion_last_sequence_id = None
    explicit_stop_command_count = 0
    explicit_stop_first_sequence_id = None
    explicit_stop_last_sequence_id = None
    motion_status_evaluation = None
    explicit_stop_status_evaluation = None
    actuator_trial_baseline_status = None
    actuator_trial_final_status = None
    finally_stop_confirmed = False
    emergency_stop_confirmed = False
    watchdog_during_motion = False
    try:
        ready_deadline = time.monotonic() + args.ready_timeout_s
        while time.monotonic() < ready_deadline:
            transport.drain(0.05, "ready")
            if (
                transport.encoder_monitor.accepted_packets >= 5
                and transport.encoder_monitor.last_packet is not None
                and transport.encoder_monitor.last_packet["encoder_healthy"]
            ):
                break
        else:
            raise TrialError("encoder stream did not become healthy")
        ready_transition_rows = [
            row
            for row in transport.encoder_rows
            if row["phase"] == "ready"
        ]
        if not ready_transition_rows:
            raise TrialError("encoder transition counter was not received")
        ready_transition = calculate_invalid_transition_change(
            ready_transition_rows
        )
        if ready_transition["change"] != 0:
            raise TrialError(
                "encoder invalid transitions increased during preflight"
            )
        invalid_transition_baseline = ready_transition["final"]
        if (
            transport.encoder_monitor.invalid_packet_count > 0
            or transport.encoder_monitor.rejected_sender_count > 0
            or transport.encoder_monitor.sequence_gaps > 0
            or transport.encoder_monitor.duplicate_packets > 0
        ):
            raise TrialError("encoder stream failed preflight quality checks")

        preflight_session = uuid.uuid4().hex
        preflight_status = None
        for preflight_sequence in range(1, 4):
            preflight_status = transport.send_and_wait_status(
                preflight_session,
                preflight_sequence,
                10.30,
                10.895,
                False,
                "stage_c_preflight_stop",
                "preflight",
                0.10,
            )
            transport.wait(1.0 / args.rate_hz, "preflight")
        if preflight_status is None:
            raise TrialError("actuator preflight status was not received")
        if not args.allow_dry_run and not preflight_status[
            "hardware_output_enabled"
        ]:
            raise TrialError("Pi actuator agent is not using hardware output")
        if not preflight_status["output_armed"]:
            raise TrialError("Pi actuator output is not armed")

        transport.wait(max(0.2, args.pre_settle_s), "pre_settle")
        pre_motion_invalid_transition_count = int(
            transport.encoder_monitor.last_packet["invalid_transition_count"]
        )
        if pre_motion_invalid_transition_count != invalid_transition_baseline:
            raise TrialError(
                "encoder invalid transitions increased before motion: "
                f"{invalid_transition_baseline} -> "
                f"{pre_motion_invalid_transition_count}"
            )
        actuator_trial_baseline_status = transport.actuator_rows[-1]
        motion_start_s = time.monotonic()
        motion_started = True
        command_count = max(1, int(round(args.duration_s * args.rate_hz)))
        period_s = 1.0 / args.rate_hz
        next_command_s = motion_start_s
        for command_index in range(command_count):
            transport.wait(
                max(0.0, next_command_s - time.monotonic()),
                "motion",
            )
            sequence_id += 1
            if motion_first_sequence_id is None:
                motion_first_sequence_id = sequence_id
            transport.send_command(
                motion_session,
                sequence_id,
                args.esc_duty,
                args.steering_duty,
                True,
                "stage_c_motion",
            )
            motion_command_count += 1
            motion_last_sequence_id = sequence_id
            next_command_s = motion_start_s + (command_index + 1) * period_s

        transport.wait(
            max(0.0, motion_start_s + args.duration_s - time.monotonic()),
            "motion",
        )
        stop_command_s = time.monotonic()
        stop_start_s = stop_command_s
        for stop_index in range(5):
            transport.wait(
                max(
                    0.0,
                    stop_start_s + stop_index * period_s - time.monotonic(),
                ),
                "explicit_stop",
            )
            sequence_id += 1
            if explicit_stop_first_sequence_id is None:
                explicit_stop_first_sequence_id = sequence_id
            transport.send_command(
                motion_session,
                sequence_id,
                10.30,
                10.895,
                False,
                "stage_c_explicit_stop",
            )
            explicit_stop_command_count += 1
            explicit_stop_last_sequence_id = sequence_id
        transport.wait(
            max(
                0.0,
                stop_start_s + 5 * period_s - time.monotonic(),
            ),
            "explicit_stop",
        )
        transport.wait(args.post_settle_s, "coast")

        expected_motion_state = (
            "dry_run" if args.allow_dry_run else "running"
        )
        motion_status_evaluation = evaluate_status_range(
            transport.actuator_rows,
            motion_session,
            motion_first_sequence_id,
            motion_last_sequence_id,
            expected_motion_state,
            "stage_c_motion",
        )
        explicit_stop_status_evaluation = evaluate_status_range(
            transport.actuator_rows,
            motion_session,
            explicit_stop_first_sequence_id,
            explicit_stop_last_sequence_id,
            "stopped",
            "stage_c_explicit_stop",
        )
        actuator_trial_final_status = transport.actuator_rows[-1]
        explicit_stop_completed = (
            explicit_stop_status_evaluation["valid_sequence_count"] > 0
            and not explicit_stop_status_evaluation["invalid_sequence_ids"]
        )
        watchdog_during_motion = watchdog_fired_during_motion(
            transport.actuator_rows,
            motion_session,
            actuator_trial_baseline_status["watchdog_count"],
        )
        if watchdog_during_motion:
            raise TrialError("actuator watchdog fired during powered motion")
        if not explicit_stop_completed:
            raise TrialError(
                "explicit stop status was not confirmed after the trial"
            )
        if motion_status_evaluation["valid_sequence_count"] == 0:
            raise TrialError("no valid motion status was received")
        if motion_status_evaluation["invalid_sequence_ids"]:
            raise TrialError(
                "invalid motion status at sequences: "
                + ",".join(
                    str(value)
                    for value in motion_status_evaluation[
                        "invalid_sequence_ids"
                    ]
                )
            )

        accepted_change = calculate_cumulative_counter_change(
            actuator_trial_baseline_status,
            actuator_trial_final_status,
            "accepted_packets",
        )
        rejected_change = calculate_cumulative_counter_change(
            actuator_trial_baseline_status,
            actuator_trial_final_status,
            "rejected_packets",
        )
        expected_accepted_commands = (
            motion_command_count + explicit_stop_command_count
        )
        if accepted_change["change"] < expected_accepted_commands:
            raise TrialError(
                "not all motion and stop commands were accepted: "
                f"expected={expected_accepted_commands} "
                f"accepted={accepted_change['change']}"
            )
        if rejected_change["change"] != 0:
            raise TrialError(
                "actuator commands were rejected during the trial: "
                f"{rejected_change['change']}"
            )
        final_invalid_transition_count = int(
            transport.encoder_monitor.last_packet["invalid_transition_count"]
        )
        if final_invalid_transition_count != invalid_transition_baseline:
            raise TrialError(
                "encoder invalid transitions increased during trial: "
                f"{invalid_transition_baseline} -> "
                f"{final_invalid_transition_count}"
            )
    except (KeyboardInterrupt, OSError, TrialError, ValueError) as exc:
        failure = str(exc)
    finally:
        if (
            motion_started
            and not explicit_stop_completed
            and transport.last_sent_session_id is not None
        ):
            emergency_session = transport.last_sent_session_id
            emergency_sequence = transport.last_sent_sequence_id
            for _stop_index in range(5):
                emergency_sequence += 1
                try:
                    transport.send_command(
                        emergency_session,
                        emergency_sequence,
                        10.30,
                        10.895,
                        False,
                        "stage_c_emergency_stop",
                    )
                except OSError:
                    pass
                transport.wait(0.02, "emergency_stop")
            transport.wait(0.25, "emergency_stop")
            emergency_stop_confirmed = any(
                row["session_id"] == emergency_session
                and row["reason"] == "stage_c_emergency_stop"
                and row["state"] == "stopped"
                for row in transport.actuator_rows
            )
        emergency_session = uuid.uuid4().hex
        emergency_sequence = 0
        for _stop_index in range(5):
            emergency_sequence += 1
            try:
                transport.send_command(
                    emergency_session,
                    emergency_sequence,
                    10.30,
                    10.895,
                    False,
                    "stage_c_finally_stop",
                )
            except OSError:
                pass
            transport.wait(0.02, "finally_stop")
        transport.wait(0.25, "finally_stop")
        finally_stop_confirmed = any(
            row["session_id"] == emergency_session
            and row["reason"] == "stage_c_finally_stop"
            and row["state"] == "stopped"
            for row in transport.actuator_rows
        )
        transport.close()

    summary = {
        "status": "failed" if failure else "completed",
        "failure": failure,
        "esc_duty_percent": args.esc_duty,
        "steering_duty_percent": args.steering_duty,
        "requested_duration_s": args.duration_s,
        "command_rate_hz": args.rate_hz,
        "battery_voltage_v": args.battery_voltage_v,
        "note": args.note,
        "distance_per_count_m": distance_per_count_m,
        "encoder_packet_count": transport.encoder_monitor.accepted_packets,
        "encoder_invalid_packets": transport.encoder_monitor.invalid_packet_count,
        "encoder_rejected_senders": transport.encoder_monitor.rejected_sender_count,
        "encoder_duplicate_packets": transport.encoder_monitor.duplicate_packets,
        "encoder_sequence_gaps": transport.encoder_monitor.sequence_gaps,
        "actuator_status_count": len(transport.actuator_rows),
        "actuator_decode_errors": transport.actuator_decode_errors,
        "motion_commands_sent": motion_command_count,
        "explicit_stop_commands_sent": explicit_stop_command_count,
        "explicit_stop_confirmed": explicit_stop_completed,
        "emergency_stop_confirmed": emergency_stop_confirmed,
        "finally_stop_confirmed": finally_stop_confirmed,
        "watchdog_observed": any(
            row["reason"] == "command_timeout"
            for row in transport.actuator_rows
        ),
        "watchdog_during_motion": watchdog_during_motion,
    }
    if motion_status_evaluation is not None:
        summary.update({
            "motion_status_sequences": motion_status_evaluation[
                "received_sequence_count"
            ],
            "motion_status_coverage": motion_status_evaluation[
                "received_coverage"
            ],
            "motion_valid_status_sequences": motion_status_evaluation[
                "valid_sequence_count"
            ],
            "motion_valid_status_coverage": motion_status_evaluation[
                "valid_coverage"
            ],
        })
    if explicit_stop_status_evaluation is not None:
        summary.update({
            "explicit_stop_status_sequences": (
                explicit_stop_status_evaluation["received_sequence_count"]
            ),
            "explicit_stop_status_coverage": (
                explicit_stop_status_evaluation["received_coverage"]
            ),
        })
    if transport.encoder_rows:
        transition_change = calculate_invalid_transition_change(
            transport.encoder_rows
        )
        summary_baseline = (
            invalid_transition_baseline
            if invalid_transition_baseline is not None
            else transition_change["baseline"]
        )
        summary.update({
            "encoder_invalid_transition_baseline": summary_baseline,
            "encoder_invalid_transition_final": transition_change["final"],
            "encoder_invalid_transitions_during_trial": (
                transition_change["final"] - summary_baseline
            ),
        })
    if transport.encoder_rows and motion_start_s is not None and stop_command_s is not None:
        summary["actual_powered_duration_s"] = (
            stop_command_s - motion_start_s
        )
        start_count = count_at_or_before(
            transport.encoder_rows, motion_start_s
        )
        stop_count = count_at_or_before(
            transport.encoder_rows, stop_command_s
        )
        final_count = int(transport.encoder_rows[-1]["cumulative_count"])
        summary.update(calculate_trial_metrics(
            start_count,
            stop_count,
            final_count,
            distance_per_count_m,
        ))
        trial_speeds = [
            abs(float(row["speed_mps"]))
            for row in transport.encoder_rows
            if float(row["received_monotonic_s"]) >= motion_start_s
        ]
        summary["max_abs_encoder_speed_mps"] = max(
            trial_speeds, default=0.0
        )
        summary["motion_detected"] = abs(summary["total_count_delta"]) > 0
        summary["forward_direction_ok"] = summary["total_count_delta"] > 0
    if transport.actuator_rows:
        last_status = transport.actuator_rows[-1]
        confirmed_stop_status = next(
            (
                row
                for row in reversed(transport.actuator_rows)
                if row["state"] == "stopped"
                and row["reason"] in {
                    "stage_c_explicit_stop",
                    "stage_c_emergency_stop",
                    "stage_c_finally_stop",
                }
            ),
            None,
        )
        final_status = confirmed_stop_status or last_status
        counter_baseline_status = (
            actuator_trial_baseline_status or transport.actuator_rows[0]
        )
        counter_final_status = actuator_trial_final_status or last_status
        accepted_change = calculate_cumulative_counter_change(
            counter_baseline_status,
            counter_final_status,
            "accepted_packets",
        )
        rejected_change = calculate_cumulative_counter_change(
            counter_baseline_status,
            counter_final_status,
            "rejected_packets",
        )
        watchdog_change = calculate_cumulative_counter_change(
            counter_baseline_status,
            counter_final_status,
            "watchdog_count",
        )
        summary.update({
            "final_actuator_state": final_status["state"],
            "final_actuator_reason": final_status["reason"],
            "last_actuator_state": last_status["state"],
            "last_actuator_reason": last_status["reason"],
            "hardware_output_enabled": final_status[
                "hardware_output_enabled"
            ],
            "output_armed": final_status["output_armed"],
            "actuator_accepted_packets": final_status["accepted_packets"],
            "actuator_rejected_packets": final_status["rejected_packets"],
            "actuator_watchdog_count": final_status["watchdog_count"],
            "actuator_accepted_packets_baseline": accepted_change["baseline"],
            "actuator_accepted_packets_during_trial": accepted_change["change"],
            "actuator_rejected_packets_baseline": rejected_change["baseline"],
            "actuator_rejected_packets_during_trial": rejected_change["change"],
            "actuator_watchdog_count_baseline": watchdog_change["baseline"],
            "actuator_watchdog_events_during_trial": watchdog_change["change"],
        })

    _write_csv(
        output_directory / "encoder.csv",
        ENCODER_FIELDS,
        transport.encoder_rows,
    )
    _write_csv(
        output_directory / "actuator_status.csv",
        ACTUATOR_FIELDS,
        transport.actuator_rows,
    )
    (output_directory / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"output_dir: {output_directory.resolve()}")
    if failure:
        raise TrialError(failure)
    return summary


def main(argv=None):
    args = parse_args(argv)
    try:
        run_trial(args)
    except TrialError as exc:
        raise SystemExit(f"Stage C trial failed safely: {exc}") from exc
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
