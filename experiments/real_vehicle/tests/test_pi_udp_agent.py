import contextlib
import io
import json
from types import SimpleNamespace
import unittest

from experiments.real_vehicle.pi_agent.agent import (
    DryRunBackend,
    UdpActuatorAgent,
    duty_to_pigpio,
    make_parser,
    validate_args,
)
from experiments.real_vehicle.pi_agent.policy import SafeCommandPolicy
from experiments.real_vehicle.pi_agent.protocol import (
    ProtocolError,
    decode_command,
    decode_status,
    encode_command,
    encode_status,
    make_command,
    make_status,
)
from experiments.real_vehicle.pi_agent.send_test_command import (
    receive_matching_status,
)


def command_packet(
    sequence_id=1,
    sent_at_unix_s=100.0,
    enable_output=True,
    esc_duty_percent=10.16,
    steering_duty_percent=10.895,
    session_id="test-session",
):
    return encode_command(make_command(
        session_id=session_id,
        sequence_id=sequence_id,
        sent_at_unix_s=sent_at_unix_s,
        esc_duty_percent=esc_duty_percent,
        steering_duty_percent=steering_duty_percent,
        enable_output=enable_output,
        reason="test_command",
    ))


def agent_args(**overrides):
    values = {
        "esc_neutral_duty": 10.30,
        "esc_start_duty": 10.16,
        "esc_forward_min_duty": 10.10,
        "steering_neutral_duty": 10.895,
        "steering_min_duty": 9.05,
        "steering_max_duty": 12.42,
        "message_max_age": 0.10,
        "future_tolerance": 0.10,
        "watchdog_timeout": 0.10,
        "status_rate_hz": 10.0,
        "arm_output": True,
        "allow_motion": True,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def status_packet(sequence_id, session_id="test-session"):
    return encode_status(make_status(
        session_id=session_id,
        sequence_id=sequence_id,
        sent_at_unix_s=100.0,
        state="stopped",
        reason="test",
        esc_duty_percent=10.30,
        steering_duty_percent=10.895,
        hardware_output_enabled=False,
        output_armed=True,
        accepted_packets=sequence_id,
        rejected_packets=0,
        watchdog_count=0,
    ))


class FakeStatusSocket:
    def __init__(self, packets):
        self.packets = list(packets)

    def settimeout(self, _timeout_s):
        return None

    def recvfrom(self, _buffer_size):
        if not self.packets:
            raise TimeoutError
        return self.packets.pop(0), ("127.0.0.1", 5005)


class ProtocolTest(unittest.TestCase):
    def test_command_round_trip(self):
        packet = command_packet()
        command = decode_command(packet)

        self.assertEqual(command["session_id"], "test-session")
        self.assertEqual(command["sequence_id"], 1)
        self.assertAlmostEqual(command["esc_duty_percent"], 10.16)
        self.assertTrue(command["enable_output"])

    def test_non_finite_and_wrong_types_are_rejected(self):
        with self.assertRaises(ProtocolError):
            make_command("s", 1, 100.0, float("nan"), 10.895, True, "bad")
        with self.assertRaises(ProtocolError):
            make_command("s", True, 100.0, 10.16, 10.895, True, "bad")

    def test_invalid_status_types_are_rejected(self):
        packet = json.dumps({
            "version": 1,
            "kind": "actuator_status",
            "session_id": "s",
            "sequence_id": "not-an-integer",
            "sent_at_unix_s": 100.0,
            "state": "running",
            "reason": "test",
            "esc_duty_percent": 10.16,
            "steering_duty_percent": 10.895,
            "hardware_output_enabled": False,
            "output_armed": False,
            "accepted_packets": 1,
            "rejected_packets": 0,
            "watchdog_count": 0,
        }).encode("ascii")

        with self.assertRaises(ProtocolError):
            decode_status(packet)

    def test_test_sender_ignores_periodic_status_for_previous_sequence(self):
        sock = FakeStatusSocket([
            status_packet(1),
            status_packet(2),
        ])

        status = receive_matching_status(sock, "test-session", 2)

        self.assertEqual(status["sequence_id"], 2)


class SafeCommandPolicyTest(unittest.TestCase):
    def setUp(self):
        self.policy = SafeCommandPolicy()

    def test_valid_motion_command_is_accepted(self):
        decision = self.policy.process(command_packet(), "192.0.2.10", 100.05)

        self.assertTrue(decision.accepted)
        self.assertTrue(decision.enable_output)
        self.assertAlmostEqual(decision.esc_duty_percent, 10.16)

    def test_disabled_command_is_forced_to_neutral(self):
        decision = self.policy.process(
            command_packet(
                enable_output=False,
                esc_duty_percent=9.0,
                steering_duty_percent=20.0,
            ),
            "192.0.2.10",
            100.05,
        )

        self.assertTrue(decision.accepted)
        self.assertFalse(decision.enable_output)
        self.assertAlmostEqual(decision.esc_duty_percent, 10.30)
        self.assertAlmostEqual(decision.steering_duty_percent, 10.895)

    def test_motion_outside_duty_limits_is_rejected(self):
        decision = self.policy.process(
            command_packet(esc_duty_percent=10.09),
            "192.0.2.10",
            100.05,
        )

        self.assertFalse(decision.accepted)
        self.assertFalse(decision.enable_output)
        self.assertEqual(decision.reason.split(":", 1)[0], "invalid_command")

    def test_steering_is_allowed_while_esc_stays_neutral(self):
        decision = self.policy.process(
            command_packet(
                esc_duty_percent=10.30,
                steering_duty_percent=10.80,
            ),
            "192.0.2.10",
            100.05,
        )

        self.assertTrue(decision.accepted)
        self.assertTrue(decision.enable_output)
        self.assertAlmostEqual(decision.esc_duty_percent, 10.30)
        self.assertAlmostEqual(decision.steering_duty_percent, 10.80)

    def test_stale_future_replay_and_sender_conflict_are_rejected(self):
        stale = self.policy.process(command_packet(), "192.0.2.10", 100.30)
        self.assertFalse(stale.accepted)

        future = self.policy.process(command_packet(), "192.0.2.10", 99.80)
        self.assertFalse(future.accepted)

        accepted = self.policy.process(command_packet(), "192.0.2.10", 100.05)
        self.assertTrue(accepted.accepted)

        replay = self.policy.process(command_packet(), "192.0.2.10", 100.05)
        self.assertFalse(replay.accepted)

        conflict = self.policy.process(
            command_packet(sequence_id=2),
            "192.0.2.11",
            100.05,
        )
        self.assertFalse(conflict.accepted)


class UdpActuatorAgentTest(unittest.TestCase):
    def test_default_watchdog_uses_wifi_validated_timeout(self):
        args = make_parser().parse_args([])

        self.assertAlmostEqual(args.watchdog_timeout, 0.20)

    def test_floor_hardware_mode_accepts_track_clear_confirmation(self):
        parser = make_parser()
        args = parser.parse_args([
            "--enable-hardware",
            "--arm-output",
            "--allow-motion",
            "--confirm-track-clear",
            "--confirm-power-cutoff",
        ])

        validate_args(args, parser)

    def test_hardware_mode_rejects_missing_safe_test_area(self):
        parser = make_parser()
        args = parser.parse_args([
            "--enable-hardware",
            "--confirm-power-cutoff",
        ])

        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                validate_args(args, parser)

    def test_motion_requires_software_arm_and_motion_permission(self):
        for overrides, expected_reason in (
            ({"arm_output": False}, "output_disarmed"),
            ({"allow_motion": False}, "motion_disabled"),
        ):
            backend = DryRunBackend()
            agent = UdpActuatorAgent(
                agent_args(**overrides),
                backend,
                wall_clock=lambda: 100.05,
                monotonic=lambda: 10.0,
            )

            decision = agent.process_packet(command_packet(), ("192.0.2.10", 5007))

            self.assertFalse(decision.enable_output)
            self.assertEqual(decision.reason, expected_reason)
            self.assertAlmostEqual(backend.last_output[0], 10.30)

    def test_watchdog_returns_output_to_neutral_and_releases_session(self):
        monotonic_time = [10.0]
        backend = DryRunBackend()
        agent = UdpActuatorAgent(
            agent_args(),
            backend,
            wall_clock=lambda: 100.05,
            monotonic=lambda: monotonic_time[0],
        )
        decision = agent.process_packet(command_packet(), ("192.0.2.10", 5007))
        self.assertTrue(decision.enable_output)

        monotonic_time[0] = 10.101
        self.assertTrue(agent.check_watchdog())
        self.assertEqual(agent.last_decision.reason, "command_timeout")
        self.assertAlmostEqual(backend.last_output[0], 10.30)
        self.assertIsNone(agent.policy.active_session_id)
        self.assertEqual(agent.watchdog_count, 1)

        delayed_packet = command_packet(sequence_id=2)
        delayed = agent.process_packet(delayed_packet, ("192.0.2.10", 5007))
        self.assertFalse(delayed.accepted)
        self.assertFalse(delayed.enable_output)
        self.assertIn("session is already closed", delayed.reason)
        self.assertAlmostEqual(backend.last_output[0], 10.30)
        self.assertFalse(agent.check_watchdog())

    def test_steering_only_does_not_require_motion_permission(self):
        backend = DryRunBackend()
        agent = UdpActuatorAgent(
            agent_args(allow_motion=False),
            backend,
            wall_clock=lambda: 100.05,
            monotonic=lambda: 10.0,
        )
        packet = command_packet(
            esc_duty_percent=10.30,
            steering_duty_percent=10.80,
        )

        decision = agent.process_packet(packet, ("192.0.2.10", 5007))

        self.assertTrue(decision.enable_output)
        self.assertAlmostEqual(backend.last_output[0], 10.30)
        self.assertAlmostEqual(backend.last_output[1], 10.80)

    def test_dry_run_status_never_claims_hardware_output(self):
        backend = DryRunBackend()
        agent = UdpActuatorAgent(
            agent_args(),
            backend,
            wall_clock=lambda: 100.05,
            monotonic=lambda: 10.0,
        )
        agent.process_packet(command_packet(), ("192.0.2.10", 5007))

        status = decode_status(agent.status_packet())

        self.assertEqual(status["state"], "dry_run")
        self.assertFalse(status["hardware_output_enabled"])

    def test_duty_conversion_matches_pigpio_scale(self):
        self.assertEqual(duty_to_pigpio(10.30), 103000)
        self.assertEqual(duty_to_pigpio(10.10), 101000)


if __name__ == "__main__":
    unittest.main()
