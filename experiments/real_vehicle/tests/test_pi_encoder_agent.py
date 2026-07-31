import json
import unittest

from experiments.real_vehicle.pi_encoder_agent.protocol import (
    ProtocolError,
    decode_sample,
    encode_sample,
    make_sample,
)
from experiments.real_vehicle.pi_encoder_agent.quadrature import QuadratureDecoder
from experiments.real_vehicle.pi_encoder_agent.receiver import EncoderPacketMonitor


def sample(**overrides):
    values = {
        "session_id": "test-session",
        "sequence": 1,
        "source_monotonic_ns": 100,
        "cumulative_count": 4,
        "delta_count": 4,
        "direction": 1,
        "sample_period_s": 0.02,
        "pulse_age_s": 0.001,
        "invalid_transition_count": 0,
        "gpio_a": 0,
        "gpio_b": 0,
        "encoder_healthy": True,
    }
    values.update(overrides)
    return make_sample(**values)


class QuadratureDecoderTest(unittest.TestCase):
    def test_positive_sequence_increments(self):
        decoder = QuadratureDecoder()
        for index, state in enumerate(
            ((0, 0), (0, 1), (1, 1), (1, 0), (0, 0)),
            start=1,
        ):
            decoder.update(*state, index)

        result = decoder.snapshot(10)
        self.assertEqual(result["cumulative_count"], 4)
        self.assertEqual(result["invalid_transition_count"], 0)

    def test_reverse_sequence_decrements(self):
        decoder = QuadratureDecoder()
        for index, state in enumerate(
            ((0, 0), (1, 0), (1, 1), (0, 1), (0, 0)),
            start=1,
        ):
            decoder.update(*state, index)

        self.assertEqual(decoder.snapshot(10)["cumulative_count"], -4)

    def test_invalid_transition_is_not_counted(self):
        decoder = QuadratureDecoder()
        decoder.update(0, 0, 1)
        decoder.update(1, 1, 2)

        result = decoder.snapshot(10)
        self.assertEqual(result["cumulative_count"], 0)
        self.assertEqual(result["invalid_transition_count"], 1)

    def test_direction_sign_can_be_inverted(self):
        decoder = QuadratureDecoder(direction_sign=-1)
        for index, state in enumerate(
            ((0, 0), (0, 1), (1, 1), (1, 0), (0, 0)),
            start=1,
        ):
            decoder.update(*state, index)

        self.assertEqual(decoder.snapshot(10)["cumulative_count"], -4)

    def test_pulse_age_increases_when_no_transition_arrives(self):
        decoder = QuadratureDecoder()
        decoder.update(0, 0, 1_000_000_000)
        decoder.update(0, 1, 2_000_000_000)

        self.assertAlmostEqual(decoder.snapshot(2_500_000_000)["pulse_age_s"], 0.5)


class EncoderProtocolTest(unittest.TestCase):
    def test_crc_round_trip(self):
        packet = encode_sample(sample())
        decoded = decode_sample(packet)
        self.assertEqual(decoded["cumulative_count"], 4)
        self.assertEqual(decoded["crc32"], decoded["crc32"].lower())

    def test_crc_tampering_is_rejected(self):
        message = json.loads(encode_sample(sample()).decode("ascii"))
        message["cumulative_count"] = 5
        tampered = json.dumps(message, separators=(",", ":")).encode("ascii")

        with self.assertRaises(ProtocolError):
            decode_sample(tampered)

    def test_bad_direction_is_rejected(self):
        with self.assertRaises(ProtocolError):
            encode_sample(sample(direction=2))


class EncoderPacketMonitorTest(unittest.TestCase):
    def test_allowed_sender_hostname_is_resolved_to_ipv4(self):
        monitor = EncoderPacketMonitor("localhost")
        packet = encode_sample(sample(sequence=0))

        accepted = monitor.accept(packet, ("127.0.0.1", 5011), 1.0)

        self.assertIsNotNone(accepted)
        self.assertEqual(monitor.allowed_sender, "127.0.0.1")
        self.assertEqual(monitor.rejected_sender_count, 0)

    def test_nonmatching_resolved_sender_is_rejected(self):
        monitor = EncoderPacketMonitor("localhost")
        packet = encode_sample(sample(sequence=0))

        accepted = monitor.accept(packet, ("192.0.2.11", 5011), 1.0)

        self.assertIsNone(accepted)
        self.assertEqual(monitor.rejected_sender_count, 1)

    def test_gap_and_session_restart_are_recorded(self):
        monitor = EncoderPacketMonitor()
        sender = ("192.0.2.11", 5011)
        monitor.accept(encode_sample(sample(sequence=0)), sender, 1.0)
        monitor.accept(encode_sample(sample(sequence=2)), sender, 1.1)
        monitor.accept(
            encode_sample(sample(session_id="new-session", sequence=0)),
            sender,
            1.2,
        )

        self.assertEqual(monitor.accepted_packets, 3)
        self.assertEqual(monitor.sequence_gaps, 1)
        self.assertEqual(monitor.session_changes, 1)

    def test_duplicate_is_rejected(self):
        monitor = EncoderPacketMonitor()
        sender = ("192.0.2.11", 5011)
        packet = encode_sample(sample(sequence=4))
        self.assertIsNotNone(monitor.accept(packet, sender, 1.0))
        self.assertIsNone(monitor.accept(packet, sender, 1.1))
        self.assertEqual(monitor.duplicate_packets, 1)


if __name__ == "__main__":
    unittest.main()
