import unittest

from experiments.real_vehicle.evaluation.actuator_transport_trial import (
    evaluate_transport_run,
    percentile,
)


def command(sequence_id, sent_at, interval=""):
    return {
        "sequence_id": sequence_id,
        "scheduled_monotonic_s": sent_at,
        "sent_monotonic_s": sent_at,
        "schedule_lateness_s": 0.0,
        "send_interval_s": interval,
    }


def status(
    sequence_id,
    received_at,
    accepted,
    rejected=0,
    watchdog=4,
    phase="stream",
    session_id="trial",
    reason="transport_keepalive",
):
    return {
        "received_monotonic_s": received_at,
        "phase": phase,
        "session_id": session_id,
        "sequence_id": sequence_id,
        "state": "stopped",
        "reason": reason,
        "hardware_output_enabled": False,
        "accepted_packets": accepted,
        "rejected_packets": rejected,
        "watchdog_count": watchdog,
    }


class ActuatorTransportTrialTest(unittest.TestCase):
    def test_percentile_interpolates_values(self):
        self.assertAlmostEqual(percentile([0.0, 1.0], 95.0), 0.95)
        self.assertIsNone(percentile([], 95.0))

    def test_passes_complete_neutral_stream_and_post_watchdog(self):
        commands = [
            command(1, 1.00),
            command(2, 1.05, 0.05),
            command(3, 1.10, 0.05),
        ]
        rows = [
            status(1, 1.01, 11),
            status(2, 1.06, 12),
            status(3, 1.11, 13),
            status(
                0,
                1.31,
                13,
                watchdog=5,
                phase="post_stream",
                session_id="",
                reason="command_timeout",
            ),
        ]
        baseline = status(0, 0.9, 10, watchdog=4)

        result = evaluate_transport_run(
            commands,
            rows,
            "trial",
            baseline,
            watchdog_timeout_s=0.20,
            minimum_status_coverage=0.99,
        )

        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["accepted_packets_during_stream"], 3)
        self.assertTrue(result["post_stream_watchdog_observed"])

    def test_detects_session_release_watchdog_during_stream(self):
        commands = [
            command(1, 1.00),
            command(2, 1.05, 0.05),
        ]
        rows = [
            status(1, 1.01, 11),
            status(
                0,
                1.16,
                11,
                rejected=1,
                watchdog=5,
                phase="stream",
                session_id="",
                reason="invalid_command:session is already closed",
            ),
            status(
                0,
                1.35,
                11,
                rejected=1,
                watchdog=5,
                phase="post_stream",
                session_id="",
                reason="command_timeout",
            ),
        ]
        baseline = status(0, 0.9, 10, watchdog=4)

        result = evaluate_transport_run(
            commands,
            rows,
            "trial",
            baseline,
            watchdog_timeout_s=0.10,
            minimum_status_coverage=0.99,
        )

        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["watchdog_during_stream"])
        self.assertIn(
            "watchdog fired during the command stream",
            result["failures"],
        )

    def test_detects_send_interval_reaching_watchdog_timeout(self):
        commands = [
            command(1, 1.00),
            command(2, 1.20, 0.20),
        ]
        rows = [
            status(1, 1.01, 11),
            status(2, 1.21, 12),
            status(
                0,
                1.42,
                12,
                watchdog=5,
                phase="post_stream",
                session_id="",
                reason="command_timeout",
            ),
        ]
        baseline = status(0, 0.9, 10, watchdog=4)

        result = evaluate_transport_run(
            commands,
            rows,
            "trial",
            baseline,
            watchdog_timeout_s=0.20,
            minimum_status_coverage=0.99,
        )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["send_interval_overruns"], 1)


if __name__ == "__main__":
    unittest.main()
