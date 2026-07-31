import unittest

from experiments.real_vehicle.evaluation.encoder_duty_trial import (
    calculate_cumulative_counter_change,
    calculate_invalid_transition_change,
    calculate_trial_metrics,
    count_at_or_before,
    evaluate_status_range,
    session_counter_increased,
    validate_stage_c_settings,
    watchdog_fired_during_motion,
)


class EncoderDutyTrialTest(unittest.TestCase):
    def test_calculates_powered_coast_and_total_distance(self):
        result = calculate_trial_metrics(100, 600, 650, 0.0014)

        self.assertEqual(result["powered_count_delta"], 500)
        self.assertEqual(result["coast_count_delta"], 50)
        self.assertEqual(result["total_count_delta"], 550)
        self.assertAlmostEqual(result["powered_distance_m"], 0.7)
        self.assertAlmostEqual(result["coast_distance_m"], 0.07)
        self.assertAlmostEqual(result["total_distance_m"], 0.77)

    def test_count_selection_uses_sample_before_command(self):
        rows = [
            {"received_monotonic_s": 1.0, "cumulative_count": 10},
            {"received_monotonic_s": 1.1, "cumulative_count": 20},
            {"received_monotonic_s": 1.2, "cumulative_count": 30},
        ]

        self.assertEqual(count_at_or_before(rows, 1.15), 20)

    def test_historical_invalid_transition_count_is_not_a_new_error(self):
        stable = [
            {"invalid_transition_count": 3},
            {"invalid_transition_count": 3},
        ]
        increased = [
            {"invalid_transition_count": 3},
            {"invalid_transition_count": 4},
        ]

        self.assertEqual(
            calculate_invalid_transition_change(stable),
            {"baseline": 3, "final": 3, "change": 0},
        )
        self.assertEqual(
            calculate_invalid_transition_change(increased),
            {"baseline": 3, "final": 4, "change": 1},
        )

    def test_historical_agent_rejections_are_not_new_rejections(self):
        first = {"rejected_packets": 5}
        stable = {"rejected_packets": 5}
        increased = {"rejected_packets": 7}

        self.assertEqual(
            calculate_cumulative_counter_change(
                first,
                stable,
                "rejected_packets",
            ),
            {"baseline": 5, "final": 5, "change": 0},
        )
        self.assertEqual(
            calculate_cumulative_counter_change(
                first,
                increased,
                "rejected_packets",
            ),
            {"baseline": 5, "final": 7, "change": 2},
        )

    def test_delayed_and_duplicate_statuses_are_evaluated_as_a_batch(self):
        rows = [
            {
                "phase": "motion",
                "session_id": "trial",
                "sequence_id": 1,
                "state": "running",
                "reason": "stage_c_motion",
            },
            {
                "phase": "emergency_stop",
                "session_id": "trial",
                "sequence_id": 2,
                "state": "running",
                "reason": "stage_c_motion",
            },
            {
                "phase": "emergency_stop",
                "session_id": "trial",
                "sequence_id": 2,
                "state": "running",
                "reason": "stage_c_motion",
            },
            {
                "phase": "other",
                "session_id": "different",
                "sequence_id": 2,
                "state": "fault",
                "reason": "invalid_command",
            },
        ]

        result = evaluate_status_range(
            rows,
            "trial",
            1,
            2,
            "running",
            "stage_c_motion",
        )

        self.assertEqual(result["command_count"], 2)
        self.assertEqual(result["received_sequence_count"], 2)
        self.assertEqual(result["valid_sequence_count"], 2)
        self.assertEqual(result["received_coverage"], 1.0)
        self.assertEqual(result["valid_coverage"], 1.0)
        self.assertEqual(result["invalid_sequence_ids"], [])

    def test_status_batch_reports_an_invalid_matching_sequence(self):
        rows = [
            {
                "session_id": "trial",
                "sequence_id": 3,
                "state": "fault",
                "reason": "invalid_command",
            },
        ]

        result = evaluate_status_range(
            rows,
            "trial",
            3,
            4,
            "stopped",
            "stage_c_explicit_stop",
        )

        self.assertEqual(result["received_sequence_count"], 1)
        self.assertEqual(result["valid_sequence_count"], 0)
        self.assertEqual(result["invalid_sequence_ids"], [3])

    def test_delayed_old_watchdog_status_is_not_a_motion_watchdog(self):
        rows = [
            {
                "session_id": "",
                "sequence_id": 0,
                "watchdog_count": 83,
                "reason": "command_timeout",
            },
            {
                "session_id": "motion",
                "sequence_id": 1,
                "watchdog_count": 83,
                "reason": "stage_c_motion",
            },
            {
                "session_id": "motion",
                "sequence_id": 21,
                "watchdog_count": 83,
                "reason": "stage_c_explicit_stop",
            },
            {
                "session_id": "",
                "sequence_id": 0,
                "watchdog_count": 84,
                "reason": "command_timeout",
            },
        ]

        self.assertFalse(
            session_counter_increased(
                rows,
                "motion",
                "watchdog_count",
                83,
            )
        )

    def test_watchdog_counter_increment_in_motion_session_is_detected(self):
        rows = [
            {
                "session_id": "motion",
                "sequence_id": 1,
                "watchdog_count": 83,
            },
            {
                "session_id": "motion",
                "sequence_id": 2,
                "watchdog_count": 84,
            },
        ]

        self.assertTrue(
            session_counter_increased(
                rows,
                "motion",
                "watchdog_count",
                83,
            )
        )

    def test_watchdog_after_session_release_is_detected_during_motion(self):
        rows = [
            {
                "phase": "motion",
                "session_id": "motion",
                "watchdog_count": 116,
            },
            {
                "phase": "motion",
                "session_id": "",
                "watchdog_count": 117,
                "reason": "invalid_command:session is already closed",
            },
        ]

        self.assertTrue(
            watchdog_fired_during_motion(rows, "motion", 116)
        )

    def test_watchdog_after_motion_is_not_a_motion_watchdog(self):
        rows = [
            {
                "phase": "motion",
                "session_id": "motion",
                "watchdog_count": 116,
            },
            {
                "phase": "coast",
                "session_id": "",
                "watchdog_count": 117,
                "reason": "command_timeout",
            },
        ]

        self.assertFalse(
            watchdog_fired_during_motion(rows, "motion", 116)
        )

    def test_stage_c_rejects_unapproved_duty_or_duration(self):
        validate_stage_c_settings(10.160, 10.895, 1.0, 20.0)
        validate_stage_c_settings(10.140, 10.895, 1.0, 20.0)
        validate_stage_c_settings(10.140, 10.895, 2.0, 20.0)
        validate_stage_c_settings(10.120, 10.895, 1.0, 20.0)
        validate_stage_c_settings(10.100, 10.895, 1.0, 20.0)
        validate_stage_c_settings(10.100, 10.895, 2.0, 20.0)
        validate_stage_c_settings(10.100, 10.895, 3.0, 20.0)
        with self.assertRaises(ValueError):
            validate_stage_c_settings(10.080, 10.895, 1.0, 20.0)
        with self.assertRaises(ValueError):
            validate_stage_c_settings(10.160, 10.800, 1.0, 20.0)
        with self.assertRaises(ValueError):
            validate_stage_c_settings(10.160, 10.895, 6.0, 20.0)
        with self.assertRaises(ValueError):
            validate_stage_c_settings(10.140, 10.895, 2.1, 20.0)
        with self.assertRaises(ValueError):
            validate_stage_c_settings(10.120, 10.895, 2.0, 20.0)
        with self.assertRaises(ValueError):
            validate_stage_c_settings(10.100, 10.895, 3.1, 20.0)
        with self.assertRaises(ValueError):
            validate_stage_c_settings(10.160, 10.895, 1.0, 10.0)


if __name__ == "__main__":
    unittest.main()
