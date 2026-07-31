import math
import tempfile
import unittest
from pathlib import Path

from experiments.real_vehicle.evaluation.encoder_calibration import (
    known_distance_calibration,
    load_encoder_csv,
    summarize_rows,
    wheel_turn_calibration,
)


class EncoderCalibrationTest(unittest.TestCase):
    def test_wheel_turn_calibration(self):
        result = wheel_turn_calibration(0, 36, 1.0, 0.066)

        self.assertAlmostEqual(result["counts_per_revolution"], 36.0)
        self.assertAlmostEqual(
            result["distance_per_count_m"], math.pi * 0.066 / 36.0
        )

    def test_known_distance_calibration(self):
        result = known_distance_calibration(10, 210, 1.0, 36.0)

        self.assertEqual(result["count_delta"], 200)
        self.assertAlmostEqual(result["distance_per_count_m"], 0.005)
        self.assertAlmostEqual(
            result["effective_wheel_diameter_m"], 1.0 * 36.0 / 200.0 / math.pi
        )

    def test_csv_summary(self):
        csv_text = (
            "sequence,cumulative_count,delta_count,direction,"
            "invalid_transition_count,pulse_age_s\n"
            "1,0,0,0,0,1.0\n"
            "2,4,4,1,0,0.01\n"
            "3,2,-2,-1,0,0.02\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "encoder.csv"
            path.write_text(csv_text, encoding="utf-8")
            rows = load_encoder_csv(path)

        summary = summarize_rows(rows)
        self.assertEqual(summary["first_count"], 0)
        self.assertEqual(summary["last_count"], 2)
        self.assertEqual(summary["max_invalid_transition_count"], 0)
        self.assertEqual(summary["direction_samples"][1], 1)
        self.assertEqual(summary["direction_samples"][-1], 1)
        self.assertAlmostEqual(
            summary["provisional_count_per_meter"],
            144.0 / (math.pi * 0.066),
        )

    def test_invalid_calibration_is_rejected(self):
        with self.assertRaises(ValueError):
            wheel_turn_calibration(0, 0, 1.0)
        with self.assertRaises(ValueError):
            known_distance_calibration(0, 1, 0.0)


if __name__ == "__main__":
    unittest.main()
