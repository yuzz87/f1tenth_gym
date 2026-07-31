import csv
import tempfile
import unittest
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_high_speed_localized_mppi import run_experiment


class HighSpeedLocalizedMppiTest(unittest.TestCase):
    def test_localized_high_speed_conditions_write_summary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            results, summary_path = run_experiment(
                Path(
                    "experiments/configs/"
                    "homur_oval_localized_a1_mppi_step8_fast.yaml"
                ),
                speeds=[3.0],
                max_steps=2,
                lap_target=0,
                seed=123,
                output_dir=Path(temp_dir),
            )

            self.assertEqual(len(results), 3)
            self.assertTrue(summary_path.exists())
            with summary_path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(
                [row["lidar_condition"] for row in rows],
                ["ideal", "noise", "noise_delay_dropout"],
            )
            self.assertTrue(
                all(float(row["mean_localizer_time_ms"]) >= 0.0 for row in rows)
            )


if __name__ == "__main__":
    unittest.main()
