import csv
import tempfile
import unittest
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_high_speed_pure_pursuit import run_experiment


class HighSpeedPurePursuitTest(unittest.TestCase):
    def test_truth_and_estimated_conditions_write_summary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            results, summary_path = run_experiment(
                Path(
                    "experiments/configs/"
                    "homur_oval_localized_a1_mppi_safe_3mps.yaml"
                ),
                speeds=[3.0],
                max_steps=2,
                lap_target=0,
                seed=123,
                output_dir=Path(temp_dir),
            )

            self.assertEqual(len(results), 4)
            self.assertTrue(summary_path.exists())
            with summary_path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(
                [row["condition"] for row in rows],
                ["truth", "ideal", "noise", "noise_delay_dropout"],
            )
            self.assertEqual(rows[0]["mode"], "truth")
            self.assertTrue(all(float(row["mean_abs_cross_track_m"]) >= 0.0 for row in rows))


if __name__ == "__main__":
    unittest.main()
