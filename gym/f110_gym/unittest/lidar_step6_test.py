import csv
import tempfile
import unittest
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_step6_mppi import run_experiment


class LidarStep6Test(unittest.TestCase):
    def test_mppi_conditions_write_timing_and_tracking_summary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            results, summary_path = run_experiment(
                Path("experiments/configs/homur_oval_mppi.yaml"),
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
                [
                    "M0_baseline",
                    "M1_short_horizon",
                    "M2_long_horizon",
                    "M3_more_samples",
                ],
            )
            self.assertTrue(all(int(row["row_count"]) == 3 for row in rows))
            self.assertTrue(all(float(row["mean_plan_time_s"]) >= 0.0 for row in rows))
            self.assertTrue(
                all(Path(row["log_csv"]).exists() for row in rows)
            )


if __name__ == "__main__":
    unittest.main()
