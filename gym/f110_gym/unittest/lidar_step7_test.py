import csv
import tempfile
import unittest
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_step7_localized_mppi import run_experiment


class LidarStep7Test(unittest.TestCase):
    def test_truth_and_estimated_mppi_conditions_write_summary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            results, summary_path = run_experiment(
                Path("experiments/configs/homur_oval_localized_a1_mppi_step7.yaml"),
                max_steps=2,
                lap_target=0,
                seed=123,
                output_dir=Path(temp_dir),
            )

            self.assertEqual(len(results), 5)
            self.assertTrue(summary_path.exists())
            with summary_path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(
                [row["condition"] for row in rows],
                [
                    "A_truth_no_lidar",
                    "B_estimated_ideal",
                    "C_estimated_noise",
                    "D_estimated_noise_delay",
                    "E_estimated_noise_delay_dropout",
                ],
            )
            self.assertEqual(rows[0]["mode"], "truth")
            self.assertTrue(all(row["mode"] == "estimated" for row in rows[1:]))
            self.assertTrue(all(int(row["row_count"]) == 3 for row in rows))
            self.assertTrue(
                all(float(row["mean_plan_time_s"]) >= 0.0 for row in rows)
            )
            self.assertTrue(
                all(float(row["mean_localizer_time_s"]) >= 0.0 for row in rows)
            )
            self.assertTrue(all(Path(row["log_csv"]).exists() for row in rows))


if __name__ == "__main__":
    unittest.main()
