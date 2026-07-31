import csv
import tempfile
import unittest
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_localizer_robustness import run_experiment


class LidarLocalizerRobustnessTest(unittest.TestCase):
    def test_seed_and_speed_sweep_writes_raw_and_aggregate_csv(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            results, aggregate, raw_path, aggregate_path = run_experiment(
                Path(
                    "experiments/configs/"
                    "homur_oval_localized_a1_mppi_step8_fast.yaml"
                ),
                seeds=[123],
                speeds=[1.0],
                max_steps=2,
                lap_target=0,
                output_dir=Path(temp_dir),
            )

            self.assertEqual(len(results), 3)
            self.assertEqual(len(aggregate), 3)
            self.assertTrue(raw_path.exists())
            self.assertTrue(aggregate_path.exists())
            with raw_path.open(newline="", encoding="utf-8") as file:
                raw_rows = list(csv.DictReader(file))
            with aggregate_path.open(newline="", encoding="utf-8") as file:
                aggregate_rows = list(csv.DictReader(file))

            self.assertEqual(
                [row["lidar_condition"] for row in raw_rows],
                ["ideal", "noise", "noise_delay_dropout"],
            )
            self.assertTrue(
                all(row["seed_count"] == "1" for row in aggregate_rows)
            )
            self.assertTrue(
                all(row["lap_target"] == "0" for row in aggregate_rows)
            )
            self.assertTrue(
                all(row["lap_success_rate"] == "" for row in aggregate_rows)
            )


if __name__ == "__main__":
    unittest.main()
