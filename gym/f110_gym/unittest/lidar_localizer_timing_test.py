import csv
import tempfile
import unittest
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_localizer_timing import run_experiment


class LidarLocalizerTimingTest(unittest.TestCase):
    def test_localizer_sweep_writes_timing_comparison(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            results, summary_path = run_experiment(
                Path(
                    "experiments/configs/"
                    "homur_oval_localized_a1_mppi_step7.yaml"
                ),
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
                    "baseline",
                    "reduced_beams",
                    "reduced_candidates",
                    "fast_combined",
                ],
            )
            self.assertEqual(rows[0]["candidate_count"], "175")
            self.assertEqual(rows[2]["candidate_count"], "45")
            self.assertEqual(rows[1]["scan_beams"], "90")
            self.assertEqual(rows[3]["preserve_scan_angles"], "0")
            self.assertTrue(
                all(float(row["mean_localizer_time_ms"]) >= 0.0 for row in rows)
            )


if __name__ == "__main__":
    unittest.main()
