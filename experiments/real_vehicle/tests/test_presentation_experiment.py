import csv
import tempfile
import unittest
from pathlib import Path

from experiments.real_vehicle.evaluation.presentation_experiment import (
    aggregate_results,
    run_presentation_experiment,
)


class PresentationExperimentTest(unittest.TestCase):
    def test_aggregate_results_keeps_controller_groups_separate(self):
        rows = []
        for controller, rmse, timing in (("mpc", 0.02, 5.0), ("mppi", 0.04, 2.0)):
            rows.append({
                "controller": controller,
                "completed": 1,
                "position_rmse_m": rmse,
                "controller_mean_ms": timing,
                "controller_p95_ms": timing + 1.0,
                "controller_max_ms": timing + 2.0,
                "deadline_exceeded_count": 0,
                "circle_radius_final_error_m": rmse / 2.0,
                "circle_phase_error_rad": 0.01,
            })
        summary = aggregate_results(rows)
        self.assertEqual([row["controller"] for row in summary], ["mpc", "mppi"])
        self.assertEqual(summary[0]["position_rmse_mean_m"], 0.02)
        self.assertEqual(summary[1]["controller_p95_mean_ms"], 3.0)

    def test_smoke_run_writes_data_figures_and_report(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "presentation"
            report = run_presentation_experiment(
                output_dir=output,
                seeds=(123,),
                max_steps=3,
            )
            self.assertTrue(report.exists())
            self.assertTrue((output / "trajectory_comparison.svg").exists())
            self.assertTrue((output / "position_error_timeseries.svg").exists())
            self.assertTrue((output / "accuracy_and_timing.svg").exists())
            self.assertTrue((output / "metadata.json").exists())
            self.assertIn(
                "MPCとMPPIによる円形軌道追従",
                (output / "trajectory_comparison.svg").read_text(encoding="utf-8"),
            )
            self.assertIn(
                "MPCとMPPIの精度・計算時間比較",
                (output / "accuracy_and_timing.svg").read_text(encoding="utf-8"),
            )
            english_dir = output / "english"
            if english_dir.exists():
                self.assertEqual(
                    {path.suffix for path in english_dir.iterdir()},
                    {".png"},
                )
                self.assertEqual(len(list(english_dir.iterdir())), 3)
            with (output / "runs.csv").open("r", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 2)
            self.assertEqual({row["controller"] for row in rows}, {"mpc", "mppi"})


if __name__ == "__main__":
    unittest.main()
