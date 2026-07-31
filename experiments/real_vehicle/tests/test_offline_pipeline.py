import tempfile
import unittest
from pathlib import Path

from experiments.real_vehicle.evaluation.map_simulation import run_map_case
from experiments.real_vehicle.evaluation.offline_pipeline import (
    _aggregate_map_runs,
)


class OfflinePipelineTest(unittest.TestCase):
    def test_saved_map_case_runs_without_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            summary, rows = run_map_case(
                "pure_pursuit",
                "ideal",
                123,
                output_dir=Path(directory),
                duration_s=0.30,
                distance_m=0.03,
            )
            self.assertEqual(summary["hardware_output_enabled"], False)
            self.assertEqual(summary["completed"], 1)
            self.assertGreater(len(rows), 0)
            self.assertTrue(
                any((Path(directory) / "trajectories").iterdir())
            )

    def test_aggregation_keeps_controller_and_lidar_groups_separate(self):
        rows = []
        for controller in ("pure_pursuit", "mpc", "mppi"):
            rows.append({
                "controller": controller,
                "lidar_condition": "ideal",
                "completed": 1,
                "position_rmse_m": 0.01,
                "localization_xy_rmse_m": 0.02,
                "controller_mean_ms": 1.0,
                "controller_p95_ms": 2.0,
                "localizer_failure_rate": 0.0,
                "minimum_map_clearance_m": 0.2,
            })
        summary = _aggregate_map_runs(rows)
        self.assertEqual(len(summary), 3)
        self.assertEqual(
            {row["controller"] for row in summary},
            {"pure_pursuit", "mpc", "mppi"},
        )


if __name__ == "__main__":
    unittest.main()
