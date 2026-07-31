import unittest
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.evaluate_lidar_mppi_next_plan import (  # noqa: E402
    TRUTH_6P1_VARIANTS,
    build_robustness_conditions,
)


class HighSpeedNextPlanTest(unittest.TestCase):
    def test_robustness_grid_contains_delay_dropout_and_combined_cases(self):
        conditions = build_robustness_conditions()

        self.assertIn("delay_0.05", conditions)
        self.assertIn("dropout_0.10", conditions)
        self.assertIn("combined_d0.10_p0.20", conditions)
        self.assertEqual(len(conditions), 19)

    def test_truth_speed_variants_cover_controller_axes(self):
        self.assertIn("h8_s64_full", TRUTH_6P1_VARIANTS)
        self.assertIn("h12_s128_full", TRUTH_6P1_VARIANTS)
        self.assertIn("h12_s64_target5p5_limited", TRUTH_6P1_VARIANTS)
        self.assertEqual(
            TRUTH_6P1_VARIANTS["h12_s64_target5p5_limited"]["steer_limit"],
            0.36,
        )


if __name__ == "__main__":
    unittest.main()
