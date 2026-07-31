"""Headless checks for the GUI simulation data source."""

import unittest

from experiments.real_vehicle.evaluation.common import load_real_config
from experiments.real_vehicle.gui.simulation_view import SimulationView


class GuiSimulationTest(unittest.TestCase):
    def setUp(self):
        config, parameters, limits = load_real_config()
        self.config = config
        self.parameters = parameters
        self.limits = limits

    def test_straight_frame_contains_scan_and_prediction(self):
        view = SimulationView(
            self.config,
            self.parameters,
            self.limits,
            controller_name="mppi",
            scenario="straight",
            max_steps=1,
        )
        frame = view.step()
        self.assertEqual(len(frame["scan"]["ranges"]), 72)
        self.assertEqual(frame["info"]["predicted_states"].shape[1], 4)
        self.assertEqual(len(view.rows), 1)

    def test_localized_frame_contains_estimated_pose(self):
        view = SimulationView(
            self.config,
            self.parameters,
            self.limits,
            controller_name="mppi",
            scenario="localized",
            noise_std=0.01,
            dropout_probability=0.1,
            delay_s=0.05,
            max_steps=1,
        )
        frame = view.step()
        self.assertIsNotNone(frame["estimated_state"])
        self.assertGreaterEqual(frame["scan"]["range_min"], 0.0)
        self.assertTrue(frame["info"]["success"])


if __name__ == "__main__":
    unittest.main()

