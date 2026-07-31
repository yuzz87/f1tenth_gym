import unittest

import numpy as np

from experiments.real_vehicle.models import VehicleLimits, VehicleParameters, step_model


class BicycleModelTest(unittest.TestCase):
    def setUp(self):
        self.parameters = VehicleParameters()
        self.limits = VehicleLimits()

    def test_stop_does_not_move(self):
        state = np.array([0.2, 1.0, -2.0, 0.1])
        next_state, _ = step_model(state, [0.0, 0.0], self.parameters, self.limits, 0.01)
        np.testing.assert_allclose(next_state[:3], state[:3], atol=1e-12)

    def test_straight_motion_keeps_heading(self):
        state = np.zeros(4)
        next_state, _ = step_model(state, [0.3, 0.0], self.parameters, self.limits, 0.01)
        self.assertAlmostEqual(next_state[1], 0.003, places=6)
        self.assertAlmostEqual(next_state[2], 0.0, places=12)
        self.assertAlmostEqual(next_state[0], 0.0, places=12)

    def test_turning_rate_is_constrained(self):
        state = np.zeros(4)
        next_state, result = step_model(state, [2.0, 3.0], self.parameters, self.limits, 0.01)
        self.assertTrue(result.clamped_speed)
        self.assertTrue(result.clamped_steer)
        self.assertLessEqual(abs(next_state[3]), self.limits.steer_max_rad)
        self.assertLessEqual(abs(result.applied_omega), self.limits.steer_rate_max_rad_s)

    def test_circle_curvature(self):
        radius = 2.0
        steer = np.arctan(self.parameters.wheelbase_m / radius)
        state = np.array([0.0, 0.0, 0.0, steer])
        next_state, _ = step_model(state, [0.3, 0.0], self.parameters, self.limits, 0.01)
        expected_yaw_rate = 0.3 / self.parameters.wheelbase_m * np.tan(steer)
        self.assertAlmostEqual(next_state[0] / 0.01, expected_yaw_rate, places=5)


if __name__ == "__main__":
    unittest.main()
