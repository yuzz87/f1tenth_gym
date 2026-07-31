import unittest

import numpy as np

from experiments.real_vehicle.adapters import (
    CommandAdapter,
    esc_duty_to_speed,
    sanitize_esc_duty,
    speed_to_esc_duty,
    steer_angle_to_duty,
    steer_duty_to_angle,
)
from experiments.real_vehicle.models import VehicleLimits


class CommandAdapterTest(unittest.TestCase):
    def test_steering_round_trip(self):
        for angle in (-0.314, 0.0, 0.314):
            self.assertAlmostEqual(
                steer_duty_to_angle(steer_angle_to_duty(angle)),
                angle,
                places=10,
            )

    def test_measured_neutral_duty_is_zero_angle(self):
        neutral_duty = 10.895

        self.assertAlmostEqual(
            steer_angle_to_duty(0.0, neutral_duty),
            neutral_duty,
        )
        self.assertAlmostEqual(
            steer_duty_to_angle(neutral_duty, neutral_duty),
            0.0,
        )

    def test_virtual_speed_duty_round_trip(self):
        for speed in (0.0, 0.05, 0.15, 0.30):
            duty = speed_to_esc_duty(speed, speed_command_max_mps=0.30)
            self.assertAlmostEqual(
                esc_duty_to_speed(duty, speed_command_max_mps=0.30),
                speed,
                places=6,
            )

    def test_stop_start_and_forward_limits(self):
        self.assertEqual(sanitize_esc_duty(10.30), (10.30, False))
        self.assertEqual(sanitize_esc_duty(10.20), (10.16, True))
        self.assertEqual(sanitize_esc_duty(10.10), (10.10, False))
        self.assertEqual(sanitize_esc_duty(9.70), (10.10, True))

    def test_experimental_duty_can_go_below_hardware_limit(self):
        self.assertEqual(
            sanitize_esc_duty(9.70, allow_experimental_duty=True),
            (9.70, False),
        )

    def test_adapter_hardware_mode_clamps_duty_below_limit(self):
        adapter = CommandAdapter(VehicleLimits(), 0.01)
        applied, clamped = adapter.normalize_esc_duty(9.70)
        self.assertTrue(clamped)
        self.assertAlmostEqual(applied, 10.10)

    def test_stop_is_zero_virtual_speed(self):
        self.assertEqual(esc_duty_to_speed(10.30), 0.0)
        adapter = CommandAdapter(VehicleLimits(), 0.01)
        action = adapter.duty_to_gym_command(
            steer_angle_to_duty(0.0, 10.895),
            10.30,
        )
        np.testing.assert_allclose(action, [0.0, 0.0])

    def test_model_command_contains_duty_diagnostics(self):
        adapter = CommandAdapter(VehicleLimits(), 0.01)
        action, result = adapter.model_input_to_command(np.zeros(4), [2.0, 2.0])
        self.assertTrue(result.clamped_speed)
        self.assertTrue(result.clamped_steer)
        self.assertFalse(result.clamped_esc_duty)
        self.assertAlmostEqual(result.esc_duty_percent, 10.10)
        self.assertEqual(action.shape, (2,))


if __name__ == "__main__":
    unittest.main()
