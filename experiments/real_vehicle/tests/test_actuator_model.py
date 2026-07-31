import unittest

import numpy as np

from experiments.real_vehicle.models import ActuatorModel, VehicleLimits, VehicleParameters


class ActuatorModelTest(unittest.TestCase):
    def setUp(self):
        self.parameters = VehicleParameters(wheelbase_m=0.25)
        self.limits = VehicleLimits(speed_max_mps=2.0)
        self.state = np.zeros(4, dtype=float)

    def test_first_order_speed_does_not_jump_to_target(self):
        actuator = ActuatorModel(
            self.parameters,
            self.limits,
            {"enabled": True, "speed_time_constant_s": 0.2, "steer_time_constant_s": 0.1},
        )
        actuator.reset(self.state)
        _next_state, _clamp, diagnostics = actuator.step(
            self.state,
            [1.0, 0.0],
            self.parameters,
            0.01,
        )
        self.assertGreater(diagnostics.applied_v, 0.0)
        self.assertLess(diagnostics.applied_v, 1.0)

    def test_disabled_actuator_preserves_direct_input(self):
        actuator = ActuatorModel(
            self.parameters,
            self.limits,
            {"enabled": False},
        )
        actuator.reset(self.state)
        _next_state, _clamp, diagnostics = actuator.step(
            self.state,
            [1.0, 0.0],
            self.parameters,
            0.01,
        )
        self.assertAlmostEqual(diagnostics.applied_v, 1.0)

    def test_battery_scale_and_longitudinal_slip_reduce_speed(self):
        actuator = ActuatorModel(
            self.parameters,
            self.limits,
            {
                "enabled": True,
                "battery_speed_scale": 0.8,
                "longitudinal_slip_ratio": 0.25,
            },
        )
        actuator.reset(self.state)
        _state, _clamp, diagnostics = actuator.step(
            self.state,
            [1.0, 0.0],
            self.parameters,
            0.01,
        )
        self.assertAlmostEqual(diagnostics.target_v, 0.8)
        self.assertAlmostEqual(diagnostics.applied_v, 0.6)

    def test_steering_deadband_holds_small_changes(self):
        actuator = ActuatorModel(
            self.parameters,
            self.limits,
            {"enabled": True, "steer_deadband_rad": 0.02},
        )
        actuator.reset(self.state)
        _state, _clamp, diagnostics = actuator.step(
            self.state,
            [0.0, 0.5],
            self.parameters,
            0.01,
        )
        self.assertAlmostEqual(diagnostics.applied_psi, 0.0)

    def test_cornering_gain_changes_heading_response(self):
        state = np.array([0.0, 0.0, 0.0, 0.2])
        low = ActuatorModel(
            self.parameters,
            self.limits,
            {"enabled": True, "cornering_gain": 0.5},
        )
        high = ActuatorModel(
            self.parameters,
            self.limits,
            {"enabled": True, "cornering_gain": 1.0},
        )
        low.reset(state)
        high.reset(state)
        low_state, _clamp, _diag = low.step(state, [1.0, 0.0], self.parameters, 0.1)
        high_state, _clamp, _diag = high.step(state, [1.0, 0.0], self.parameters, 0.1)
        self.assertLess(low_state[0], high_state[0])


if __name__ == "__main__":
    unittest.main()
