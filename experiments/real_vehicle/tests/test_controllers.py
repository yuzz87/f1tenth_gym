import unittest

import numpy as np

from experiments.real_vehicle.controllers import (
    RealVehicleMPC,
    RealVehicleMPPI,
    RealVehiclePurePursuit,
)
from experiments.real_vehicle.models import VehicleLimits, VehicleParameters


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.parameters = VehicleParameters()
        self.limits = VehicleLimits()
        self.config = {
            "horizon": 4,
            "max_iterations": 4,
            "num_samples": 16,
            "speed_target_mps": 0.3,
            "seed": 1,
        }
        self.reference = np.column_stack((
            np.zeros(4),
            np.arange(1, 5) * 0.003,
            np.full(4, -2.0),
            np.zeros(4),
            np.full(4, 0.3),
            np.zeros(4),
        ))

    def test_mpc_returns_bounded_two_input_command(self):
        controller = RealVehicleMPC(self.parameters, self.limits, 0.01, self.config)
        control, info = controller.plan([0.0, 0.0, -2.0, 0.0], self.reference)
        self.assertEqual(control.shape, (2,))
        self.assertGreaterEqual(control[0], self.limits.speed_min_mps)
        self.assertLessEqual(control[0], self.limits.speed_max_mps)
        self.assertTrue(np.isfinite(info["solve_time_s"]))

    def test_mpc_control_blocking_reduces_optimizer_variables(self):
        config = dict(self.config, horizon=8, mpc_control_blocks=3)
        reference = np.repeat(self.reference[-1:, :], 8, axis=0)
        controller = RealVehicleMPC(self.parameters, self.limits, 0.01, config)
        _control, info = controller.plan([0.0, 0.0, -2.0, 0.0], reference)
        self.assertEqual(info["control_blocks"], 3)
        self.assertEqual(info["optimizer_variables"], 6)
        self.assertGreater(info["function_evaluations"], 0)
        self.assertEqual(info["predicted_states"].shape, (8, 4))

    def test_mpc_control_blocks_are_clamped_to_horizon(self):
        controller = RealVehicleMPC(
            self.parameters,
            self.limits,
            0.01,
            dict(self.config, horizon=3, mpc_control_blocks=99),
        )
        self.assertEqual(controller.control_blocks, 3)

    def test_mpc_analytic_gradient_matches_central_difference(self):
        config = dict(
            self.config,
            horizon=4,
            mpc_control_blocks=4,
            mpc_deadline_s=0.0,
        )
        controller = RealVehicleMPC(self.parameters, self.limits, 0.01, config)
        state = np.array([0.03, 0.01, -1.98, 0.02])
        controls = np.array([
            [0.22, 0.03],
            [0.24, 0.02],
            [0.25, -0.01],
            [0.26, -0.02],
        ])
        _cost, analytic = controller._cost_and_gradient(
            state,
            controls,
            self.reference,
        )
        numeric = np.zeros_like(controls)
        epsilon = 1e-6
        for row in range(controls.shape[0]):
            for column in range(controls.shape[1]):
                plus = controls.copy()
                minus = controls.copy()
                plus[row, column] += epsilon
                minus[row, column] -= epsilon
                numeric[row, column] = (
                    controller.cost(state, plus, self.reference)
                    - controller.cost(state, minus, self.reference)
                ) / (2.0 * epsilon)
        np.testing.assert_allclose(analytic, numeric, rtol=2e-4, atol=2e-5)

    def test_mpc_deadline_returns_a_bounded_fallback(self):
        config = dict(self.config, mpc_deadline_s=1e-12)
        controller = RealVehicleMPC(self.parameters, self.limits, 0.01, config)
        control, info = controller.plan([0.0, 0.0, -2.0, 0.0], self.reference)
        self.assertTrue(info["deadline_exceeded"])
        self.assertGreaterEqual(control[0], self.limits.speed_min_mps)
        self.assertLessEqual(control[0], self.limits.speed_max_mps)

    def test_projected_gradient_solver_returns_bounded_control(self):
        config = dict(
            self.config,
            mpc_solver="projected_gradient",
            mpc_use_analytic_gradient=True,
            mpc_deadline_s=0.0,
        )
        controller = RealVehicleMPC(self.parameters, self.limits, 0.01, config)
        control, info = controller.plan([0.0, 0.0, -2.0, 0.0], self.reference)
        self.assertEqual(info["solver"], "projected_gradient")
        self.assertGreaterEqual(control[0], self.limits.speed_min_mps)
        self.assertLessEqual(control[0], self.limits.speed_max_mps)

    def test_mppi_returns_bounded_two_input_command(self):
        controller = RealVehicleMPPI(self.parameters, self.limits, 0.01, self.config)
        control, info = controller.plan([0.0, 0.0, -2.0, 0.0], self.reference)
        self.assertEqual(control.shape, (2,))
        self.assertTrue(np.isfinite(info["cost"]))
        self.assertLessEqual(abs(control[1]), self.limits.steer_rate_max_rad_s)

    def test_pure_pursuit_uses_same_two_input_contract(self):
        controller = RealVehiclePurePursuit(
            self.parameters,
            self.limits,
            0.01,
            dict(self.config, pure_pursuit_lookahead_m=0.01),
        )
        control, info = controller.plan(
            [0.0, 0.0, -2.0, 0.0],
            self.reference,
        )
        self.assertEqual(control.shape, (2,))
        self.assertGreater(control[0], 0.0)
        self.assertLessEqual(abs(control[1]), self.limits.steer_rate_max_rad_s)
        self.assertEqual(info["controller"], "pure_pursuit")


if __name__ == "__main__":
    unittest.main()
