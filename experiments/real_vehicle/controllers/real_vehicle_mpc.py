"""Constrained shooting MPC for the actual RC-car bicycle model."""

import time

import numpy as np
from scipy.optimize import minimize

from .common import ControllerBase


class _DeadlineExceeded(RuntimeError):
    pass


class RealVehicleMPC(ControllerBase):
    """Optimize both speed and steering-angle rate over a finite horizon."""

    def __init__(self, parameters, limits, timestep_s, config=None):
        super().__init__(parameters, limits, timestep_s, config)
        self.max_iterations = int(self.config.get("max_iterations", 30))
        self.max_function_evaluations = int(
            self.config.get("mpc_max_function_evaluations", 1000)
        )
        self.use_analytic_gradient = bool(
            self.config.get("mpc_use_analytic_gradient", True)
        )
        self.deadline_s = max(float(self.config.get("mpc_deadline_s", 0.0)), 0.0)
        self.solver = str(self.config.get("mpc_solver", "lbfgsb")).lower()
        if self.solver not in ("lbfgsb", "projected_gradient"):
            raise ValueError("mpc_solver must be lbfgsb or projected_gradient")
        self.gradient_step = max(
            float(self.config.get("mpc_gradient_step", 0.05)),
            1e-9,
        )
        self.control_blocks = max(
            1,
            min(int(self.config.get("mpc_control_blocks", self.horizon)), self.horizon),
        )
        self._block_indices = np.array_split(np.arange(self.horizon), self.control_blocks)

    def plan(self, state, reference):
        started = time.perf_counter()
        reference = self._prepare_reference(reference)
        desired = self._reference_controls(reference)
        initial = 0.7 * self.prev_sequence + 0.3 * desired
        initial = self._clip_sequence(initial)
        blocked_initial = self._compress_sequence(initial)
        lower = np.tile(
            [self.limits.speed_min_mps, self.limits.steer_rate_min_rad_s],
            self.control_blocks,
        )
        upper = np.tile(
            [self.limits.speed_max_mps, self.limits.steer_rate_max_rad_s],
            self.control_blocks,
        )
        self._solve_started = started
        self._best_cost = float("inf")
        self._best_blocked = blocked_initial.copy()
        deadline_exceeded = False
        try:
            if self.solver == "projected_gradient":
                blocked, success, iterations, evaluations = (
                    self._solve_projected_gradient(
                        blocked_initial,
                        state,
                        reference,
                        lower.reshape(self.control_blocks, 2),
                        upper.reshape(self.control_blocks, 2),
                    )
                )
            else:
                result = minimize(
                    self._objective_with_gradient
                    if self.use_analytic_gradient
                    else self._objective,
                    blocked_initial.reshape(-1),
                    args=(state, reference),
                    method="L-BFGS-B",
                    jac=self.use_analytic_gradient,
                    bounds=list(zip(lower, upper)),
                    options={
                        "maxiter": self.max_iterations,
                        "maxfun": self.max_function_evaluations,
                        "ftol": 1e-8,
                    },
                )
                if result.success or np.isfinite(result.fun):
                    blocked = result.x.reshape(self.control_blocks, 2)
                else:
                    blocked = self._best_blocked
                success = bool(result.success)
                iterations = int(getattr(result, "nit", 0))
                evaluations = int(getattr(result, "nfev", 0))
        except _DeadlineExceeded:
            blocked = self._best_blocked
            success = False
            iterations = 0
            evaluations = 0
            deadline_exceeded = True
        sequence = self._expand_sequence(blocked)
        sequence = self._clip_sequence(sequence)
        predicted = self.rollout(state, sequence)
        self._shift(sequence)
        return sequence[0].copy(), {
            "controller": "mpc",
            "cost": float(self.cost_from_prediction(predicted, sequence, reference)),
            "solve_time_s": time.perf_counter() - started,
            "success": success,
            "control_blocks": self.control_blocks,
            "optimizer_variables": 2 * self.control_blocks,
            "optimizer_iterations": iterations,
            "function_evaluations": evaluations,
            "analytic_gradient": self.use_analytic_gradient,
            "solver": self.solver,
            "deadline_exceeded": deadline_exceeded,
            "predicted_states": predicted,
        }

    def _objective(self, flattened, state, reference):
        blocked = np.asarray(flattened, dtype=float).reshape(self.control_blocks, 2)
        sequence = self._expand_sequence(blocked)
        cost = self.cost(state, sequence, reference)
        self._record_candidate(blocked, cost)
        return cost

    def _objective_with_gradient(self, flattened, state, reference):
        blocked = np.asarray(flattened, dtype=float).reshape(self.control_blocks, 2)
        sequence = self._expand_sequence(blocked)
        cost, full_gradient = self._cost_and_gradient(state, sequence, reference)
        blocked_gradient = np.vstack([
            np.sum(full_gradient[indices], axis=0)
            for indices in self._block_indices
        ])
        self._record_candidate(blocked, cost)
        return cost, blocked_gradient.reshape(-1)

    def _solve_projected_gradient(self, initial, state, reference, lower, upper):
        blocked = np.asarray(initial, dtype=float).copy()
        flattened = blocked.reshape(-1)
        cost, gradient = self._objective_with_gradient(
            flattened,
            state,
            reference,
        )
        evaluations = 1
        success = False
        completed_iterations = 0
        for iteration in range(self.max_iterations):
            completed_iterations = iteration + 1
            if np.linalg.norm(gradient, ord=np.inf) < 1e-6:
                success = True
                break
            step = self.gradient_step
            improved = False
            for _line_search in range(8):
                trial = np.clip(
                    blocked - step * gradient.reshape(self.control_blocks, 2),
                    lower,
                    upper,
                )
                trial_cost, trial_gradient = self._objective_with_gradient(
                    trial.reshape(-1),
                    state,
                    reference,
                )
                evaluations += 1
                if trial_cost < cost:
                    blocked = trial
                    cost = trial_cost
                    gradient = trial_gradient
                    improved = True
                    break
                step *= 0.5
                if evaluations >= self.max_function_evaluations:
                    break
            if not improved:
                success = True
                break
            if evaluations >= self.max_function_evaluations:
                break
        return blocked, success, completed_iterations, evaluations

    def _record_candidate(self, blocked, cost):
        if np.isfinite(cost) and cost < self._best_cost:
            self._best_cost = float(cost)
            self._best_blocked = np.asarray(blocked, dtype=float).copy()
        if self.deadline_s > 0.0:
            if time.perf_counter() - self._solve_started > self.deadline_s:
                raise _DeadlineExceeded

    def _cost_and_gradient(self, state, controls, reference):
        controls = np.asarray(controls, dtype=float)
        reference = self._prepare_reference(reference)
        desired = self._reference_controls(reference)
        current = np.asarray(state, dtype=float).copy()
        sensitivity = np.zeros((4, 2 * self.horizon), dtype=float)
        total = 0.0
        gradient = np.zeros(2 * self.horizon, dtype=float)
        previous = np.array([desired[0, 0], 0.0], dtype=float)

        for index, control in enumerate(controls):
            current, state_jacobian, control_jacobian = self._rk4_transition_jacobians(
                current,
                control,
            )
            sensitivity = state_jacobian @ sensitivity
            columns = slice(2 * index, 2 * index + 2)
            sensitivity[:, columns] += control_jacobian
            position_error = current[1:3] - reference[index, 1:3]
            heading_error = float(
                (current[0] - reference[index, 0] + np.pi) % (2.0 * np.pi) - np.pi
            )
            steer_error = float(
                (current[3] - reference[index, 3] + np.pi) % (2.0 * np.pi) - np.pi
            )
            input_delta = control - previous
            factor = self.terminal_weight if index == self.horizon - 1 else 1.0
            stage = (
                self.position_weight * float(position_error @ position_error)
                + self.heading_weight * heading_error**2
                + self.steer_weight * steer_error**2
                + self.speed_weight * (control[0] - desired[index, 0]) ** 2
                + self.input_rate_weight * float(input_delta @ input_delta)
            )
            total += factor * stage
            state_gradient = factor * np.array([
                2.0 * self.heading_weight * heading_error,
                2.0 * self.position_weight * position_error[0],
                2.0 * self.position_weight * position_error[1],
                2.0 * self.steer_weight * steer_error,
            ])
            gradient += state_gradient @ sensitivity
            direct = factor * np.array([
                2.0 * self.speed_weight * (control[0] - desired[index, 0]),
                0.0,
            ])
            direct += factor * 2.0 * self.input_rate_weight * input_delta
            gradient[columns] += direct
            if index > 0:
                previous_columns = slice(2 * (index - 1), 2 * index)
                gradient[previous_columns] -= (
                    factor * 2.0 * self.input_rate_weight * input_delta
                )
            previous = control
        return float(total), gradient.reshape(self.horizon, 2)

    def _rk4_transition_jacobians(self, state, control):
        dt = self.timestep_s
        identity = np.eye(4)
        k1, a1, b1 = self._dynamics_jacobians(state, control)
        z2 = state + 0.5 * dt * k1
        z2_state = identity + 0.5 * dt * a1
        z2_control = 0.5 * dt * b1
        k2, a2, b2 = self._dynamics_jacobians(z2, control)
        k2_state = a2 @ z2_state
        k2_control = a2 @ z2_control + b2
        z3 = state + 0.5 * dt * k2
        z3_state = identity + 0.5 * dt * k2_state
        z3_control = 0.5 * dt * k2_control
        k3, a3, b3 = self._dynamics_jacobians(z3, control)
        k3_state = a3 @ z3_state
        k3_control = a3 @ z3_control + b3
        z4 = state + dt * k3
        z4_state = identity + dt * k3_state
        z4_control = dt * k3_control
        k4, a4, b4 = self._dynamics_jacobians(z4, control)
        k4_state = a4 @ z4_state
        k4_control = a4 @ z4_control + b4
        next_state = state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        state_jacobian = identity + dt * (
            a1 + 2.0 * k2_state + 2.0 * k3_state + k4_state
        ) / 6.0
        control_jacobian = dt * (
            b1 + 2.0 * k2_control + 2.0 * k3_control + k4_control
        ) / 6.0
        next_state[0] = (next_state[0] + np.pi) % (2.0 * np.pi) - np.pi
        unclamped_steer = next_state[3]
        next_state[3] = np.clip(
            unclamped_steer,
            self.limits.steer_min_rad,
            self.limits.steer_max_rad,
        )
        if next_state[3] != unclamped_steer:
            state_jacobian[3] = 0.0
            control_jacobian[3] = 0.0
        return next_state, state_jacobian, control_jacobian

    def _dynamics_jacobians(self, state, control):
        phi, _x, _y, psi = np.asarray(state, dtype=float)
        velocity, steer_rate = np.asarray(control, dtype=float)
        wheelbase = float(self.parameters.wheelbase_m)
        derivative = np.array([
            velocity / wheelbase * np.tan(psi),
            velocity * np.cos(phi),
            velocity * np.sin(phi),
            steer_rate,
        ])
        state_jacobian = np.zeros((4, 4), dtype=float)
        state_jacobian[0, 3] = velocity / wheelbase / np.cos(psi) ** 2
        state_jacobian[1, 0] = -velocity * np.sin(phi)
        state_jacobian[2, 0] = velocity * np.cos(phi)
        control_jacobian = np.array([
            [np.tan(psi) / wheelbase, 0.0],
            [np.cos(phi), 0.0],
            [np.sin(phi), 0.0],
            [0.0, 1.0],
        ])
        return derivative, state_jacobian, control_jacobian

    def _compress_sequence(self, sequence):
        sequence = np.asarray(sequence, dtype=float)
        return np.vstack([
            np.mean(sequence[indices], axis=0) for indices in self._block_indices
        ])

    def _expand_sequence(self, blocked):
        blocked = np.asarray(blocked, dtype=float)
        sequence = np.empty((self.horizon, 2), dtype=float)
        for block, indices in zip(blocked, self._block_indices):
            sequence[indices] = block
        return sequence

    def _clip_sequence(self, sequence):
        sequence = np.asarray(sequence, dtype=float).copy()
        sequence[:, 0] = np.clip(
            sequence[:, 0], self.limits.speed_min_mps, self.limits.speed_max_mps
        )
        sequence[:, 1] = np.clip(
            sequence[:, 1], self.limits.steer_rate_min_rad_s, self.limits.steer_rate_max_rad_s
        )
        return sequence
