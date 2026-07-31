"""Pure Pursuit baseline using the real-vehicle controller interface."""

import time

import numpy as np

from ..models.bicycle_model import normalize_angle
from .common import ControllerBase


class RealVehiclePurePursuit(ControllerBase):
    """Convert a reference path into speed and steering-angle-rate commands."""

    def __init__(self, parameters, limits, timestep_s, config=None):
        super().__init__(parameters, limits, timestep_s, config)
        self.lookahead_m = max(
            float(self.config.get("pure_pursuit_lookahead_m", 0.30)),
            1e-3,
        )
        self.steer_response_s = max(
            float(self.config.get("pure_pursuit_steer_response_s", 0.15)),
            self.timestep_s,
        )

    def plan(self, state, reference):
        started = time.perf_counter()
        state = np.asarray(state, dtype=float)
        reference = self._prepare_reference(reference)
        distances = np.linalg.norm(reference[:, 1:3] - state[1:3], axis=1)
        candidates = np.flatnonzero(distances >= self.lookahead_m)
        target_index = int(candidates[0]) if candidates.size else len(reference) - 1
        target = reference[target_index]

        target_heading = np.arctan2(
            target[2] - state[2],
            target[1] - state[1],
        )
        alpha = float(normalize_angle(target_heading - state[0]))
        lookahead = max(float(distances[target_index]), self.lookahead_m)
        desired_steer = np.arctan2(
            2.0 * float(self.parameters.wheelbase_m) * np.sin(alpha),
            lookahead,
        )
        desired_steer = float(np.clip(
            desired_steer,
            self.limits.steer_min_rad,
            self.limits.steer_max_rad,
        ))
        steer_rate = float(np.clip(
            (desired_steer - state[3]) / self.steer_response_s,
            self.limits.steer_rate_min_rad_s,
            self.limits.steer_rate_max_rad_s,
        ))
        desired_controls = self._reference_controls(reference)
        speed = float(desired_controls[0, 0])
        sequence = np.tile([speed, steer_rate], (self.horizon, 1))
        predicted = self.rollout(state, sequence)
        self._shift(sequence)
        return sequence[0].copy(), {
            "controller": "pure_pursuit",
            "cost": float(self.cost_from_prediction(
                predicted,
                sequence,
                reference,
            )),
            "solve_time_s": time.perf_counter() - started,
            "success": True,
            "deadline_exceeded": False,
            "lookahead_index": target_index,
            "lookahead_distance_m": lookahead,
            "desired_steer_rad": desired_steer,
            "predicted_states": predicted,
        }
