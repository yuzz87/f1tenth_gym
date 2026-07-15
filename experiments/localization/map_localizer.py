from pathlib import Path

import numpy as np

from f110_gym.envs.laser_models import ScanSimulator2D

from .base_localizer import BaseLocalizer


class BruteForceMapLocalizer(BaseLocalizer):
    """Local search map-based localizer using LiDAR scan matching."""

    def __init__(self, conf, localizer_conf, scan_angles):
        self.conf = conf
        self.localizer_conf = localizer_conf
        self.full_scan_angles = np.asarray(scan_angles, dtype=float)
        self.scan_beams = int(localizer_conf.get("scan_beams", 180))
        self.xy_search_radius = float(localizer_conf.get("xy_search_radius", 0.06))
        self.theta_search_radius = float(localizer_conf.get("theta_search_radius", 0.12))
        self.xy_candidates = int(localizer_conf.get("xy_candidates", 5))
        self.theta_candidates = int(localizer_conf.get("theta_candidates", 7))
        self.score_penalty = float(localizer_conf.get("score_penalty", 0.3))
        self.motion_prior_weight = float(localizer_conf.get("motion_prior_weight", 2.0))
        self.heading_prior_weight = float(localizer_conf.get("heading_prior_weight", 0.6))
        self.scan_noise_std = float(localizer_conf.get("scan_noise_std", 0.0))
        self.initialization_noise = np.asarray(
            localizer_conf.get("initialization_noise", [0.0, 0.0, 0.0]),
            dtype=float,
        )
        self.dt = float(conf.timestep)
        self.wheelbase = float(conf.car_params["lf"] + conf.car_params["lr"])

        self.observed_indices = np.linspace(
            0, self.full_scan_angles.shape[0] - 1, num=self.scan_beams, dtype=int
        )
        self.observed_angles = self.full_scan_angles[self.observed_indices]
        fov = float(self.full_scan_angles[-1] - self.full_scan_angles[0])
        self.scan_simulator = ScanSimulator2D(self.scan_beams, fov)
        self.scan_simulator.set_map(str(Path(conf.map_path).with_suffix(".yaml")), conf.map_ext)

        self.estimated_pose = None
        self.last_score = None
        self.last_scan_error = None
        self.last_motion_error = None
        self.predicted_pose = None
        self.scan_rng = np.random.default_rng(seed=1234)

    def initialize(self, initial_pose):
        initial_pose = np.asarray(initial_pose, dtype=float)
        self.estimated_pose = initial_pose + self.initialization_noise
        self.estimated_pose[2] = self._normalize_angle(self.estimated_pose[2])
        self.predicted_pose = self.estimated_pose.copy()
        self.last_score = None
        self.last_scan_error = None
        self.last_motion_error = None
        return self.estimated_pose.copy()

    def update(self, obs, control=None):
        if self.estimated_pose is None:
            raise RuntimeError("Localizer must be initialized before update().")

        self.predicted_pose = self._predict_pose(obs, control)
        observed_scan = np.asarray(obs["scans"][0], dtype=float)[self.observed_indices]
        best_pose = self.predicted_pose.copy()
        best_score = np.inf
        best_scan_error = np.inf
        best_motion_error = np.inf

        x_offsets = np.linspace(-self.xy_search_radius, self.xy_search_radius, self.xy_candidates)
        y_offsets = np.linspace(-self.xy_search_radius, self.xy_search_radius, self.xy_candidates)
        theta_offsets = np.linspace(
            -self.theta_search_radius, self.theta_search_radius, self.theta_candidates
        )

        for dx in x_offsets:
            for dy in y_offsets:
                for dtheta in theta_offsets:
                    candidate_pose = self.predicted_pose + np.array([dx, dy, dtheta], dtype=float)
                    candidate_pose[2] = self._normalize_angle(candidate_pose[2])
                    simulated_scan = self.scan_simulator.scan(
                        candidate_pose, None, std_dev=self.scan_noise_std
                    )
                    scan_error = float(np.mean((simulated_scan - observed_scan) ** 2))
                    motion_dx = candidate_pose[0] - self.predicted_pose[0]
                    motion_dy = candidate_pose[1] - self.predicted_pose[1]
                    motion_dtheta = self._normalize_angle(candidate_pose[2] - self.predicted_pose[2])
                    motion_error = motion_dx * motion_dx + motion_dy * motion_dy
                    motion_penalty = self.motion_prior_weight * motion_error
                    heading_penalty = self.heading_prior_weight * motion_dtheta * motion_dtheta
                    offset_penalty = self.score_penalty * (dx * dx + dy * dy + 0.25 * dtheta * dtheta)
                    score = scan_error + motion_penalty + heading_penalty + offset_penalty
                    if score < best_score:
                        best_score = score
                        best_scan_error = scan_error
                        best_motion_error = motion_error + motion_dtheta * motion_dtheta
                        best_pose = candidate_pose

        self.estimated_pose = best_pose
        self.last_score = float(best_score)
        self.last_scan_error = float(best_scan_error)
        self.last_motion_error = float(best_motion_error)
        return self.estimated_pose.copy()

    def debug_info(self):
        return {
            "localization_score": self.last_score if self.last_score is not None else 0.0,
            "scan_error": self.last_scan_error if self.last_scan_error is not None else 0.0,
            "motion_error": self.last_motion_error if self.last_motion_error is not None else 0.0,
            "scan_beams": self.scan_beams,
        }

    def _predict_pose(self, obs, control):
        del control
        x, y, theta = self.estimated_pose
        speed = float(obs["linear_vels_x"][0])
        yaw_rate = float(obs["ang_vels_z"][0])

        if abs(yaw_rate) > 1e-6:
            theta_next = self._normalize_angle(theta + yaw_rate * self.dt)
        else:
            theta_next = theta

        x_next = x + speed * np.cos(theta) * self.dt
        y_next = y + speed * np.sin(theta) * self.dt
        return np.array([x_next, y_next, theta_next], dtype=float)

    @staticmethod
    def _normalize_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi
