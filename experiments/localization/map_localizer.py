from pathlib import Path
from collections import deque

import numpy as np

from f110_gym.envs.laser_models import ScanSimulator2D
from f110_gym.envs.lidar_config import (
    get_lidar_extrinsics,
    resolve_lidar_config,
    transform_pose_to_lidar,
)

from .base_localizer import BaseLocalizer


class BruteForceMapLocalizer(BaseLocalizer):
    """Local search map-based localizer using LiDAR scan matching."""

    def __init__(self, conf, localizer_conf, scan_angles, lidar_config=None):
        self.conf = conf
        self.localizer_conf = localizer_conf
        self.full_scan_angles = np.asarray(scan_angles, dtype=float)
        self.lidar_config = resolve_lidar_config(lidar_config)
        extrinsic_overrides = {
            key: localizer_conf[key]
            for key in ("x_offset", "y_offset", "z_offset", "yaw_offset")
            if key in localizer_conf
        }
        self.lidar_extrinsics = get_lidar_extrinsics(
            self.lidar_config,
            legacy_lidar_dist=getattr(conf, "lidar_dist", 0.0),
            overrides=extrinsic_overrides,
        )
        self.scan_beams = int(localizer_conf.get("scan_beams", 180))
        self.xy_search_radius = float(localizer_conf.get("xy_search_radius", 0.06))
        self.theta_search_radius = float(localizer_conf.get("theta_search_radius", 0.12))
        self.xy_candidates = int(localizer_conf.get("xy_candidates", 5))
        self.theta_candidates = int(localizer_conf.get("theta_candidates", 7))
        self.score_penalty = float(localizer_conf.get("score_penalty", 0.3))
        self.motion_prior_weight = float(localizer_conf.get("motion_prior_weight", 2.0))
        self.heading_prior_weight = float(localizer_conf.get("heading_prior_weight", 0.6))
        self.scan_noise_std = float(localizer_conf.get("scan_noise_std", 0.0))
        self.ignore_invalid_beams = bool(
            localizer_conf.get("ignore_invalid_beams", True)
        )
        self.min_valid_beams = int(localizer_conf.get("min_valid_beams", 20))
        self.hold_pose_on_invalid_scan = bool(
            localizer_conf.get("hold_pose_on_invalid_scan", True)
        )
        self.preserve_scan_angles = bool(
            localizer_conf.get("preserve_scan_angles", False)
        )
        self.scan_time_compensation = bool(
            localizer_conf.get("scan_time_compensation", False)
        )
        self.scan_motion_history_compensation = bool(
            localizer_conf.get(
                "scan_motion_history_compensation",
                self.scan_time_compensation,
            )
        )
        self.motion_history_seconds = float(
            localizer_conf.get("motion_history_seconds", 1.0)
        )
        self.range_min = float(
            localizer_conf.get("range_min", self.lidar_config["range_min"])
        )
        self.range_max = float(
            localizer_conf.get("range_max", self.lidar_config["range_max"])
        )
        self.initialization_noise = np.asarray(
            localizer_conf.get("initialization_noise", [0.0, 0.0, 0.0]),
            dtype=float,
        )
        self.dt = float(conf.timestep)
        self.wheelbase = float(conf.car_params["lf"] + conf.car_params["lr"])

        if self.full_scan_angles.ndim != 1 or self.full_scan_angles.size < 2:
            raise ValueError("scan_angles must contain at least two angles")
        if self.scan_beams < 2 or self.scan_beams > self.full_scan_angles.size:
            raise ValueError("localizer scan_beams must be between 2 and the scan length")
        if self.xy_candidates < 1 or self.theta_candidates < 1:
            raise ValueError("localizer candidate counts must be at least 1")

        self.observed_indices = np.linspace(
            0, self.full_scan_angles.shape[0] - 1, num=self.scan_beams, dtype=int
        )
        self.observed_angles = self.full_scan_angles[self.observed_indices]
        fov = float(self.full_scan_angles[-1] - self.full_scan_angles[0])
        simulation_beams = (
            self.full_scan_angles.size
            if self.preserve_scan_angles
            else self.scan_beams
        )
        self.scan_simulator = ScanSimulator2D(
            simulation_beams,
            fov,
            max_range=self.range_max,
            range_min=self.range_min,
            angle_min=float(self.full_scan_angles[0]),
        )
        self.scan_simulator.set_map(str(Path(conf.map_path).with_suffix(".yaml")), conf.map_ext)

        # 毎回作り直さず、探索候補と固定ペナルティを事前計算する。
        x_offsets = np.linspace(
            -self.xy_search_radius, self.xy_search_radius, self.xy_candidates
        )
        y_offsets = np.linspace(
            -self.xy_search_radius, self.xy_search_radius, self.xy_candidates
        )
        theta_offsets = np.linspace(
            -self.theta_search_radius,
            self.theta_search_radius,
            self.theta_candidates,
        )
        grid_x, grid_y, grid_theta = np.meshgrid(
            x_offsets, y_offsets, theta_offsets, indexing="ij"
        )
        self.candidate_offsets = np.column_stack(
            (grid_x.ravel(), grid_y.ravel(), grid_theta.ravel())
        )
        dx = self.candidate_offsets[:, 0]
        dy = self.candidate_offsets[:, 1]
        dtheta = self.candidate_offsets[:, 2]
        motion_error = dx * dx + dy * dy
        self.candidate_motion_errors = motion_error + dtheta * dtheta
        self.candidate_prior_penalties = (
            self.motion_prior_weight * motion_error
            + self.heading_prior_weight * dtheta * dtheta
            + self.score_penalty * (motion_error + 0.25 * dtheta * dtheta)
        )

        self.estimated_pose = None
        self.last_score = None
        self.last_scan_error = None
        self.last_motion_error = None
        self.predicted_pose = None
        self.last_scan_age = 0.0
        self.last_valid_beam_count = self.scan_beams
        self.last_scan_rejected = False
        self.scan_rng = np.random.default_rng(seed=1234)
        self._localizer_time = 0.0
        self._motion_history = deque()

    def initialize(self, initial_pose):
        initial_pose = np.asarray(initial_pose, dtype=float)
        self.estimated_pose = initial_pose + self.initialization_noise
        self.estimated_pose[2] = self._normalize_angle(self.estimated_pose[2])
        self.predicted_pose = self.estimated_pose.copy()
        self._localizer_time = 0.0
        self._motion_history.clear()
        self.last_score = None
        self.last_scan_error = None
        self.last_motion_error = None
        return self.estimated_pose.copy()

    def update(self, obs, control=None):
        if self.estimated_pose is None:
            raise RuntimeError("Localizer must be initialized before update().")

        self.predicted_pose = self._predict_pose(obs, control)
        full_observed_scan = np.asarray(obs["scans"][0], dtype=float)
        if full_observed_scan.shape != self.full_scan_angles.shape:
            raise ValueError(
                "LiDAR scan length does not match the configured scan angles: "
                f"{full_observed_scan.size} != {self.full_scan_angles.size}"
            )
        observed_scan = full_observed_scan[self.observed_indices]
        full_valid_mask = obs.get("scan_valid")
        if full_valid_mask is None:
            observed_valid_mask = np.ones(self.scan_beams, dtype=bool)
        else:
            full_valid_mask = np.asarray(full_valid_mask[0], dtype=bool)
            if full_valid_mask.shape != self.full_scan_angles.shape:
                raise ValueError("scan_valid length does not match scan length")
            observed_valid_mask = full_valid_mask[self.observed_indices]
        self.last_valid_beam_count = int(np.sum(observed_valid_mask))
        self.last_scan_rejected = False
        if (
            self.ignore_invalid_beams
            and self.last_valid_beam_count < self.min_valid_beams
        ):
            self.last_scan_rejected = True
            if self.hold_pose_on_invalid_scan:
                self.estimated_pose = self.predicted_pose.copy()
                return self.estimated_pose.copy()
        best_pose = self.predicted_pose.copy()
        best_score = np.inf
        best_scan_error = np.inf
        best_motion_error = np.inf
        scan_age = self._get_scan_age(obs)
        scan_speed = float(obs["linear_vels_x"][0])
        scan_yaw_rate = float(obs["ang_vels_z"][0])

        for index, offset in enumerate(self.candidate_offsets):
            candidate_pose = self.predicted_pose + offset
            candidate_pose[2] = self._normalize_angle(candidate_pose[2])
            scan_pose = candidate_pose
            if self.scan_time_compensation and scan_age > 0.0:
                # 古いスキャンを取得時刻の姿勢へ戻してから比較する。
                if self.scan_motion_history_compensation:
                    scan_pose = self._rewind_pose(
                        candidate_pose,
                        scan_age,
                        scan_speed,
                        scan_yaw_rate,
                    )
                else:
                    scan_pose = self._propagate_pose(
                        candidate_pose,
                        -scan_age,
                        scan_speed,
                        scan_yaw_rate,
                    )
            simulated_scan = self.scan_simulator.scan(
                transform_pose_to_lidar(scan_pose, self.lidar_extrinsics),
                None,
                std_dev=self.scan_noise_std,
            )
            if self.preserve_scan_angles:
                simulated_scan = simulated_scan[self.observed_indices]
            residual = simulated_scan - observed_scan
            if self.ignore_invalid_beams:
                scan_error = float(
                    np.mean(residual[observed_valid_mask] ** 2)
                )
            else:
                scan_error = float(np.mean(residual**2))
            score = scan_error + self.candidate_prior_penalties[index]
            if score < best_score:
                best_score = score
                best_scan_error = scan_error
                best_motion_error = self.candidate_motion_errors[index]
                best_pose = candidate_pose

        self.estimated_pose = best_pose
        self.last_score = float(best_score)
        self.last_scan_error = float(best_scan_error)
        self.last_motion_error = float(best_motion_error)
        return self.estimated_pose.copy()

    def predict(self, obs, control=None):
        """スキャン更新がない周期は運動モデルだけで姿勢を進める。"""

        if self.estimated_pose is None:
            raise RuntimeError("Localizer must be initialized before predict().")
        self.predicted_pose = self._predict_pose(obs, control)
        self.estimated_pose = self.predicted_pose.copy()
        return self.estimated_pose.copy()

    def debug_info(self):
        return {
            "localization_score": self.last_score if self.last_score is not None else 0.0,
            "scan_error": self.last_scan_error if self.last_scan_error is not None else 0.0,
            "motion_error": self.last_motion_error if self.last_motion_error is not None else 0.0,
            "scan_beams": self.scan_beams,
            "candidate_count": int(self.candidate_offsets.shape[0]),
            "preserve_scan_angles": self.preserve_scan_angles,
            "scan_time_compensation": self.scan_time_compensation,
            "scan_motion_history_compensation": self.scan_motion_history_compensation,
            "motion_history_seconds": self.motion_history_seconds,
            "scan_age_s": self.last_scan_age,
            "lidar_extrinsics": dict(self.lidar_extrinsics),
            "valid_beam_count": int(self.last_valid_beam_count),
            "valid_beam_ratio": float(self.last_valid_beam_count / self.scan_beams),
            "scan_rejected": self.last_scan_rejected,
        }

    def _get_scan_age(self, obs):
        scan_ages = obs.get("scan_ages")
        if scan_ages is None:
            self.last_scan_age = 0.0
        else:
            self.last_scan_age = max(0.0, float(scan_ages[0]))
        return self.last_scan_age

    def _propagate_pose(self, pose, duration, speed, yaw_rate):
        x, y, theta = np.asarray(pose, dtype=float)
        x += speed * np.cos(theta) * duration
        y += speed * np.sin(theta) * duration
        theta = self._normalize_angle(theta + yaw_rate * duration)
        return np.array([x, y, theta], dtype=float)

    def _predict_pose(self, obs, control):
        del control
        x, y, theta = self.estimated_pose
        speed = float(obs["linear_vels_x"][0])
        yaw_rate = float(obs["ang_vels_z"][0])
        predicted = self._propagate_pose(
            np.array([x, y, theta], dtype=float),
            self.dt,
            speed,
            yaw_rate,
        )
        self._localizer_time += self.dt
        self._motion_history.append(
            (self._localizer_time, speed, yaw_rate, self.dt)
        )
        history_start = self._localizer_time - self.motion_history_seconds
        while self._motion_history and self._motion_history[0][0] - self._motion_history[0][3] < history_start:
            self._motion_history.popleft()
        return predicted

    def _rewind_pose(self, pose, scan_age, fallback_speed, fallback_yaw_rate):
        """履歴速度で現在姿勢をスキャン取得時刻まで逆伝播する。"""

        target_time = max(0.0, self._localizer_time - float(scan_age))
        current_time = self._localizer_time
        rewound_pose = np.asarray(pose, dtype=float).copy()
        covered_until = current_time

        for end_time, speed, yaw_rate, duration in reversed(self._motion_history):
            start_time = end_time - duration
            overlap_start = max(target_time, start_time)
            overlap_end = min(current_time, end_time)
            if overlap_end > overlap_start:
                rewound_pose = self._propagate_pose(
                    rewound_pose,
                    -(overlap_end - overlap_start),
                    speed,
                    yaw_rate,
                )
                covered_until = overlap_start
            if target_time >= start_time:
                break

        # 履歴より古い遅延には、利用可能な速度で残りを補う。
        if target_time < covered_until:
            rewound_pose = self._propagate_pose(
                rewound_pose,
                -(covered_until - target_time),
                fallback_speed,
                fallback_yaw_rate,
            )
        return rewound_pose

    @staticmethod
    def _normalize_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi
