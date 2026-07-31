"""Small ROS LaserScan-compatible virtual LiDAR and rectangular-map localizer."""

from collections import deque

import numpy as np

from ..models.bicycle_model import normalize_angle


DEFAULT_COMPLEX_OBSTACLES = (
    (-1.2, -0.4, 1.0, 2.4),
    (1.0, 2.0, -2.2, -1.0),
    (-3.4, -2.4, -0.4, 0.5),
)


class VirtualLidar:
    """Ray-cast a 2D scan against an axis-aligned rectangular room."""

    def __init__(self, config, seed=7):
        self.config = dict(config)
        self.num_beams = int(self.config.get("num_beams", 72))
        self.fov_rad = float(self.config.get("fov_rad", 2.0 * np.pi))
        self.range_min_m = float(self.config.get("range_min_m", 0.15))
        self.range_max_m = float(self.config.get("range_max_m", 12.0))
        self.angle_min = -self.fov_rad / 2.0
        self.angle_increment = self.fov_rad / max(self.num_beams - 1, 1)
        self.angles = np.linspace(
            self.angle_min,
            self.angle_min + self.angle_increment * (self.num_beams - 1),
            self.num_beams,
        )
        self.x_min = float(self.config.get("map_x_min_m", -5.0))
        self.x_max = float(self.config.get("map_x_max_m", 5.0))
        self.y_min = float(self.config.get("map_y_min_m", -5.0))
        self.y_max = float(self.config.get("map_y_max_m", 5.0))
        self.obstacles = self._parse_obstacles(self.config.get("obstacles", ()))
        self.noise_std_m = max(float(self.config.get("noise_std_m", 0.0)), 0.0)
        self.distance_noise_slope = max(
            float(self.config.get("distance_noise_slope", 0.0)),
            0.0,
        )
        self.dropout_probability = float(np.clip(
            self.config.get("dropout_probability", 0.0), 0.0, 1.0
        ))
        self.burst_start_probability = float(np.clip(
            self.config.get("burst_start_probability", 0.0), 0.0, 1.0
        ))
        self.burst_length_scans = max(
            int(self.config.get("burst_length_scans", 1)),
            1,
        )
        self.delay_jitter_s = max(float(self.config.get("delay_jitter_s", 0.0)), 0.0)
        self.timestamp_jitter_s = max(
            float(self.config.get("timestamp_jitter_s", 0.0)),
            0.0,
        )
        self.x_offset_m = float(self.config.get("x_offset_m", 0.0))
        self.y_offset_m = float(self.config.get("y_offset_m", 0.0))
        self.yaw_offset_rad = float(self.config.get("yaw_offset_rad", 0.0))
        self.rng = np.random.default_rng(int(seed))
        self._pending = deque()
        self._last = None
        self._burst_remaining = 0
        self.last_valid_ratio = 1.0

    @staticmethod
    def _parse_obstacles(obstacles):
        parsed = []
        for obstacle in obstacles or ():
            if isinstance(obstacle, dict):
                values = (
                    obstacle["x_min_m"],
                    obstacle["x_max_m"],
                    obstacle["y_min_m"],
                    obstacle["y_max_m"],
                )
            else:
                if len(obstacle) != 4:
                    raise ValueError("LiDAR obstacles must contain x_min, x_max, y_min, y_max")
                values = obstacle
            x_min, x_max, y_min, y_max = [float(value) for value in values]
            if x_min >= x_max or y_min >= y_max:
                raise ValueError("LiDAR obstacle bounds must be increasing")
            parsed.append((x_min, x_max, y_min, y_max))
        return tuple(parsed)

    def _ideal_scan(self, pose):
        phi, x, y = map(float, np.asarray(pose)[:3])
        x, y = (
            x + np.cos(phi) * self.x_offset_m - np.sin(phi) * self.y_offset_m,
            y + np.sin(phi) * self.x_offset_m + np.cos(phi) * self.y_offset_m,
        )
        phi += self.yaw_offset_rad
        world_angles = phi + self.angles
        dx = np.cos(world_angles)
        dy = np.sin(world_angles)
        candidates = []
        with np.errstate(divide="ignore", invalid="ignore"):
            for wall, denominator, numerator in (
                ("x_min", dx, self.x_min - x),
                ("x_max", dx, self.x_max - x),
                ("y_min", dy, self.y_min - y),
                ("y_max", dy, self.y_max - y),
            ):
                values = numerator / denominator
                if wall.startswith("x"):
                    cross = y + values * dy
                    valid = (values > 0.0) & (cross >= self.y_min) & (cross <= self.y_max)
                else:
                    cross = x + values * dx
                    valid = (values > 0.0) & (cross >= self.x_min) & (cross <= self.x_max)
                candidates.append(np.where(valid, values, np.inf))
        ranges = np.min(np.vstack(candidates), axis=0)
        for obstacle_x_min, obstacle_x_max, obstacle_y_min, obstacle_y_max in self.obstacles:
            for denominator, numerator, cross_min, cross_max, is_x_wall in (
                (dx, obstacle_x_min - x, obstacle_y_min, obstacle_y_max, True),
                (dx, obstacle_x_max - x, obstacle_y_min, obstacle_y_max, True),
                (dy, obstacle_y_min - y, obstacle_x_min, obstacle_x_max, False),
                (dy, obstacle_y_max - y, obstacle_x_min, obstacle_x_max, False),
            ):
                values = numerator / denominator
                cross = (y + values * dy) if is_x_wall else (x + values * dx)
                valid = (values > 0.0) & (cross >= cross_min) & (cross <= cross_max)
                ranges = np.minimum(ranges, np.where(valid, values, np.inf))
        return np.clip(ranges, self.range_min_m, self.range_max_m)

    def scan(self, pose, timestamp_s=0.0):
        ranges = self._ideal_scan(pose)
        noise_std = self.noise_std_m + self.distance_noise_slope * ranges
        if np.any(noise_std > 0.0):
            ranges = ranges + self.rng.normal(0.0, noise_std, self.num_beams)
        invalid = self.rng.random(self.num_beams) < self.dropout_probability
        if self._burst_remaining <= 0 and self.rng.random() < self.burst_start_probability:
            self._burst_remaining = self.burst_length_scans
        burst_dropped = self._burst_remaining > 0
        if burst_dropped:
            invalid[:] = True
            self._burst_remaining -= 1
        ranges[invalid] = self.range_max_m
        ranges = np.clip(ranges, self.range_min_m, self.range_max_m)
        self.last_valid_ratio = float(np.mean(~invalid))
        stamp_s = float(timestamp_s)
        if self.timestamp_jitter_s > 0.0:
            stamp_s += float(self.rng.normal(0.0, self.timestamp_jitter_s))
        scan = {
            "header": {"stamp_s": stamp_s, "frame_id": self.config.get("frame_id", "laser")},
            "angle_min": self.angle_min,
            "angle_max": self.angle_min + self.angle_increment * (self.num_beams - 1),
            "angle_increment": self.angle_increment,
            "range_min": self.range_min_m,
            "range_max": self.range_max_m,
            "ranges": ranges,
            "valid_ratio": self.last_valid_ratio,
            "burst_dropped": burst_dropped,
        }
        delay_s = max(float(self.config.get("delay_s", 0.0)), 0.0)
        if self.delay_jitter_s > 0.0:
            delay_s = max(delay_s + float(self.rng.normal(0.0, self.delay_jitter_s)), 0.0)
        self._pending.append((float(timestamp_s) + delay_s, scan))
        while self._pending and self._pending[0][0] <= float(timestamp_s):
            _, self._last = self._pending.popleft()
        return self._last if self._last is not None else scan


class GridSearchLocalizer:
    """Estimate [phi, x, y] by comparing a scan against the known room."""

    def __init__(self, lidar: VirtualLidar, xy_step_m=0.10, theta_step_rad=0.05,
                 search_xy_m=0.30, search_theta_rad=0.15, minimum_valid_beams=4,
                 maximum_match_score=float("inf")):
        self.lidar = lidar
        self.xy_step_m = float(xy_step_m)
        self.theta_step_rad = float(theta_step_rad)
        self.search_xy_m = float(search_xy_m)
        self.search_theta_rad = float(search_theta_rad)
        self.minimum_valid_beams = int(minimum_valid_beams)
        self.maximum_match_score = float(maximum_match_score)
        self.last_success = False

    def estimate(self, scan, predicted_pose):
        predicted_pose = np.asarray(predicted_pose, dtype=float)
        measured = np.asarray(scan["ranges"], dtype=float)
        offsets_xy = np.arange(
            -self.search_xy_m,
            self.search_xy_m + 0.5 * self.xy_step_m,
            self.xy_step_m,
        )
        offsets_theta = np.arange(
            -self.search_theta_rad,
            self.search_theta_rad + 0.5 * self.theta_step_rad,
            self.theta_step_rad,
        )
        valid = np.isfinite(measured) & (measured < self.lidar.range_max_m - 1e-9)
        if np.count_nonzero(valid) < self.minimum_valid_beams:
            self.last_success = False
            return predicted_pose[:3].copy(), float("inf")
        best_pose = predicted_pose[:3].copy()
        best_score = float("inf")
        for dtheta in offsets_theta:
            for dx in offsets_xy:
                for dy in offsets_xy:
                    candidate = predicted_pose[:3].copy()
                    candidate[0] = float(normalize_angle(candidate[0] + dtheta))
                    candidate[1] += dx
                    candidate[2] += dy
                    expected = self.lidar._ideal_scan(candidate)
                    residual = np.minimum(
                        np.abs(expected[valid] - measured[valid]),
                        2.0,
                    )
                    score = float(np.mean(residual**2))
                    if score < best_score:
                        best_score = score
                        best_pose = candidate
        self.last_success = bool(best_score <= self.maximum_match_score)
        if not self.last_success:
            return predicted_pose[:3].copy(), best_score
        return best_pose, best_score
