"""Virtual 2D LiDAR that ray-casts against a saved ROS occupancy map."""

from collections import deque

import numpy as np


class OccupancyGridLidar:
    """Generate LaserScan-compatible dictionaries from a ROS PGM/YAML map."""

    def __init__(self, occupancy_map, config=None, seed=7):
        self.map = occupancy_map
        self.config = dict(config or {})
        self.num_beams = int(self.config.get("num_beams", 72))
        self.fov_rad = float(self.config.get("fov_rad", 2.0 * np.pi))
        self.range_min_m = float(self.config.get("range_min_m", 0.15))
        self.range_max_m = float(self.config.get("range_max_m", 12.0))
        self.angle_min = -0.5 * self.fov_rad
        self.angle_increment = self.fov_rad / max(self.num_beams - 1, 1)
        self.angles = self.angle_min + self.angle_increment * np.arange(
            self.num_beams,
            dtype=float,
        )
        self.x_offset_m = float(self.config.get("x_offset_m", 0.25))
        self.y_offset_m = float(self.config.get("y_offset_m", 0.0))
        self.yaw_offset_rad = float(self.config.get("yaw_offset_rad", 0.0))
        self.noise_std_m = max(float(self.config.get("noise_std_m", 0.0)), 0.0)
        self.distance_noise_slope = max(
            float(self.config.get("distance_noise_slope", 0.0)),
            0.0,
        )
        self.dropout_probability = float(np.clip(
            self.config.get("dropout_probability", 0.0),
            0.0,
            1.0,
        ))
        self.delay_s = max(float(self.config.get("delay_s", 0.0)), 0.0)
        self.ray_step_m = max(
            float(self.config.get(
                "ray_step_m",
                0.5 * float(self.map.resolution_m),
            )),
            0.25 * float(self.map.resolution_m),
        )
        self.unknown_is_occupied = bool(
            self.config.get("unknown_is_occupied", True)
        )
        self.rng = np.random.default_rng(int(seed))
        self._pending = deque()
        self._last = None
        self.last_valid_ratio = 1.0

    def _ideal_scan(self, pose):
        yaw, x_m, y_m = map(float, np.asarray(pose)[:3])
        cosine = np.cos(yaw)
        sine = np.sin(yaw)
        laser_x = x_m + cosine * self.x_offset_m - sine * self.y_offset_m
        laser_y = y_m + sine * self.x_offset_m + cosine * self.y_offset_m
        directions = yaw + self.yaw_offset_rad + self.angles
        ray_cosine = np.cos(directions)
        ray_sine = np.sin(directions)
        ranges = np.full(self.num_beams, self.range_max_m, dtype=float)
        active = np.ones(self.num_beams, dtype=bool)
        distances = np.arange(
            self.range_min_m,
            self.range_max_m + 0.5 * self.ray_step_m,
            self.ray_step_m,
            dtype=float,
        )
        for distance in distances:
            if not np.any(active):
                break
            indices = np.flatnonzero(active)
            xs = laser_x + distance * ray_cosine[indices]
            ys = laser_y + distance * ray_sine[indices]
            rows, columns, inside = self.map.world_to_cells(xs, ys)
            hit = np.zeros(indices.size, dtype=bool)
            if np.any(~inside):
                active[indices[~inside]] = False
            if np.any(inside):
                occupied = self.map.occupied[
                    rows[inside],
                    columns[inside],
                ]
                if self.unknown_is_occupied:
                    occupied = occupied | self.map.unknown[
                        rows[inside],
                        columns[inside],
                    ]
                hit[inside] = occupied
            if np.any(hit):
                hit_indices = indices[hit]
                ranges[hit_indices] = min(distance, self.range_max_m)
                active[hit_indices] = False
        return ranges

    def scan(self, pose, timestamp_s=0.0):
        ranges = self._ideal_scan(pose)
        noise_std = self.noise_std_m + self.distance_noise_slope * ranges
        if np.any(noise_std > 0.0):
            ranges += self.rng.normal(0.0, noise_std, self.num_beams)
        invalid = self.rng.random(self.num_beams) < self.dropout_probability
        ranges[invalid] = self.range_max_m
        ranges = np.clip(ranges, self.range_min_m, self.range_max_m)
        self.last_valid_ratio = float(np.mean(~invalid))
        scan = {
            "header": {
                "stamp_s": float(timestamp_s),
                "frame_id": self.config.get("frame_id", "laser"),
            },
            "angle_min": self.angle_min,
            "angle_max": (
                self.angle_min
                + self.angle_increment * (self.num_beams - 1)
            ),
            "angle_increment": self.angle_increment,
            "range_min": self.range_min_m,
            "range_max": self.range_max_m,
            "ranges": ranges,
            "valid_ratio": self.last_valid_ratio,
        }
        self._pending.append((float(timestamp_s) + self.delay_s, scan))
        while self._pending and self._pending[0][0] <= float(timestamp_s):
            _, self._last = self._pending.popleft()
        return self._last if self._last is not None else scan
