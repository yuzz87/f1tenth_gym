"""Known-map LiDAR scan matching for the real-vehicle ROS 2 path.

This module deliberately has no ROS imports so that map loading and scan
matching can be tested with the repository Python environment.
"""

from pathlib import Path
import math

import numpy as np
import yaml

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:  # pragma: no cover - reported when the node is created
    distance_transform_edt = None


def normalize_angle(angle_rad):
    """Wrap an angle to [-pi, pi)."""

    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def _read_pgm(path):
    """Read binary P5 or ASCII P2 PGM data without Pillow."""

    data = Path(path).read_bytes()
    index = 0

    def next_token():
        nonlocal index
        while index < len(data):
            if data[index] in b" \t\r\n":
                index += 1
                continue
            if data[index] == ord("#"):
                newline = data.find(b"\n", index)
                index = len(data) if newline < 0 else newline + 1
                continue
            break
        start = index
        while index < len(data) and data[index] not in b" \t\r\n#":
            index += 1
        if start == index:
            raise ValueError("PGM header is incomplete")
        return data[start:index].decode("ascii")

    magic = next_token()
    if magic not in ("P2", "P5"):
        raise ValueError(f"unsupported PGM format {magic!r}")
    width = int(next_token())
    height = int(next_token())
    max_value = int(next_token())
    if width < 1 or height < 1 or not 1 <= max_value <= 65535:
        raise ValueError("invalid PGM dimensions or maximum value")

    if magic == "P2":
        values = []
        while True:
            try:
                values.append(int(next_token()))
            except ValueError:
                break
        image = np.asarray(values, dtype=np.uint16)
    else:
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if max_value <= 255:
            image = np.frombuffer(data, dtype=np.uint8, offset=index)
        else:
            image = np.frombuffer(data, dtype=">u2", offset=index)

    expected = width * height
    if image.size < expected:
        raise ValueError(
            f"PGM contains {image.size} pixels; expected {expected}"
        )
    image = np.asarray(image[:expected], dtype=np.float64).reshape((height, width))
    if max_value != 255:
        image = image * (255.0 / float(max_value))
    return image


class OccupancyMap:
    """Load a ROS map YAML/PGM pair and provide map-coordinate utilities."""

    def __init__(self, yaml_path, occupied_threshold=None, free_threshold=None):
        self.yaml_path = Path(yaml_path).expanduser().resolve()
        metadata = yaml.safe_load(self.yaml_path.read_text(encoding="utf-8")) or {}
        image_value = metadata.get("image")
        if not image_value:
            raise ValueError(f"map YAML has no image field: {self.yaml_path}")
        image_path = Path(str(image_value)).expanduser()
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path
        self.image_path = image_path.resolve()
        self.resolution_m = float(metadata.get("resolution", 0.05))
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError("map resolution must be positive")
        origin = metadata.get("origin", [0.0, 0.0, 0.0])
        if len(origin) != 3:
            raise ValueError("map origin must contain x, y, yaw")
        self.origin_x_m = float(origin[0])
        self.origin_y_m = float(origin[1])
        self.origin_yaw_rad = float(origin[2])
        self.negate = bool(int(metadata.get("negate", 0)))
        self.occupied_threshold = float(
            metadata.get("occupied_thresh", 0.65)
            if occupied_threshold is None else occupied_threshold
        )
        self.free_threshold = float(
            metadata.get("free_thresh", 0.25)
            if free_threshold is None else free_threshold
        )
        if not 0.0 <= self.free_threshold <= self.occupied_threshold <= 1.0:
            raise ValueError("map free/occupied thresholds are invalid")

        gray = _read_pgm(self.image_path)
        probability = gray / 255.0 if self.negate else 1.0 - gray / 255.0
        self.height, self.width = gray.shape
        self.occupied = probability >= self.occupied_threshold
        self.free = probability <= self.free_threshold
        self.unknown = ~(self.occupied | self.free)

        # ROS OccupancyGrid data starts at the bottom-left. PGM rows start at
        # the top, so flip the image vertically when creating message data.
        self.occupancy_data = np.full(gray.shape, -1, dtype=np.int8)
        self.occupancy_data[self.free] = 0
        self.occupancy_data[self.occupied] = 100
        self.occupancy_data_bottom_up = np.flipud(self.occupancy_data)

        if distance_transform_edt is None:
            raise RuntimeError(
                "scipy is required for real-map LiDAR scan matching"
            )
        if np.any(self.occupied):
            self.distance_to_occupied_m = (
                distance_transform_edt(~self.occupied) * self.resolution_m
            )
        else:
            self.distance_to_occupied_m = np.full(
                gray.shape,
                np.inf,
                dtype=float,
            )

    @property
    def size_m(self):
        return self.width * self.resolution_m, self.height * self.resolution_m

    def world_to_cell(self, x_m, y_m):
        """Return (row, column, inside) for a map-frame point."""

        dx = float(x_m) - self.origin_x_m
        dy = float(y_m) - self.origin_y_m
        cosine = math.cos(self.origin_yaw_rad)
        sine = math.sin(self.origin_yaw_rad)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column = int(math.floor(local_x / self.resolution_m))
        grid_y = int(math.floor(local_y / self.resolution_m))
        inside = (
            0 <= column < self.width
            and 0 <= grid_y < self.height
        )
        row = self.height - 1 - grid_y
        return row, column, inside

    def world_to_cells(self, x_m, y_m):
        """Vectorized world-to-PGM row/column conversion."""

        x = np.asarray(x_m, dtype=float)
        y = np.asarray(y_m, dtype=float)
        dx = x - self.origin_x_m
        dy = y - self.origin_y_m
        cosine = math.cos(self.origin_yaw_rad)
        sine = math.sin(self.origin_yaw_rad)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column = np.floor(local_x / self.resolution_m).astype(np.int64)
        grid_y = np.floor(local_y / self.resolution_m).astype(np.int64)
        inside = (
            (column >= 0)
            & (column < self.width)
            & (grid_y >= 0)
            & (grid_y < self.height)
        )
        row = self.height - 1 - grid_y
        return row, column, inside

    def scan_score(
        self,
        pose,
        ranges,
        angle_min_rad,
        angle_increment_rad,
        range_min_m,
        range_max_m,
        lidar_x_m=0.25,
        lidar_y_m=0.0,
        lidar_yaw_rad=0.0,
        max_beams=90,
        max_error_m=0.40,
        unknown_penalty_m=0.25,
        outside_penalty_m=0.75,
    ):
        """Score measured scan endpoints against occupied map cells."""

        measured = np.asarray(ranges, dtype=float)
        if measured.ndim != 1 or measured.size == 0:
            return float("inf"), 0
        valid = (
            np.isfinite(measured)
            & (measured >= float(range_min_m))
            & (measured < float(range_max_m) - 1e-6)
        )
        indices = np.flatnonzero(valid)
        if indices.size == 0:
            return float("inf"), 0
        if max_beams > 0 and indices.size > int(max_beams):
            selection = np.linspace(
                0,
                indices.size - 1,
                int(max_beams),
            ).round().astype(np.int64)
            indices = indices[selection]

        yaw, x_m, y_m = [float(value) for value in np.asarray(pose)[:3]]
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        laser_x = x_m + cosine * float(lidar_x_m) - sine * float(lidar_y_m)
        laser_y = y_m + sine * float(lidar_x_m) + cosine * float(lidar_y_m)
        angles = (
            float(angle_min_rad)
            + float(angle_increment_rad) * indices
            + float(lidar_yaw_rad)
            + yaw
        )
        endpoints_x = laser_x + measured[indices] * np.cos(angles)
        endpoints_y = laser_y + measured[indices] * np.sin(angles)
        rows, columns, inside = self.world_to_cells(endpoints_x, endpoints_y)
        residual = np.full(
            indices.size,
            float(outside_penalty_m) ** 2,
            dtype=float,
        )
        inside_rows = rows[inside]
        inside_columns = columns[inside]
        distances = np.minimum(
            self.distance_to_occupied_m[inside_rows, inside_columns],
            float(max_error_m),
        )
        values = distances ** 2
        unknown = self.unknown[inside_rows, inside_columns]
        values = values + np.where(
            unknown,
            float(unknown_penalty_m) ** 2,
            0.0,
        )
        residual[inside] = values
        return float(np.mean(residual)), int(indices.size)


def _compose_pose(first, second):
    """Compose planar poses represented as [yaw, x, y]."""

    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    yaw = normalize_angle(first[0] + second[0])
    cosine = math.cos(first[0])
    sine = math.sin(first[0])
    x = first[1] + cosine * second[1] - sine * second[2]
    y = first[2] + sine * second[1] + cosine * second[2]
    return np.array([yaw, x, y], dtype=float)


def _inverse_pose(pose):
    """Invert a planar pose represented as [yaw, x, y]."""

    pose = np.asarray(pose, dtype=float)
    yaw = normalize_angle(-pose[0])
    cosine = math.cos(pose[0])
    sine = math.sin(pose[0])
    return np.array([
        yaw,
        -cosine * pose[1] - sine * pose[2],
        sine * pose[1] - cosine * pose[2],
    ], dtype=float)


def odom_delta(previous_odom, current_odom):
    """Return motion from the previous odom pose to the current pose."""

    return _compose_pose(_inverse_pose(previous_odom), current_odom)


class GridMapLocalizer:
    """Search for the map pose that best explains the latest LiDAR scan."""

    def __init__(
        self,
        occupancy_map,
        xy_step_m=0.15,
        theta_step_rad=0.10,
        search_xy_m=0.30,
        search_theta_rad=0.25,
        minimum_valid_beams=20,
        maximum_match_score_m2=0.25,
        max_scan_beams=90,
        max_error_m=0.40,
        unknown_penalty_m=0.25,
        outside_penalty_m=0.75,
        lidar_x_m=0.25,
        lidar_y_m=0.0,
        lidar_yaw_rad=0.0,
        position_prior_weight=0.20,
        yaw_prior_weight=0.05,
        minimum_scan_score_improvement_m2=0.001,
        minimum_objective_improvement_m2=0.001,
        maximum_position_correction_m=0.22,
        maximum_yaw_correction_rad=0.16,
        correction_confirmation_scans=2,
        correction_cooldown_scans=2,
        correction_consistency_position_m=0.01,
        correction_consistency_yaw_rad=0.01,
    ):
        self.map = occupancy_map
        self.xy_step_m = float(xy_step_m)
        self.theta_step_rad = float(theta_step_rad)
        self.search_xy_m = float(search_xy_m)
        self.search_theta_rad = float(search_theta_rad)
        self.minimum_valid_beams = int(minimum_valid_beams)
        self.maximum_match_score_m2 = float(maximum_match_score_m2)
        self.max_scan_beams = int(max_scan_beams)
        self.max_error_m = float(max_error_m)
        self.unknown_penalty_m = float(unknown_penalty_m)
        self.outside_penalty_m = float(outside_penalty_m)
        self.lidar_x_m = float(lidar_x_m)
        self.lidar_y_m = float(lidar_y_m)
        self.lidar_yaw_rad = float(lidar_yaw_rad)
        self.position_prior_weight = max(float(position_prior_weight), 0.0)
        self.yaw_prior_weight = max(float(yaw_prior_weight), 0.0)
        self.minimum_scan_score_improvement_m2 = max(
            float(minimum_scan_score_improvement_m2),
            0.0,
        )
        self.minimum_objective_improvement_m2 = max(
            float(minimum_objective_improvement_m2),
            0.0,
        )
        self.maximum_position_correction_m = max(
            float(maximum_position_correction_m),
            0.0,
        )
        self.maximum_yaw_correction_rad = max(
            float(maximum_yaw_correction_rad),
            0.0,
        )
        self.correction_confirmation_scans = max(
            int(correction_confirmation_scans),
            1,
        )
        self.correction_cooldown_scans = max(
            int(correction_cooldown_scans),
            0,
        )
        self.correction_consistency_position_m = max(
            float(correction_consistency_position_m),
            0.0,
        )
        self.correction_consistency_yaw_rad = max(
            float(correction_consistency_yaw_rad),
            0.0,
        )
        self.last_success = False
        self.last_valid_beams = 0
        self.last_score = float("inf")
        self.last_predicted_score = float("inf")
        self.last_objective_score = float("inf")
        self.last_prior_cost = 0.0
        self.last_scan_improvement_m2 = 0.0
        self.last_objective_improvement_m2 = 0.0
        self.last_position_correction_m = 0.0
        self.last_yaw_correction_rad = 0.0
        self.last_correction_dx_m = 0.0
        self.last_correction_dy_m = 0.0
        self.last_correction_dyaw_rad = 0.0
        self.last_proposed_correction_dx_m = 0.0
        self.last_proposed_correction_dy_m = 0.0
        self.last_proposed_correction_dyaw_rad = 0.0
        self.correction_event_id = 0
        self.correction_proposal_count = 0
        self.correction_pending_count = 0
        self.correction_rejection_count = 0
        self.pending_reset_count = 0
        self.last_confirmation_count = 0
        self.last_cooldown_scans_remaining = 0
        self.pending_correction_dx_m = 0.0
        self.pending_correction_dy_m = 0.0
        self.pending_correction_dyaw_rad = 0.0
        self._pending_correction = None
        self._pending_confirmation_count = 0
        self._cooldown_scans_remaining = 0
        self.last_selection_reason = "not_run"
        self.last_candidate_count = 0

    @staticmethod
    def _offsets(limit, step):
        """Return symmetric offsets that always include zero."""

        if limit <= 0.0:
            return np.array([0.0], dtype=float)
        step = max(float(step), 1e-6)
        limit = float(limit)
        positive = np.arange(step, limit + 1e-12, step, dtype=float)
        if positive.size == 0 or limit - positive[-1] > 1e-12:
            positive = np.append(positive, limit)
        return np.concatenate((-positive[::-1], [0.0], positive))

    def _scan_score(self, pose, measured, scan):
        return self.map.scan_score(
            pose,
            measured,
            scan["angle_min"],
            scan["angle_increment"],
            scan["range_min"],
            scan["range_max"],
            lidar_x_m=self.lidar_x_m,
            lidar_y_m=self.lidar_y_m,
            lidar_yaw_rad=self.lidar_yaw_rad,
            max_beams=self.max_scan_beams,
            max_error_m=self.max_error_m,
            unknown_penalty_m=self.unknown_penalty_m,
            outside_penalty_m=self.outside_penalty_m,
        )[0]

    def _clear_pending_correction(self, count_reset=False):
        if count_reset and self._pending_correction is not None:
            self.pending_reset_count += 1
        self._pending_correction = None
        self._pending_confirmation_count = 0
        self.pending_correction_dx_m = 0.0
        self.pending_correction_dy_m = 0.0
        self.pending_correction_dyaw_rad = 0.0

    def _pending_matches(self, dx, dy, dtheta):
        if self._pending_correction is None:
            return False
        pending_dx, pending_dy, pending_dtheta = self._pending_correction
        position_difference = math.hypot(
            float(dx) - pending_dx,
            float(dy) - pending_dy,
        )
        yaw_difference = abs(normalize_angle(
            float(dtheta) - pending_dtheta
        ))
        return (
            position_difference
            <= self.correction_consistency_position_m + 1e-12
            and yaw_difference
            <= self.correction_consistency_yaw_rad + 1e-12
        )

    def _remember_pending_correction(self, dx, dy, dtheta):
        self._pending_correction = (
            float(dx),
            float(dy),
            float(dtheta),
        )
        self._pending_confirmation_count = 1
        self.pending_correction_dx_m = float(dx)
        self.pending_correction_dy_m = float(dy)
        self.pending_correction_dyaw_rad = float(dtheta)

    def reset_correction_history(self, reset_counters=False):
        """Clear temporal correction state after an external pose reset."""

        self._clear_pending_correction()
        self._cooldown_scans_remaining = 0
        self.last_confirmation_count = 0
        self.last_cooldown_scans_remaining = 0
        if reset_counters:
            self.correction_event_id = 0
            self.correction_proposal_count = 0
            self.correction_pending_count = 0
            self.correction_rejection_count = 0
            self.pending_reset_count = 0

    def _reset_failed_metrics(self, reason):
        self.last_success = False
        self.last_score = float("inf")
        self.last_predicted_score = float("inf")
        self.last_objective_score = float("inf")
        self.last_prior_cost = 0.0
        self.last_scan_improvement_m2 = 0.0
        self.last_objective_improvement_m2 = 0.0
        self.last_position_correction_m = 0.0
        self.last_yaw_correction_rad = 0.0
        self.last_correction_dx_m = 0.0
        self.last_correction_dy_m = 0.0
        self.last_correction_dyaw_rad = 0.0
        self.last_proposed_correction_dx_m = 0.0
        self.last_proposed_correction_dy_m = 0.0
        self.last_proposed_correction_dyaw_rad = 0.0
        self.last_confirmation_count = 0
        self.last_cooldown_scans_remaining = (
            self._cooldown_scans_remaining
        )
        self._clear_pending_correction(count_reset=True)
        self.last_selection_reason = reason
        self.last_candidate_count = 0

    def estimate(self, scan, predicted_pose):
        """Estimate [yaw, x, y] around an odometry-predicted pose."""

        predicted_pose = np.asarray(predicted_pose, dtype=float)[:3].copy()
        measured = np.asarray(scan["ranges"], dtype=float)
        valid = (
            np.isfinite(measured)
            & (measured >= float(scan["range_min"]))
            & (measured < float(scan["range_max"]) - 1e-6)
        )
        self.last_valid_beams = int(np.count_nonzero(valid))
        if self.last_valid_beams < self.minimum_valid_beams:
            self._reset_failed_metrics("insufficient_valid_beams")
            return predicted_pose, float("inf")

        xy_offsets = self._offsets(self.search_xy_m, self.xy_step_m)
        theta_offsets = self._offsets(
            self.search_theta_rad,
            self.theta_step_rad,
        )
        predicted_score = self._scan_score(predicted_pose, measured, scan)
        best_pose = predicted_pose.copy()
        best_scan_score = predicted_score
        best_objective = predicted_score
        best_prior_cost = 0.0
        best_position_correction = 0.0
        best_yaw_correction = 0.0
        best_dx = 0.0
        best_dy = 0.0
        best_dtheta = 0.0
        candidate_count = 0
        for dtheta in theta_offsets:
            for dx in xy_offsets:
                for dy in xy_offsets:
                    position_correction = math.hypot(float(dx), float(dy))
                    yaw_correction = abs(float(dtheta))
                    if (
                        position_correction
                        > self.maximum_position_correction_m + 1e-12
                        or yaw_correction
                        > self.maximum_yaw_correction_rad + 1e-12
                    ):
                        continue
                    candidate_count += 1
                    candidate = predicted_pose.copy()
                    candidate[0] = normalize_angle(candidate[0] + dtheta)
                    candidate[1] += dx
                    candidate[2] += dy
                    if (
                        position_correction <= 1e-12
                        and yaw_correction <= 1e-12
                    ):
                        scan_score = predicted_score
                    else:
                        scan_score = self._scan_score(candidate, measured, scan)
                    prior_cost = (
                        self.position_prior_weight * position_correction ** 2
                        + self.yaw_prior_weight * yaw_correction ** 2
                    )
                    objective = scan_score + prior_cost
                    correction_norm = position_correction + yaw_correction
                    best_correction_norm = (
                        best_position_correction + best_yaw_correction
                    )
                    if (
                        objective < best_objective - 1e-12
                        or (
                            abs(objective - best_objective) <= 1e-12
                            and correction_norm < best_correction_norm
                        )
                    ):
                        best_scan_score = scan_score
                        best_objective = objective
                        best_prior_cost = prior_cost
                        best_position_correction = position_correction
                        best_yaw_correction = yaw_correction
                        best_dx = float(dx)
                        best_dy = float(dy)
                        best_dtheta = float(dtheta)
                        best_pose = candidate

        selection_reason = "prediction_best"
        scan_improvement = predicted_score - best_scan_score
        objective_improvement = predicted_score - best_objective
        correction_requested = (
            best_position_correction > 1e-12
            or best_yaw_correction > 1e-12
        )
        proposed_dx = best_dx
        proposed_dy = best_dy
        proposed_dtheta = best_dtheta
        proposed_scan_improvement = scan_improvement
        proposed_objective_improvement = objective_improvement

        def keep_prediction(reason):
            nonlocal best_pose
            nonlocal best_scan_score
            nonlocal best_objective
            nonlocal best_prior_cost
            nonlocal best_position_correction
            nonlocal best_yaw_correction
            nonlocal best_dx
            nonlocal best_dy
            nonlocal best_dtheta
            nonlocal selection_reason
            best_pose = predicted_pose.copy()
            best_scan_score = predicted_score
            best_objective = predicted_score
            best_prior_cost = 0.0
            best_position_correction = 0.0
            best_yaw_correction = 0.0
            best_dx = 0.0
            best_dy = 0.0
            best_dtheta = 0.0
            selection_reason = reason

        cooldown_active = self._cooldown_scans_remaining > 0
        if cooldown_active:
            self._cooldown_scans_remaining -= 1
        self.last_confirmation_count = 0

        if correction_requested:
            self.correction_proposal_count += 1
        if (
            correction_requested
            and math.isfinite(predicted_score)
            and predicted_score <= self.maximum_match_score_m2
            and scan_improvement < self.minimum_scan_score_improvement_m2
        ):
            keep_prediction("hysteresis_kept_prediction")
            self.correction_rejection_count += 1
        elif (
            correction_requested
            and math.isfinite(predicted_score)
            and predicted_score <= self.maximum_match_score_m2
            and objective_improvement
            < self.minimum_objective_improvement_m2
        ):
            keep_prediction("objective_hysteresis_kept_prediction")
            self.correction_rejection_count += 1
        elif correction_requested:
            selection_reason = "scan_correction"

        temporal_gate_enabled = (
            selection_reason == "scan_correction"
            and math.isfinite(predicted_score)
            and predicted_score <= self.maximum_match_score_m2
        )
        if temporal_gate_enabled and cooldown_active:
            keep_prediction("cooldown_kept_prediction")
            self.correction_rejection_count += 1
            self._clear_pending_correction(count_reset=True)
        elif (
            temporal_gate_enabled
            and self.correction_confirmation_scans > 1
        ):
            if self._pending_matches(
                proposed_dx,
                proposed_dy,
                proposed_dtheta,
            ):
                self._pending_confirmation_count += 1
            else:
                self._clear_pending_correction(count_reset=True)
                self._remember_pending_correction(
                    proposed_dx,
                    proposed_dy,
                    proposed_dtheta,
                )
            self.last_confirmation_count = (
                self._pending_confirmation_count
            )
            if (
                self._pending_confirmation_count
                < self.correction_confirmation_scans
            ):
                keep_prediction("confirmation_pending")
                self.correction_pending_count += 1
            else:
                self._clear_pending_correction()
                self._cooldown_scans_remaining = (
                    self.correction_cooldown_scans
                )
        elif selection_reason == "scan_correction":
            self._clear_pending_correction(count_reset=True)
            self._cooldown_scans_remaining = self.correction_cooldown_scans
        else:
            self._clear_pending_correction(count_reset=True)

        self.last_cooldown_scans_remaining = (
            self._cooldown_scans_remaining
        )
        self.last_candidate_count = candidate_count
        self.last_score = best_scan_score
        self.last_predicted_score = predicted_score
        self.last_objective_score = best_objective
        self.last_prior_cost = best_prior_cost
        self.last_scan_improvement_m2 = proposed_scan_improvement
        self.last_objective_improvement_m2 = (
            proposed_objective_improvement
        )
        self.last_position_correction_m = best_position_correction
        self.last_yaw_correction_rad = best_yaw_correction
        self.last_correction_dx_m = best_dx
        self.last_correction_dy_m = best_dy
        self.last_correction_dyaw_rad = best_dtheta
        self.last_proposed_correction_dx_m = proposed_dx
        self.last_proposed_correction_dy_m = proposed_dy
        self.last_proposed_correction_dyaw_rad = proposed_dtheta
        self.last_selection_reason = selection_reason
        self.last_success = bool(
            math.isfinite(best_scan_score)
            and best_scan_score <= self.maximum_match_score_m2
        )
        if not self.last_success:
            self.last_selection_reason = "match_score_rejected"
            self.last_position_correction_m = 0.0
            self.last_yaw_correction_rad = 0.0
            self.last_correction_dx_m = 0.0
            self.last_correction_dy_m = 0.0
            self.last_correction_dyaw_rad = 0.0
            return predicted_pose, best_scan_score
        if selection_reason == "scan_correction":
            self.correction_event_id += 1
        return best_pose, best_scan_score


__all__ = [
    "GridMapLocalizer",
    "OccupancyMap",
    "_compose_pose",
    "_inverse_pose",
    "normalize_angle",
    "odom_delta",
]
