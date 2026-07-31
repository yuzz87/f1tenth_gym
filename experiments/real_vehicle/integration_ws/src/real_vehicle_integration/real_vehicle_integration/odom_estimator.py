"""ROS-independent bicycle-model odometry integration."""

import math


CALIBRATED_ENCODER_COUNTS_PER_REVOLUTION = 144.0
CALIBRATED_ENCODER_COUNT_PER_METER = 706.5
CALIBRATED_ENCODER_DISTANCE_PER_COUNT_M = (
    1.0 / CALIBRATED_ENCODER_COUNT_PER_METER
)
ENCODER_CALIBRATION_LABEL = "measured_known_distance_2m_3_trials_2026_07_22"


def encoder_count_to_distance(delta_count, distance_per_count_m=None):
    """Convert a signed quadrature count delta to calibrated travel distance."""

    if distance_per_count_m is None:
        distance_per_count_m = CALIBRATED_ENCODER_DISTANCE_PER_COUNT_M
    distance_per_count_m = float(distance_per_count_m)
    if distance_per_count_m <= 0.0:
        raise ValueError("distance_per_count_m must be positive")
    return float(delta_count) * distance_per_count_m


def normalize_angle(angle_rad):
    """Normalize an angle to [-pi, pi)."""

    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


class BicycleOdomEstimator:
    """Integrate signed wheel distance using a rear-axle bicycle model."""

    def __init__(self, wheelbase_m=0.25, initial_pose=None):
        if float(wheelbase_m) <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        self.wheelbase_m = float(wheelbase_m)
        pose = (0.0, 0.0, 0.0) if initial_pose is None else tuple(initial_pose)
        if len(pose) != 3:
            raise ValueError("initial_pose must contain yaw, x, and y")
        self.yaw_rad = normalize_angle(pose[0])
        self.x_m = float(pose[1])
        self.y_m = float(pose[2])
        self.distance_m = 0.0

    def reset(self, initial_pose=None):
        pose = (0.0, 0.0, 0.0) if initial_pose is None else tuple(initial_pose)
        if len(pose) != 3:
            raise ValueError("initial_pose must contain yaw, x, and y")
        self.yaw_rad = normalize_angle(pose[0])
        self.x_m = float(pose[1])
        self.y_m = float(pose[2])
        self.distance_m = 0.0

    def integrate(self, distance_m, steering_angle_rad):
        """Integrate one signed distance increment and return the pose."""

        distance_m = float(distance_m)
        steering_angle_rad = float(steering_angle_rad)
        yaw_before = self.yaw_rad
        curvature = math.tan(steering_angle_rad) / self.wheelbase_m
        yaw_delta = distance_m * curvature

        if abs(yaw_delta) < 1e-9:
            self.x_m += distance_m * math.cos(yaw_before)
            self.y_m += distance_m * math.sin(yaw_before)
        else:
            radius_m = distance_m / yaw_delta
            yaw_after = yaw_before + yaw_delta
            self.x_m += radius_m * (
                math.sin(yaw_after) - math.sin(yaw_before)
            )
            self.y_m += radius_m * (
                -math.cos(yaw_after) + math.cos(yaw_before)
            )
            self.yaw_rad = normalize_angle(yaw_after)

        self.distance_m += distance_m
        return self.pose()

    def pose(self):
        return self.yaw_rad, self.x_m, self.y_m
