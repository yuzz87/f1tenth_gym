import math

import pytest

from real_vehicle_integration.odom_estimator import (
    CALIBRATED_ENCODER_COUNTS_PER_REVOLUTION,
    CALIBRATED_ENCODER_DISTANCE_PER_COUNT_M,
    BicycleOdomEstimator,
    encoder_count_to_distance,
    normalize_angle,
)


def test_straight_forward_motion_updates_x_only():
    estimator = BicycleOdomEstimator(wheelbase_m=0.25)

    estimator.integrate(1.0, 0.0)

    yaw, x, y = estimator.pose()
    assert yaw == pytest.approx(0.0)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(0.0)


def test_reverse_motion_has_negative_distance():
    estimator = BicycleOdomEstimator(wheelbase_m=0.25)

    estimator.integrate(-0.5, 0.0)

    assert estimator.pose()[1] == pytest.approx(-0.5)
    assert estimator.distance_m == pytest.approx(-0.5)


def test_measured_encoder_calibration_maps_average_count_to_two_meters():
    assert CALIBRATED_ENCODER_COUNTS_PER_REVOLUTION == 144.0
    assert CALIBRATED_ENCODER_DISTANCE_PER_COUNT_M == pytest.approx(
        1.0 / 706.5
    )
    assert encoder_count_to_distance(1413) == pytest.approx(2.0)
    assert encoder_count_to_distance(-1413) == pytest.approx(-2.0)


def test_encoder_count_conversion_rejects_invalid_scale():
    with pytest.raises(ValueError, match="distance_per_count_m"):
        encoder_count_to_distance(1, 0.0)


def test_positive_steering_produces_positive_yaw():
    estimator = BicycleOdomEstimator(wheelbase_m=0.25)

    estimator.integrate(0.25, math.atan(0.25 / 2.0))

    yaw, _x, _y = estimator.pose()
    assert yaw > 0.0
    assert yaw == pytest.approx(0.125, abs=1e-9)


def test_circle_returns_close_to_start_after_one_lap():
    radius_m = 2.0
    wheelbase_m = 0.25
    steering = math.atan(wheelbase_m / radius_m)
    estimator = BicycleOdomEstimator(wheelbase_m=wheelbase_m)

    estimator.integrate(2.0 * math.pi * radius_m, steering)

    yaw, x, y = estimator.pose()
    assert normalize_angle(yaw) == pytest.approx(0.0, abs=1e-9)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
