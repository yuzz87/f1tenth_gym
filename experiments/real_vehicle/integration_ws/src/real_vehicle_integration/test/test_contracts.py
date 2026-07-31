import math

import pytest

from real_vehicle_integration.contracts import (
    ControlCommand,
    PoseEstimate,
    neutral_control,
    validate_control,
    validate_pose,
)


def test_valid_contracts_are_returned():
    pose = PoseEstimate(1.0, 0.1, 2.0, -1.0)
    command = ControlCommand(1.0, 2, 0.2, 0.1, 0.2)
    assert validate_pose(pose, 1.05, 0.2) is pose
    assert validate_control(command, 1.05, 0.1) == command


@pytest.mark.parametrize(
    "command",
    [
        ControlCommand(1.0, 0, math.nan, 0.0, 0.0),
        ControlCommand(1.0, 0, 0.31, 0.0, 0.0),
        ControlCommand(1.0, 0, 0.1, 0.4, 0.0),
        ControlCommand(1.0, 0, 0.1, 0.0, 0.8),
    ],
)
def test_invalid_control_is_rejected(command):
    with pytest.raises(ValueError):
        validate_control(command, 1.0, 0.1)


def test_stale_and_future_messages_are_rejected():
    command = ControlCommand(1.0, 0, 0.1, 0.0, 0.0)
    with pytest.raises(ValueError, match="stale"):
        validate_control(command, 1.2, 0.1)
    with pytest.raises(ValueError, match="future"):
        validate_control(command, 0.8, 0.1)


def test_neutral_control_is_stationary():
    command = neutral_control(3.0, sequence_id=4)
    assert command.sequence_id == 4
    assert command.speed_mps == 0.0
    assert command.steering_angle_rad == 0.0


def test_float32_boundary_rounding_is_normalized():
    command = ControlCommand(1.0, 1, 0.30000001192092896, 0.0, 0.0)
    normalized = validate_control(command, 1.0, 0.1)
    assert normalized.speed_mps == 0.30
