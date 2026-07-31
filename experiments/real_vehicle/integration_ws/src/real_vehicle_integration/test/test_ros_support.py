import math

import pytest

pytest.importorskip("ackermann_msgs")

from real_vehicle_integration.contracts import ControlCommand
from real_vehicle_integration.ros_support import (
    ackermann_message_to_contract,
    contract_to_ackermann_message,
    yaw_to_quaternion,
)


def test_ackermann_control_round_trip():
    command = ControlCommand(0.0, 7, 0.2, 0.1, -0.3)
    message = contract_to_ackermann_message(command)
    restored = ackermann_message_to_contract(message, sequence_id=7)
    assert restored.speed_mps == pytest.approx(command.speed_mps)
    assert restored.steering_angle_rad == pytest.approx(command.steering_angle_rad)
    assert restored.steering_rate_rad_s == pytest.approx(
        command.steering_rate_rad_s
    )


def test_yaw_quaternion_is_normalized():
    values = yaw_to_quaternion(math.pi / 3.0)
    assert sum(value * value for value in values) == pytest.approx(1.0)
