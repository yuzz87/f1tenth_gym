import pytest

from real_vehicle_integration.contracts import ActuatorOutput
from real_vehicle_integration.contracts import ControlCommand
from real_vehicle_integration.actuator_policy import is_neutral_stop
from real_vehicle_integration.pwm_backends import MockPwmBackend, create_pwm_backend


def test_mock_backend_records_and_neutralizes():
    backend = MockPwmBackend()
    backend.apply(ActuatorOutput(1.0, 1, 10.16, 10.8, True, "running"))
    backend.neutral(1.1, "timeout")
    assert backend.history[-1].esc_duty_percent == 10.30
    assert not backend.history[-1].enabled


def test_hardware_backend_is_guarded():
    with pytest.raises(RuntimeError, match="disabled"):
        create_pwm_backend("hardware", hardware_output_enabled=False)
    with pytest.raises(RuntimeError, match="not implemented"):
        create_pwm_backend("hardware", hardware_output_enabled=True)


def test_unknown_backend_is_rejected():
    with pytest.raises(ValueError):
        create_pwm_backend("unknown")


def test_neutral_stop_requires_zero_motion_and_center_request():
    stopped = ControlCommand(1.0, 1, 0.0, 0.0, 0.0)
    steering = ControlCommand(1.0, 2, 0.0, 0.1, 0.0)
    assert is_neutral_stop(stopped)
    assert not is_neutral_stop(steering)
