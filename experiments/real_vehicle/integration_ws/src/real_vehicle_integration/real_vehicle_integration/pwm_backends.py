"""PWM backend boundary. Hardware output is deliberately unavailable here."""

from dataclasses import dataclass

from .contracts import ActuatorOutput


@dataclass(frozen=True)
class PwmSample:
    stamp_s: float
    esc_duty_percent: float
    steering_duty_percent: float
    enabled: bool
    reason: str


class MockPwmBackend:
    def __init__(self, esc_neutral=10.30, steering_neutral=10.895):
        self.esc_neutral = float(esc_neutral)
        self.steering_neutral = float(steering_neutral)
        self.history = []
        self.closed = False

    def apply(self, output: ActuatorOutput):
        if self.closed:
            raise RuntimeError("PWM backend is closed")
        sample = PwmSample(
            output.stamp_s,
            output.esc_duty_percent,
            output.steering_duty_percent,
            output.enabled,
            output.reason,
        )
        self.history.append(sample)
        return sample

    def neutral(self, stamp_s, reason="neutral"):
        return self.apply(ActuatorOutput(
            stamp_s=float(stamp_s),
            sequence_id=0,
            esc_duty_percent=self.esc_neutral,
            steering_duty_percent=self.steering_neutral,
            enabled=False,
            reason=str(reason),
        ))

    def close(self, stamp_s=0.0):
        if not self.closed:
            self.neutral(stamp_s, "backend_close")
            self.closed = True


class HardwarePwmBackend:
    """Refuse hardware output until pins and the PWM library are confirmed."""

    def __init__(self, *args, **kwargs):
        raise RuntimeError(
            "hardware PWM is not implemented: confirm Raspberry Pi pins, "
            "PWM library, and actuator calibration first"
        )


def create_pwm_backend(name, hardware_output_enabled=False, **kwargs):
    backend_name = str(name).lower()
    if backend_name == "mock":
        return MockPwmBackend(**kwargs)
    if backend_name == "hardware":
        if not hardware_output_enabled:
            raise RuntimeError("hardware PWM requested while hardware output is disabled")
        return HardwarePwmBackend(**kwargs)
    raise ValueError(f"unknown PWM backend: {name}")
