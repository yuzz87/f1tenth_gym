"""Hardware-free final checks before connecting the real vehicle."""

import json
import sys

import numpy as np

from ..adapters import CommandAdapter
from ..controllers import RealVehicleMPC, RealVehicleMPPI
from ..localization import GridSearchLocalizer, VirtualLidar
from ..models import ActuatorModel, step_model
from .common import DEFAULT_CONFIG, load_real_config


def main():
    config, parameters, limits = load_real_config(DEFAULT_CONFIG)
    dt = float(config["simulation"]["timestep_s"])
    state = np.array([0.0, 0.0, 0.0, 0.0])
    esc_config = config.get("calibration", {}).get("esc", {})
    adapter = CommandAdapter(
        limits,
        dt,
        esc_config=esc_config,
        allow_experimental_duty=False,
    )
    action, command = adapter.model_input_to_command(state, [1.0, 2.0])
    next_state, clamp = step_model(state, [1.0, 2.0], parameters, limits, dt)
    actuator = ActuatorModel(parameters, limits, config["simulation"].get("actuator", {}))
    actuator.reset(state)
    delayed_state, _delayed_clamp, actuator_log = actuator.step(
        state,
        [limits.speed_max_mps, 0.0],
        parameters,
        dt,
        integrator=str(config["simulation"].get("integrator", "rk4")),
    )
    guarded_duty, guarded = adapter.normalize_esc_duty(9.70)
    lidar = VirtualLidar(config["lidar"])
    scan = lidar.scan(state, 0.0)
    estimate, score = GridSearchLocalizer(lidar).estimate(scan, state[:3])
    checks = {
        "config_loaded": bool(parameters.wheelbase_m == 0.25),
        "speed_clamp": bool(clamp.clamped_speed and command.clamped_speed),
        "steer_clamp": bool(clamp.clamped_steer and command.clamped_steer),
        "zero_stop": bool(adapter.duty_to_gym_command(10.30, 10.30)[1] == 0.0),
        "hardware_duty_guard": bool(guarded and np.isclose(guarded_duty, 10.10)),
        "scan_contract": bool(len(scan["ranges"]) == config["lidar"]["num_beams"] and scan["angle_min"] < scan["angle_max"]),
        "scan_full_circle": bool(abs(float(scan["angle_max"] - scan["angle_min"]) - 2.0 * np.pi) < 1e-6),
        "localizer_finite": bool(np.isfinite(estimate).all() and np.isfinite(score)),
        "hardware_modules_absent": bool("pigpio" not in sys.modules and "RPi.GPIO" not in sys.modules),
        "hardware_output_disabled": bool(
            not config["simulation"].get("hardware_output_enabled", False)
        ),
        "controllers_imported": bool(all((RealVehicleMPC, RealVehicleMPPI))),
        "next_state_finite": bool(np.isfinite(next_state).all() and np.isfinite(action).all()),
        "actuator_finite": bool(np.isfinite(delayed_state).all() and np.isfinite(actuator_log.applied_v)),
    }
    print(json.dumps(checks, indent=2))
    if not all(checks.values()):
        raise SystemExit(1)
    print("preflight: PASS (simulation only; no GPIO/pigpio/PWM output)")


if __name__ == "__main__":
    main()
