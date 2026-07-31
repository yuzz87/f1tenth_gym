"""Configuration, references, and CSV simulation utilities."""

import csv
from pathlib import Path

import numpy as np
import yaml

from ..adapters.command_adapter import CommandAdapter
from ..controllers import RealVehicleMPC, RealVehicleMPPI, RealVehiclePurePursuit
from ..models import ActuatorModel, VehicleLimits, VehicleParameters, normalize_angle


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = REPO_ROOT / "experiments/real_vehicle/config/real_vehicle.yaml"


def load_real_config(path=DEFAULT_CONFIG):
    with open(path, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}
    vehicle = config["vehicle"]
    limits = config["limits"]
    parameters = VehicleParameters(
        wheelbase_m=float(vehicle["wheelbase_m"]),
        wheel_diameter_m=float(vehicle["wheel_diameter_m"]),
        encoder_teeth=int(vehicle["encoder_teeth"]),
    )
    vehicle_limits = VehicleLimits(**{key: float(value) for key, value in limits.items()})
    return config, parameters, vehicle_limits


def make_controller(name, parameters, limits, timestep_s, config):
    name = name.lower()
    if name == "mpc":
        return RealVehicleMPC(parameters, limits, timestep_s, config)
    if name == "mppi":
        return RealVehicleMPPI(parameters, limits, timestep_s, config)
    if name in ("pure_pursuit", "pure-pursuit"):
        return RealVehiclePurePursuit(parameters, limits, timestep_s, config)
    raise ValueError("controller must be 'pure_pursuit', 'mpc', or 'mppi'")


def straight_reference(time_s, horizon, timestep_s, speed_mps, goal_x, y=-2.0, stop_at_goal=False):
    """Reference state/control sequence for the 4 m straight test."""
    times = time_s + timestep_s * np.arange(1, horizon + 1, dtype=float)
    positions = np.minimum(speed_mps * times, goal_x)
    speeds = np.full(horizon, 0.0 if stop_at_goal else speed_mps)
    return np.column_stack((
        np.zeros(horizon),
        positions,
        np.full(horizon, y),
        np.zeros(horizon),
        speeds,
        np.zeros(horizon),
    ))


def circle_reference(time_s, horizon, timestep_s, speed_mps, radius, parameters):
    """Reference circle starting at (R, 0) and heading +y."""
    times = time_s + timestep_s * np.arange(1, horizon + 1, dtype=float)
    phase = speed_mps * times / radius
    steer = np.arctan(parameters.wheelbase_m / radius)
    return np.column_stack((
        phase + np.pi / 2.0,
        radius * np.cos(phase),
        radius * np.sin(phase),
        np.full(horizon, steer),
        np.full(horizon, speed_mps),
        np.zeros(horizon),
    ))


def write_rows(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_simulation(
    controller_name,
    reference_function,
    initial_state,
    duration_s,
    config,
    parameters,
    limits,
    goal_function=None,
):
    timestep_s = float(config["simulation"]["timestep_s"])
    controller = make_controller(
        controller_name,
        parameters,
        limits,
        timestep_s,
        config["controller"],
    )
    adapter = CommandAdapter(
        limits,
        timestep_s,
        esc_config=config.get("calibration", {}).get("esc", {}),
        allow_experimental_duty=bool(
            config["simulation"].get("allow_experimental_duty", False)
        ),
    )
    state = np.asarray(initial_state, dtype=float).copy()
    actuator = ActuatorModel(
        parameters,
        limits,
        config["simulation"].get("actuator", {}),
        seed=int(config.get("controller", {}).get("seed", 7)),
    )
    actuator.reset(state)
    integrator = str(config["simulation"].get("integrator", "rk4")).lower()
    rows = []
    total_steps = int(np.ceil(float(duration_s) / timestep_s))
    reached = False
    solve_times = []
    for step_index in range(total_steps):
        time_s = step_index * timestep_s
        reference = reference_function(time_s, controller.horizon, state, reached)
        control, info = controller.plan(state, reference)
        gym_command, command_log = adapter.model_input_to_command(state, control)
        next_state, clamp_log, actuator_log = actuator.step(
            state,
            control,
            parameters,
            timestep_s,
            integrator=integrator,
        )
        reference_state = reference[0]
        solve_times.append(float(info["solve_time_s"]))
        if goal_function is not None:
            reached = reached or bool(goal_function(next_state, time_s + timestep_s))
        rows.append({
            "timestamp_s": time_s,
            "reference_x_m": reference_state[1],
            "reference_y_m": reference_state[2],
            "reference_phi_rad": reference_state[0],
            "reference_psi_rad": reference_state[3],
            "actual_x_m": state[1],
            "actual_y_m": state[2],
            "actual_phi_rad": state[0],
            "actual_psi_rad": state[3],
            "target_speed_mps": control[0],
            "target_steer_rate_rad_s": control[1],
            "requested_speed_mps": actuator_log.requested_v,
            "actuator_target_speed_mps": actuator_log.target_v,
            "actuator_applied_speed_mps": actuator_log.applied_v,
            "applied_speed_mps": clamp_log.applied_v,
            "applied_steer_rad": clamp_log.next_steer_rad,
            "actuator_target_steer_rad": actuator_log.target_psi,
            "actuator_applied_steer_rad": actuator_log.applied_psi,
            "requested_steer_rad": command_log.requested_psi,
            "requested_esc_duty_percent": command_log.requested_esc_duty_percent,
            "steer_duty_percent": command_log.steer_duty_percent,
            "esc_duty_percent": command_log.esc_duty_percent,
            "clamped_esc_duty": int(command_log.clamped_esc_duty),
            "experimental_duty": int(command_log.experimental_duty),
            "clamped_speed": int(clamp_log.clamped_speed),
            "clamped_steer": int(clamp_log.clamped_steer),
            "controller_time_ms": 1000.0 * info["solve_time_s"],
            "mpc_success": int(info["success"]),
            "controller_deadline_exceeded": int(
                info.get("deadline_exceeded", False)
            ),
        })
        state = next_state
    positions = np.array([[row["actual_x_m"], row["actual_y_m"]] for row in rows])
    references = np.array([[row["reference_x_m"], row["reference_y_m"]] for row in rows])
    position_errors = np.linalg.norm(positions - references, axis=1)
    return rows, {
        "controller": controller_name,
        "steps": len(rows),
        "sim_elapsed_time_s": float(len(rows) * timestep_s),
        "reached_goal": reached,
        "position_rmse_m": float(np.sqrt(np.mean(position_errors**2))),
        "position_max_error_m": float(np.max(position_errors)),
        "mean_controller_time_ms": float(1000.0 * np.mean(solve_times)),
        "max_controller_time_ms": float(1000.0 * np.max(solve_times)),
        "speed_clamp_count": int(sum(row["clamped_speed"] for row in rows)),
        "steer_clamp_count": int(sum(row["clamped_steer"] for row in rows)),
        "controller_deadline_count": int(sum(
            row["controller_deadline_exceeded"] for row in rows
        )),
    }


def heading_error(reference_phi, actual_phi):
    return float(normalize_angle(actual_phi - reference_phi))
