"""Step-by-step simulation state used by the real-vehicle GUI."""

import json
from pathlib import Path

import numpy as np

from ..adapters.command_adapter import CommandAdapter
from ..localization import DEFAULT_COMPLEX_OBSTACLES, GridSearchLocalizer, VirtualLidar
from ..models import ActuatorModel, step_model
from ..evaluation.common import (
    circle_reference,
    make_controller,
    straight_reference,
    write_rows,
)


class SimulationView:
    """Run one simulation step and expose the data needed for drawing."""

    def __init__(
        self,
        config,
        parameters,
        limits,
        controller_name="mppi",
        scenario="straight",
        noise_std=None,
        dropout_probability=None,
        delay_s=None,
        initial_x_error=0.0,
        initial_y_error=0.0,
        initial_heading_error=0.0,
        complex_map=False,
        max_steps=0,
    ):
        if scenario not in ("straight", "circle", "localized"):
            raise ValueError("scenario must be straight, circle, or localized")
        self.config = config
        self.parameters = parameters
        self.limits = limits
        self.controller_name = controller_name
        self.scenario = scenario
        self.max_steps = max(0, int(max_steps))
        self.initial_pose_error = np.array([
            float(initial_heading_error),
            float(initial_x_error),
            float(initial_y_error),
        ])
        self.dt = float(config["simulation"]["timestep_s"])
        self.controller_config = config["controller"]
        self.controller = make_controller(
            controller_name,
            parameters,
            limits,
            self.dt,
            self.controller_config,
        )
        self.adapter = CommandAdapter(
            limits,
            self.dt,
            esc_config=config.get("calibration", {}).get("esc", {}),
            allow_experimental_duty=bool(
                config["simulation"].get("allow_experimental_duty", False)
            ),
        )
        self.actuator = ActuatorModel(
            parameters,
            limits,
            config["simulation"].get("actuator", {}),
            seed=int(self.controller_config.get("seed", 7)),
        )

        lidar_config = dict(config["lidar"])
        if noise_std is not None:
            lidar_config["noise_std_m"] = float(noise_std)
        if dropout_probability is not None:
            lidar_config["dropout_probability"] = float(dropout_probability)
        if delay_s is not None:
            lidar_config["delay_s"] = float(delay_s)
        if complex_map:
            lidar_config["obstacles"] = DEFAULT_COMPLEX_OBSTACLES
        self.lidar = VirtualLidar(lidar_config, seed=7)
        self.localizer = GridSearchLocalizer(self.lidar) if scenario == "localized" else None

        self.speed = float(self.controller_config["speed_target_mps"])
        self.goal_x = float(config["simulation"]["straight_goal_x_m"])
        self.radius = float(config["simulation"]["circle_radius_m"])
        if scenario == "circle":
            lap_time = 2.0 * np.pi * self.radius / max(self.speed, 1e-9)
            self.duration_s = lap_time + 1.0
        else:
            self.duration_s = max(
                float(config["simulation"]["duration_s"]),
                self.goal_x / max(self.speed, 1e-9) + 2.0,
            )
        self.reset()

    def _initial_state(self):
        if self.scenario == "circle":
            steer = np.arctan(self.parameters.wheelbase_m / self.radius)
            return np.array([np.pi / 2.0, self.radius, 0.0, steer], dtype=float)
        return np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

    def reset(self):
        """Reset the simulation and controller to the initial condition."""
        self.controller.reset()
        self.state = self._initial_state()
        self.predicted_pose = self.state[:3].copy() + self.initial_pose_error
        self.actuator.reset(self.state)
        self.time_s = 0.0
        self.steps = 0
        self.reached_goal = False
        self.finished = False
        self.rows = []
        self.frame = {
            "true_state": self.state.copy(),
            "estimated_state": None,
            "scan": None,
            "reference": self._reference(0.0),
            "control": np.array([0.0, 0.0], dtype=float),
            "info": {"controller": self.controller_name, "solve_time_s": 0.0},
            "clamp": None,
            "command": None,
            "match_score": None,
            "reached_goal": False,
            "finished": False,
        }

    def _reference(self, timestamp_s):
        if self.scenario in ("straight", "localized"):
            return straight_reference(
                timestamp_s,
                self.controller.horizon,
                self.dt,
                self.speed,
                self.goal_x,
                y=0.0,
                stop_at_goal=self.reached_goal,
            )
        return circle_reference(
            timestamp_s,
            self.controller.horizon,
            self.dt,
            self.speed,
            self.radius,
            self.parameters,
        )

    def _is_finished(self):
        return (
            self.time_s >= self.duration_s - 1e-9
            or (self.max_steps > 0 and self.steps >= self.max_steps)
        )

    def step(self):
        """Advance one control period and return a drawable frame."""
        if self.finished:
            return self.frame

        scan = self.lidar.scan(self.state, self.time_s)
        estimated_state = None
        match_score = None
        if self.localizer is not None:
            estimated_pose, match_score = self.localizer.estimate(scan, self.predicted_pose)
            estimated_state = self.state.copy()
            estimated_state[:3] = estimated_pose
            estimated_state[3] = self.state[3]
            control_state = estimated_state
        else:
            control_state = self.state

        reference = self._reference(self.time_s)
        control, info = self.controller.plan(control_state, reference)
        _, command = self.adapter.model_input_to_command(self.state, control)
        next_state, clamp, actuator_log = self.actuator.step(
            self.state,
            control,
            self.parameters,
            self.dt,
            integrator=str(self.config["simulation"].get("integrator", "rk4")),
        )
        if self.scenario in ("straight", "localized"):
            self.reached_goal = self.reached_goal or bool(
                next_state[1] >= self.goal_x - 0.05
            )

        if self.localizer is not None:
            predicted_next, _ = step_model(
                self.predicted_pose_to_state(),
                control,
                self.parameters,
                self.limits,
                self.dt,
            )
            self.predicted_pose = predicted_next[:3]

        row = self._make_row(
            reference[0],
            estimated_state,
            scan,
            control,
            command,
            clamp,
            actuator_log,
            info,
            match_score,
        )
        self.rows.append(row)
        self.state = next_state
        self.time_s += self.dt
        self.steps += 1
        self.finished = self._is_finished()
        self.frame = {
            "true_state": self.state.copy(),
            "estimated_state": None if estimated_state is None else estimated_state.copy(),
            "scan": scan,
            "reference": reference,
            "control": np.asarray(control, dtype=float).copy(),
            "info": info,
            "clamp": clamp,
            "command": command,
            "actuator": actuator_log,
            "match_score": match_score,
            "reached_goal": self.reached_goal,
            "finished": self.finished,
        }
        return self.frame

    def predicted_pose_to_state(self):
        state = np.zeros(4, dtype=float)
        state[:3] = self.predicted_pose
        state[3] = self.state[3]
        return state

    def _make_row(self, reference, estimated_state, scan, control, command, clamp, actuator, info, match_score):
        row = {
            "timestamp_s": self.time_s,
            "reference_x_m": float(reference[1]),
            "reference_y_m": float(reference[2]),
            "reference_phi_rad": float(reference[0]),
            "reference_psi_rad": float(reference[3]),
            "actual_x_m": float(self.state[1]),
            "actual_y_m": float(self.state[2]),
            "actual_phi_rad": float(self.state[0]),
            "actual_psi_rad": float(self.state[3]),
            "target_speed_mps": float(control[0]),
            "target_steer_rate_rad_s": float(control[1]),
            "requested_speed_mps": float(actuator.requested_v),
            "actuator_target_speed_mps": float(actuator.target_v),
            "actuator_applied_speed_mps": float(actuator.applied_v),
            "applied_speed_mps": float(clamp.applied_v),
            "applied_steer_rad": float(clamp.next_steer_rad),
            "actuator_target_steer_rad": float(actuator.target_psi),
            "actuator_applied_steer_rad": float(actuator.applied_psi),
            "requested_steer_rad": float(command.requested_psi),
            "requested_esc_duty_percent": float(command.requested_esc_duty_percent),
            "steer_duty_percent": float(command.steer_duty_percent),
            "esc_duty_percent": float(command.esc_duty_percent),
            "clamped_esc_duty": int(command.clamped_esc_duty),
            "experimental_duty": int(command.experimental_duty),
            "clamped_speed": int(clamp.clamped_speed),
            "clamped_steer": int(clamp.clamped_steer),
            "controller_time_ms": 1000.0 * float(info["solve_time_s"]),
            "scan_valid_ratio": float(
                np.mean(np.asarray(scan["ranges"]) < self.lidar.range_max_m - 1e-9)
            ),
            "scan_age_s": self.time_s - float(scan["header"]["stamp_s"]),
            "match_score": "" if match_score is None else float(match_score),
        }
        if estimated_state is not None:
            row.update({
                "estimated_x_m": float(estimated_state[1]),
                "estimated_y_m": float(estimated_state[2]),
                "estimated_phi_rad": float(estimated_state[0]),
            })
        else:
            row.update({"estimated_x_m": "", "estimated_y_m": "", "estimated_phi_rad": ""})
        return row

    def save(self, output):
        """Save the GUI run using the same CSV convention as batch evaluations."""
        output = Path(output)
        write_rows(output, self.rows)
        if not self.rows:
            summary = {"controller": self.controller_name, "scenario": self.scenario, "steps": 0}
        else:
            actual = np.array([[r["actual_x_m"], r["actual_y_m"]] for r in self.rows], dtype=float)
            reference = np.array([[r["reference_x_m"], r["reference_y_m"]] for r in self.rows], dtype=float)
            errors = np.linalg.norm(actual - reference, axis=1)
            summary = {
                "controller": self.controller_name,
                "scenario": self.scenario,
                "steps": len(self.rows),
                "sim_elapsed_time_s": self.time_s,
                "reached_goal": self.reached_goal,
                "finished": self.finished,
                "position_rmse_m": float(np.sqrt(np.mean(errors**2))),
                "position_max_error_m": float(np.max(errors)),
                "mean_controller_time_ms": float(np.mean([r["controller_time_ms"] for r in self.rows])),
                "max_controller_time_ms": float(np.max([r["controller_time_ms"] for r in self.rows])),
                "mean_scan_valid_ratio": float(np.mean([r["scan_valid_ratio"] for r in self.rows])),
                "mean_scan_age_s": float(np.mean([r["scan_age_s"] for r in self.rows])),
                "speed_clamp_count": int(sum(r["clamped_speed"] for r in self.rows)),
                "steer_clamp_count": int(sum(r["clamped_steer"] for r in self.rows)),
            }
        output.with_suffix(".summary.json").parent.mkdir(parents=True, exist_ok=True)
        output.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return summary
