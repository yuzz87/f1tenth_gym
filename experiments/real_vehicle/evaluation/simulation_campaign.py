"""Run reproducible controller, LiDAR, plant, and network Monte Carlo cases."""

import argparse
import copy
import csv
from dataclasses import dataclass
import itertools
import json
from pathlib import Path
import time

import numpy as np
import yaml

from ..localization import GridSearchLocalizer, VirtualLidar
from ..models import ActuatorModel, normalize_angle, step_model
from .common import (
    DEFAULT_CONFIG,
    circle_reference,
    load_real_config,
    make_controller,
    straight_reference,
    write_rows,
)


DEFAULT_CAMPAIGN = DEFAULT_CONFIG.with_name("simulation_campaign.yaml")


@dataclass
class ChannelStatistics:
    received: int = 0
    dropped: int = 0
    delivered: int = 0
    burst_dropped: int = 0
    outage_dropped: int = 0
    reordered: int = 0


class SimulationChannel:
    """Small deterministic channel model for non-ROS Monte Carlo runs."""

    def __init__(self, config=None, seed=1, prefix=""):
        config = dict(config or {})

        def key(name, default):
            return config.get(f"{prefix}{name}", default)
        self.delay_s = max(float(key("delay_s", 0.0)), 0.0)
        self.delay_jitter_s = max(float(key("delay_jitter_s", 0.0)), 0.0)
        self.dropout_probability = float(np.clip(
            key("dropout_probability", 0.0), 0.0, 1.0
        ))
        self.burst_start_probability = float(np.clip(
            key("burst_start_probability", 0.0), 0.0, 1.0
        ))
        self.burst_length = max(int(key("burst_length", 1)), 1)
        self.outage_after_s = float(key("outage_after_s", -1.0))
        self.outage_duration_s = max(float(key("outage_duration_s", 0.0)), 0.0)
        self.rng = np.random.default_rng(int(seed))
        self.pending = []
        self.sequence = itertools.count()
        self.burst_remaining = 0
        self.statistics = ChannelStatistics()

    def _outage_active(self, now_s):
        return (
            self.outage_after_s >= 0.0
            and self.outage_after_s
            <= now_s
            < self.outage_after_s + self.outage_duration_s
        )

    def send(self, value, now_s):
        self.statistics.received += 1
        if self._outage_active(now_s):
            self.statistics.dropped += 1
            self.statistics.outage_dropped += 1
            return
        if self.burst_remaining <= 0:
            if self.rng.random() < self.burst_start_probability:
                self.burst_remaining = self.burst_length
        if self.burst_remaining > 0:
            self.burst_remaining -= 1
            self.statistics.dropped += 1
            self.statistics.burst_dropped += 1
            return
        if self.rng.random() < self.dropout_probability:
            self.statistics.dropped += 1
            return
        jitter = 0.0
        if self.delay_jitter_s > 0.0:
            jitter = float(self.rng.normal(0.0, self.delay_jitter_s))
        release_s = now_s + max(self.delay_s + jitter, 0.0)
        if self.pending and release_s < max(item[0] for item in self.pending):
            self.statistics.reordered += 1
        self.pending.append((release_s, next(self.sequence), value))
        self.pending.sort(key=lambda item: (item[0], item[1]))

    def receive(self, now_s):
        ready = []
        while self.pending and self.pending[0][0] <= now_s:
            _release, _sequence, value = self.pending.pop(0)
            ready.append(value)
            self.statistics.delivered += 1
        return ready


def load_campaign(path=DEFAULT_CAMPAIGN):
    with Path(path).open("r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def expand_cases(campaign):
    dimensions = campaign["campaign"]
    names = (
        "seed",
        "controller",
        "scenario",
        "pose_source",
        "plant_profile",
        "lidar_profile",
        "network_profile",
    )
    values = (
        dimensions["seeds"],
        dimensions["controllers"],
        dimensions["scenarios"],
        dimensions["pose_sources"],
        dimensions["plant_profiles"],
        dimensions["lidar_profiles"],
        dimensions["network_profiles"],
    )
    return [dict(zip(names, combination)) for combination in itertools.product(*values)]


def _reference(case, elapsed_s, controller, dt, speed, config, parameters):
    if case["scenario"] == "straight":
        return straight_reference(
            elapsed_s,
            controller.horizon,
            dt,
            speed,
            float(config["simulation"]["straight_goal_x_m"]),
            y=0.0,
        )
    return circle_reference(
        elapsed_s,
        controller.horizon,
        dt,
        speed,
        float(config["simulation"]["circle_radius_m"]),
        parameters,
    )


def run_case(
    case,
    base_config,
    campaign,
    parameters,
    limits,
    output_dir=None,
    max_steps=0,
):
    config = copy.deepcopy(base_config)
    seed = int(case["seed"])
    config["controller"]["seed"] = seed
    config["simulation"]["actuator"].update(
        campaign["plant_profiles"][case["plant_profile"]]
    )
    config["lidar"].update(campaign["lidar_profiles"][case["lidar_profile"]])
    network = campaign["network_profiles"][case["network_profile"]]
    dt = float(config["simulation"]["timestep_s"])
    rate = float(campaign["evaluation"]["control_rates_hz"][case["controller"]])
    controller_period_steps = max(int(round(1.0 / (rate * dt))), 1)
    controller = make_controller(
        case["controller"], parameters, limits, 1.0 / rate, config["controller"]
    )
    actuator = ActuatorModel(
        parameters,
        limits,
        config["simulation"]["actuator"],
        seed=seed,
    )
    lidar = VirtualLidar(config["lidar"], seed=seed)
    localizer = GridSearchLocalizer(
        lidar,
        minimum_valid_beams=max(4, lidar.num_beams // 10),
        maximum_match_score=1.0,
    )
    sensor_channel = SimulationChannel(network, seed + 1000, "sensor_")
    control_channel = SimulationChannel(network, seed + 2000, "control_")
    speed = min(float(config["controller"]["speed_target_mps"]), limits.speed_max_mps)
    if case["scenario"] == "straight":
        state = np.array([0.0, 0.0, 0.0, 0.0])
        duration_s = float(campaign["evaluation"]["straight_duration_s"])
    else:
        radius = float(config["simulation"]["circle_radius_m"])
        state = np.array([
            np.pi / 2.0,
            radius,
            0.0,
            np.arctan(parameters.wheelbase_m / radius),
        ])
        duration_s = float(campaign["evaluation"]["circle_duration_s"])
    actuator.reset(state)
    predicted_state = state.copy()
    estimated_state = state.copy()
    applied_control = np.zeros(2, dtype=float)
    requested_control = np.zeros(2, dtype=float)
    latest_scan_stamp = None
    latest_pose_stamp = 0.0 if case["pose_source"] == "ground_truth" else None
    latest_control_stamp = None
    scan_period = 1.0 / max(float(config["lidar"]["update_rate_hz"]), 1e-6)
    next_scan_s = 0.0
    scan_rng = np.random.default_rng(seed + 3000)
    safety_latched = False
    safety_fault_reason = ""
    safety_stop_count = 0
    safety_fault_time_s = float("nan")
    started = False
    controller_times = []
    localization_errors = []
    heading_errors = []
    position_errors = []
    valid_ratios = []
    localizer_failures = 0
    deadline_exceeded = 0
    overrun_count = 0
    trajectory = []
    total_steps = int(np.ceil(duration_s / dt))
    if max_steps > 0:
        total_steps = min(total_steps, int(max_steps))

    for step_index in range(total_steps):
        now_s = step_index * dt
        if now_s + 1e-12 >= next_scan_s:
            scan = lidar.scan(state, now_s)
            sensor_channel.send(scan, now_s)
            jitter = float(config["lidar"].get("update_rate_jitter_fraction", 0.0))
            period_scale = max(1.0 + scan_rng.normal(0.0, jitter), 0.1)
            next_scan_s = now_s + scan_period * period_scale
        for scan in sensor_channel.receive(now_s):
            latest_scan_stamp = float(scan["header"]["stamp_s"])
            valid_ratios.append(float(scan.get("valid_ratio", 1.0)))
            if case["pose_source"] == "localized":
                estimate, _score = localizer.estimate(scan, predicted_state[:3])
                estimated_state[:3] = estimate
                if not localizer.last_success:
                    localizer_failures += 1
                latest_pose_stamp = latest_scan_stamp

        if case["pose_source"] == "ground_truth":
            estimated_state = state.copy()
            latest_pose_stamp = now_s
        localization_errors.append(float(np.linalg.norm(
            state[1:3] - estimated_state[1:3]
        )))
        heading_errors.append(float(normalize_angle(state[0] - estimated_state[0])))

        if step_index % controller_period_steps == 0:
            reference = _reference(
                case, now_s, controller, 1.0 / rate, speed, config, parameters
            )
            requested_control, info = controller.plan(estimated_state, reference)
            controller_times.append(1000.0 * float(info["solve_time_s"]))
            deadline_exceeded += int(info.get("deadline_exceeded", False))
            overrun_count += int(info["solve_time_s"] > 1.0 / rate)
            control_channel.send((now_s, requested_control.copy()), now_s)

        for command_stamp, command in control_channel.receive(now_s):
            latest_control_stamp = command_stamp
            if not safety_latched:
                applied_control = command

        sensor_timeout = float(campaign["evaluation"]["scan_timeout_s"])
        pose_timeout = float(campaign["evaluation"]["pose_timeout_s"])
        control_timeout = float(campaign["evaluation"]["control_timeout_s"])
        ready = all(value is not None for value in (
            latest_scan_stamp, latest_pose_stamp, latest_control_stamp
        ))
        if ready:
            started = True
        fault = ""
        if started and not safety_latched:
            if now_s - latest_scan_stamp > sensor_timeout:
                fault = "scan_timeout"
            elif now_s - latest_pose_stamp > pose_timeout:
                fault = "pose_timeout"
            elif now_s - latest_control_stamp > control_timeout:
                fault = "control_timeout"
        if fault:
            safety_latched = True
            safety_fault_reason = fault
            safety_fault_time_s = now_s
            safety_stop_count += 1
            applied_control = np.zeros(2, dtype=float)

        next_state, _clamp, _diagnostics = actuator.step(
            state,
            applied_control,
            parameters,
            dt,
            integrator=str(config["simulation"].get("integrator", "rk4")),
        )
        predicted_state, _clamp = step_model(
            predicted_state,
            applied_control,
            parameters,
            limits,
            dt,
            integrator=str(config["simulation"].get("integrator", "rk4")),
        )
        reference = _reference(case, now_s, controller, dt, speed, config, parameters)[0]
        position_error = float(np.linalg.norm(state[1:3] - reference[1:3]))
        position_errors.append(position_error)
        if output_dir is not None and campaign["campaign"].get("save_trajectories"):
            trajectory.append({
                "timestamp_s": now_s,
                "x_m": state[1],
                "y_m": state[2],
                "phi_rad": state[0],
                "estimated_x_m": estimated_state[1],
                "estimated_y_m": estimated_state[2],
                "reference_x_m": reference[1],
                "reference_y_m": reference[2],
                "speed_mps": applied_control[0],
                "position_error_m": position_error,
                "safety_latched": int(safety_latched),
            })
        state = next_state
        if case["pose_source"] == "localized":
            estimated_state[3] = predicted_state[3]

    radius = float(config["simulation"]["circle_radius_m"])
    goal_x = float(config["simulation"]["straight_goal_x_m"])
    reached_goal = bool(state[1] >= goal_x - 0.05) if case["scenario"] == "straight" else False
    if case["scenario"] == "circle":
        expected_phase = speed * (total_steps * dt) / radius
        actual_phase = float(np.arctan2(state[2], state[1]))
        phase_error = float(normalize_angle(actual_phase - expected_phase))
        radius_error = abs(float(np.hypot(state[1], state[2])) - radius)
        completed = bool(
            total_steps * dt >= 2.0 * np.pi * radius / max(speed, 1e-9) - dt
            and abs(phase_error) < 0.5
            and radius_error < 0.2
            and not safety_latched
        )
    else:
        phase_error = float("nan")
        radius_error = float("nan")
        completed = reached_goal and not safety_latched
    timing = np.asarray(controller_times or [0.0])
    summary = dict(case)
    summary.update({
        "run_id": "_".join(str(case[key]) for key in (
            "controller", "scenario", "pose_source", "lidar_profile",
            "plant_profile", "network_profile", "seed"
        )),
        "steps": total_steps,
        "completed": int(completed),
        "reached_goal": int(reached_goal),
        "position_rmse_m": float(np.sqrt(np.mean(np.square(position_errors)))),
        "localization_xy_rmse_m": float(np.sqrt(np.mean(np.square(localization_errors)))),
        "localization_heading_rmse_rad": float(np.sqrt(np.mean(np.square(heading_errors)))),
        "controller_mean_ms": float(np.mean(timing)),
        "controller_p95_ms": float(np.percentile(timing, 95)),
        "controller_max_ms": float(np.max(timing)),
        "control_overrun_rate": float(overrun_count / max(len(timing), 1)),
        "deadline_exceeded_count": deadline_exceeded,
        "safety_stop_count": safety_stop_count,
        "safety_fault_reason": safety_fault_reason,
        "safety_fault_time_s": safety_fault_time_s,
        "lidar_valid_ratio": float(np.mean(valid_ratios or [0.0])),
        "localizer_failure_rate": float(localizer_failures / max(len(valid_ratios), 1)),
        "sensor_messages_dropped": sensor_channel.statistics.dropped,
        "control_messages_dropped": control_channel.statistics.dropped,
        "network_reordered": (
            sensor_channel.statistics.reordered + control_channel.statistics.reordered
        ),
        "circle_radius_final_error_m": radius_error,
        "circle_phase_error_rad": phase_error,
        "course_departure": int(max(position_errors) > float(
            campaign["evaluation"]["course_error_limit_m"]
        )),
    })
    if output_dir is not None and trajectory:
        write_rows(Path(output_dir) / "trajectories" / f"{summary['run_id']}.csv", trajectory)
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--campaign-config", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("experiments/real_vehicle/results/simulation_campaign"),
    )
    parser.add_argument("--max-runs", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--seeds", default="")
    parser.add_argument("--controllers", default="")
    parser.add_argument("--scenarios", default="")
    parser.add_argument("--pose-sources", default="")
    parser.add_argument("--plant-profiles", default="")
    parser.add_argument("--lidar-profiles", default="")
    parser.add_argument("--network-profiles", default="")
    args = parser.parse_args()
    campaign = load_campaign(args.campaign_config)
    config, parameters, limits = load_real_config(args.vehicle_config)
    cases = expand_cases(campaign)
    filters = {
        "seed": args.seeds,
        "controller": args.controllers,
        "scenario": args.scenarios,
        "pose_source": args.pose_sources,
        "plant_profile": args.plant_profiles,
        "lidar_profile": args.lidar_profiles,
        "network_profile": args.network_profiles,
    }
    for field, raw_values in filters.items():
        if raw_values:
            allowed = {value.strip() for value in raw_values.split(",")}
            cases = [case for case in cases if str(case[field]) in allowed]
    if args.smoke:
        cases = [
            case for case in cases
            if case["seed"] == campaign["campaign"]["seeds"][0]
            and case["scenario"] == "straight"
            and case["network_profile"] in ("normal", "outage")
            and case["lidar_profile"] == "ideal"
            and case["plant_profile"] == "nominal"
        ]
        args.max_steps = args.max_steps or 250
    if args.max_runs > 0:
        cases = cases[:args.max_runs]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    started = time.perf_counter()
    for index, case in enumerate(cases, 1):
        summary = run_case(
            case,
            config,
            campaign,
            parameters,
            limits,
            output_dir=args.output_dir,
            max_steps=args.max_steps,
        )
        results.append(summary)
        print(
            f"[{index}/{len(cases)}] {summary['run_id']}: "
            f"completed={summary['completed']}, "
            f"rmse={summary['position_rmse_m']:.4f} m, "
            f"time={summary['controller_mean_ms']:.3f} ms"
        )
    raw_path = args.output_dir / "runs.csv"
    fields = sorted({key for row in results for key in row})
    with raw_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
    metadata = {
        "campaign_name": campaign["campaign"]["name"],
        "run_count": len(results),
        "wall_time_s": time.perf_counter() - started,
        "vehicle_config": str(args.vehicle_config),
        "campaign_config": str(args.campaign_config),
        "smoke": args.smoke,
        "max_steps": args.max_steps,
    }
    (args.output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )
    from .simulation_report import generate_report

    report_path = generate_report(raw_path, args.output_dir)
    print(f"runs_csv: {raw_path}")
    print(f"report: {report_path}")


if __name__ == "__main__":
    main()
