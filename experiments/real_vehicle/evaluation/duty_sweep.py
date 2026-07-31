"""Check ESC duty safety boundaries without sending hardware output."""

import argparse
import csv
import json
from dataclasses import replace
from pathlib import Path

from ..adapters import CommandAdapter, esc_duty_to_speed
from .common import DEFAULT_CONFIG, load_real_config


DEFAULT_DUTIES = (10.30, 10.20, 10.16, 10.10, 10.00, 9.90, 9.70)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate duty boundaries in simulation only; no physical speed "
            "calibration or hardware output is used."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--duties",
        default=",".join(f"{value:.2f}" for value in DEFAULT_DUTIES),
        help="Duty values to inspect, in percent.",
    )
    parser.add_argument(
        "--virtual-speed-max-mps",
        type=float,
        default=None,
        help="Only for the reversible simulation duty envelope.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("experiments/real_vehicle/results/duty_sweep"),
    )
    args = parser.parse_args()

    config, _parameters, limits = load_real_config(args.config)
    if config["simulation"].get("hardware_output_enabled", False):
        raise ValueError("duty_sweep is simulation-only; hardware output must be disabled")
    esc_config = config.get("calibration", {}).get("esc", {})
    speed_max = float(args.virtual_speed_max_mps or limits.speed_max_mps)
    virtual_limits = replace(limits, speed_max_mps=speed_max)
    safe_adapter = CommandAdapter(
        virtual_limits,
        float(config["simulation"]["timestep_s"]),
        esc_config=esc_config,
        allow_experimental_duty=False,
    )
    experimental_adapter = CommandAdapter(
        virtual_limits,
        float(config["simulation"]["timestep_s"]),
        esc_config=esc_config,
        allow_experimental_duty=True,
    )

    rows = []
    steering_duty = 10.895
    duties = [float(value) for value in args.duties.split(",") if value.strip()]
    for requested_duty in duties:
        safe_duty, safe_clamped = safe_adapter.normalize_esc_duty(requested_duty)
        experimental_duty, experimental_clamped = experimental_adapter.normalize_esc_duty(
            requested_duty
        )
        safe_action = safe_adapter.duty_to_gym_command(steering_duty, requested_duty)
        experimental_action = experimental_adapter.duty_to_gym_command(
            steering_duty,
            requested_duty,
        )
        rows.append({
            "requested_duty_percent": requested_duty,
            "safe_applied_duty_percent": safe_duty,
            "experimental_applied_duty_percent": experimental_duty,
            "safe_clamped": int(safe_clamped),
            "experimental_clamped": int(experimental_clamped),
            "hardware_safe_without_clamp": int(not safe_clamped),
            "safe_virtual_speed_mps": float(safe_action[1]),
            "experimental_virtual_speed_mps": float(experimental_action[1]),
            "raw_virtual_speed_mps": float(esc_duty_to_speed(
                requested_duty,
                speed_command_max_mps=speed_max,
                stop_duty_percent=safe_adapter.esc_stop_duty,
                start_duty_percent=safe_adapter.esc_start_duty,
                forward_min_duty_percent=safe_adapter.esc_forward_min_duty,
                allow_experimental_duty=True,
            )),
            "virtual_speed_max_mps": speed_max,
            "hardware_output_enabled": int(
                config["simulation"].get("hardware_output_enabled", False)
            ),
        })

    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.output_dir / "duty_sweep.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "duties": rows,
        "purpose": "simulation_only_duty_boundary_check",
        "encoder_speed_used": False,
        "hardware_output_enabled": False,
        "safe_forward_min_duty_percent": safe_adapter.esc_forward_min_duty,
        "safe_start_duty_percent": safe_adapter.esc_start_duty,
        "safe_stop_duty_percent": safe_adapter.esc_stop_duty,
        "virtual_speed_max_mps": speed_max,
    }
    summary_path = args.output_dir / "duty_sweep.summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"encoder_speed_used: false")
    print(f"hardware_output_enabled: false")
    for row in rows:
        print(
            f"duty={row['requested_duty_percent']:.2f}% -> "
            f"safe={row['safe_applied_duty_percent']:.2f}%, "
            f"experimental={row['experimental_applied_duty_percent']:.2f}%"
        )
    print(f"raw_csv: {raw_path}")
    print(f"summary_json: {summary_path}")


if __name__ == "__main__":
    main()
