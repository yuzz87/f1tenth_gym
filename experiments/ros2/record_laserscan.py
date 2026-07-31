"""ROS2 LaserScanを保存するStep 8用の収集ノード。"""

import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
except ImportError:  # ROS2がない開発環境でもimportとテストを可能にする。
    rclpy = None
    LaserScan = None

    class Node:
        pass


class LaserScanRecorder(Node):
    def __init__(self, topic, samples, duration):
        if rclpy is None:
            raise RuntimeError("ROS2 rclpy is required to record /scan")
        super().__init__("f110_laserscan_recorder")
        self.samples_target = int(samples)
        self.duration = float(duration)
        self.started_at = time.monotonic()
        self.messages = []
        self.subscription = self.create_subscription(
            LaserScan,
            topic,
            self._callback,
            10,
        )

    def _callback(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        ranges = np.asarray(message.ranges, dtype=np.float64)
        finite = np.isfinite(ranges)
        valid = finite & (ranges >= float(message.range_min))
        valid &= ranges <= float(message.range_max)
        self.messages.append(
            {
                "stamp": float(stamp),
                "frame_id": message.header.frame_id,
                "angle_min": float(message.angle_min),
                "angle_max": float(message.angle_max),
                "angle_increment": float(message.angle_increment),
                "time_increment": float(message.time_increment),
                "scan_time": float(message.scan_time),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "ranges": ranges,
                "valid_count": int(np.sum(valid)),
            }
        )

    def finished(self):
        if self.samples_target > 0 and len(self.messages) >= self.samples_target:
            return True
        return self.duration > 0.0 and time.monotonic() - self.started_at >= self.duration


def _pad_ranges(messages):
    max_beams = max((message["ranges"].size for message in messages), default=0)
    ranges = np.full((len(messages), max_beams), np.nan, dtype=np.float64)
    beam_counts = np.zeros(len(messages), dtype=np.int32)
    valid_counts = np.zeros(len(messages), dtype=np.int32)
    for index, message in enumerate(messages):
        beam_count = message["ranges"].size
        ranges[index, :beam_count] = message["ranges"]
        beam_counts[index] = beam_count
        valid_counts[index] = message["valid_count"]
    return ranges, beam_counts, valid_counts


def summarize_messages(messages):
    if not messages:
        raise ValueError("no LaserScan messages were recorded")
    stamps = np.asarray([message["stamp"] for message in messages], dtype=float)
    periods = np.diff(stamps)
    scan_times = np.asarray([message["scan_time"] for message in messages])
    return {
        "sample_count": len(messages),
        "frame_ids": sorted({message["frame_id"] for message in messages}),
        "beam_counts": sorted({int(message["ranges"].size) for message in messages}),
        "angle_min": messages[0]["angle_min"],
        "angle_max": messages[0]["angle_max"],
        "angle_increment": messages[0]["angle_increment"],
        "range_min": messages[0]["range_min"],
        "range_max": messages[0]["range_max"],
        "mean_update_hz": float(1.0 / np.mean(periods)) if periods.size else 0.0,
        "std_update_period_s": float(np.std(periods)) if periods.size else 0.0,
        "mean_scan_time_s": float(np.mean(scan_times)),
        "valid_ratio": float(
            np.sum([message["valid_count"] for message in messages])
            / max(np.sum([message["ranges"].size for message in messages]), 1)
        ),
    }


def save_recording(messages, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    ranges, beam_counts, valid_counts = _pad_ranges(messages)
    np.savez_compressed(
        output_dir / "laserscan_recording.npz",
        ranges=ranges,
        beam_counts=beam_counts,
        valid_counts=valid_counts,
        stamp=np.asarray([message["stamp"] for message in messages]),
    )

    metadata_path = output_dir / "laserscan_metadata.csv"
    fields = [
        "index",
        "stamp",
        "frame_id",
        "beam_count",
        "valid_count",
        "angle_min",
        "angle_max",
        "angle_increment",
        "time_increment",
        "scan_time",
        "range_min",
        "range_max",
    ]
    with metadata_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for index, message in enumerate(messages):
            writer.writerow(
                {
                    "index": index,
                    "stamp": message["stamp"],
                    "frame_id": message["frame_id"],
                    "beam_count": message["ranges"].size,
                    "valid_count": message["valid_count"],
                    "angle_min": message["angle_min"],
                    "angle_max": message["angle_max"],
                    "angle_increment": message["angle_increment"],
                    "time_increment": message["time_increment"],
                    "scan_time": message["scan_time"],
                    "range_min": message["range_min"],
                    "range_max": message["range_max"],
                }
            )

    summary = summarize_messages(messages)
    (output_dir / "laserscan_summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    return output_dir


def main():
    parser = argparse.ArgumentParser(description="ROS2 /scan LaserScan recorder")
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--output-dir", default="experiments/results/step8_laserscan")
    args = parser.parse_args()
    if args.samples <= 0 and args.duration <= 0.0:
        parser.error("--samples or --duration must be positive")
    if rclpy is None:
        raise SystemExit("ROS2 rclpy is not available; source ROS2 Foxy first")

    rclpy.init()
    recorder = LaserScanRecorder(args.topic, args.samples, args.duration)
    try:
        while rclpy.ok() and not recorder.finished():
            rclpy.spin_once(recorder, timeout_sec=0.1)
    finally:
        save_recording(recorder.messages, args.output_dir)
        recorder.destroy_node()
        rclpy.shutdown()
    print(f"recorded_samples: {len(recorder.messages)}")
    print(f"output_dir: {args.output_dir}")


if __name__ == "__main__":
    main()
