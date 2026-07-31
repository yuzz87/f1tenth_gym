import json
import tempfile
import unittest
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.compare_laserscan_recording import (  # noqa: E402
    check_geometry,
    compare_recordings,
)
from experiments.ros2.record_laserscan import save_recording  # noqa: E402


class LidarStep8Test(unittest.TestCase):
    def test_recording_metadata_and_distribution_comparison(self):
        messages = []
        for index in range(3):
            ranges = np.full(360, 12.0, dtype=float)
            ranges[10] = 1.0 + index * 0.01
            messages.append(
                {
                    "stamp": index * 0.2,
                    "frame_id": "laser",
                    "angle_min": -np.pi,
                    "angle_max": np.pi,
                    "angle_increment": 2.0 * np.pi / 359.0,
                    "time_increment": 0.0,
                    "scan_time": 0.2,
                    "range_min": 0.15,
                    "range_max": 12.0,
                    "ranges": ranges,
                    "valid_count": 360,
                }
            )

        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = save_recording(messages, temp_dir)
            summary = json.loads(
                (Path(output_dir) / "laserscan_summary.json").read_text(
                    encoding="utf-8"
                )
            )
            errors = check_geometry(
                summary,
                {"beams": 360, "range_min": 0.15, "range_max": 12.0},
            )
            self.assertEqual(errors, [])

            simulation_path = Path(temp_dir) / "simulation.npz"
            np.savez_compressed(
                simulation_path,
                scans=np.stack([message["ranges"] for message in messages]),
            )
            comparison = compare_recordings(
                Path(output_dir) / "laserscan_recording.npz",
                simulation_path,
            )
            self.assertTrue(comparison["beam_count_match"])
            self.assertAlmostEqual(comparison["mean_difference"], 0.0)


if __name__ == "__main__":
    unittest.main()
