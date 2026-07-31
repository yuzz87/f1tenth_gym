import unittest
from pathlib import Path
import tempfile

import numpy as np

from experiments.real_vehicle.localization import (
    GridSearchLocalizer,
    OccupancyGridLidar,
    VirtualLidar,
)
from experiments.real_vehicle.integration_ws.src.real_vehicle_integration.real_vehicle_integration.real_map_localizer import (
    OccupancyMap,
)


class LidarTest(unittest.TestCase):
    def test_scan_contract_and_localization(self):
        lidar = VirtualLidar({"num_beams": 36, "range_max_m": 12.0})
        true_pose = np.array([0.1, 0.0, 0.0])
        scan = lidar.scan(true_pose, 0.0)
        self.assertEqual(len(scan["ranges"]), 36)
        self.assertAlmostEqual(scan["angle_min"], -np.pi)
        self.assertLessEqual(scan["ranges"].min(), 5.1)
        estimate, score = GridSearchLocalizer(lidar, xy_step_m=0.1, theta_step_rad=0.1).estimate(
            scan, [0.1, 0.0, 0.0]
        )
        self.assertTrue(np.isfinite(score))
        np.testing.assert_allclose(estimate, true_pose, atol=0.11)

    def test_obstacle_changes_scan_geometry(self):
        lidar = VirtualLidar({
            "num_beams": 72,
            "fov_rad": 2.0 * np.pi,
            "range_max_m": 12.0,
            "obstacles": ((1.0, 2.0, -0.5, 0.5),),
        })
        scan = lidar.scan([0.0, 0.0, 0.0])
        self.assertLess(float(np.min(scan["ranges"])), 2.1)

    def test_mount_offset_changes_ranges(self):
        base = VirtualLidar({"num_beams": 36, "x_offset_m": 0.0})
        offset = VirtualLidar({"num_beams": 36, "x_offset_m": 0.5})
        base_scan = base.scan([0.0, 0.0, 0.0])
        offset_scan = offset.scan([0.0, 0.0, 0.0])
        self.assertFalse(np.allclose(base_scan["ranges"], offset_scan["ranges"]))

    def test_burst_dropout_produces_consecutive_invalid_scans(self):
        lidar = VirtualLidar({
            "num_beams": 12,
            "burst_start_probability": 1.0,
            "burst_length_scans": 2,
        })
        first = lidar.scan([0.0, 0.0, 0.0], 0.0)
        second = lidar.scan([0.0, 0.0, 0.0], 0.1)
        self.assertTrue(first["burst_dropped"])
        self.assertTrue(second["burst_dropped"])
        self.assertEqual(first["valid_ratio"], 0.0)

    def test_localizer_reports_failure_with_too_few_beams(self):
        lidar = VirtualLidar({"num_beams": 12, "dropout_probability": 1.0})
        scan = lidar.scan([0.0, 0.0, 0.0])
        localizer = GridSearchLocalizer(lidar, minimum_valid_beams=4)
        estimate, score = localizer.estimate(scan, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(estimate, [0.0, 0.0, 0.0])
        self.assertFalse(localizer.last_success)
        self.assertTrue(np.isinf(score))

    def test_occupancy_grid_lidar_uses_saved_map_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pixels = []
            for row in range(20):
                pixels.append(" ".join(
                    "0" if row in (0, 19) or column in (0, 19) else "254"
                    for column in range(20)
                ))
            (root / "map.pgm").write_text(
                "P2\n20 20\n255\n" + "\n".join(pixels) + "\n",
                encoding="ascii",
            )
            (root / "map.yaml").write_text(
                "\n".join([
                    "image: map.pgm",
                    "resolution: 0.1",
                    "origin: [-1.0, -1.0, 0.0]",
                    "negate: 0",
                    "occupied_thresh: 0.65",
                    "free_thresh: 0.25",
                    "",
                ]),
                encoding="ascii",
            )
            occupancy_map = OccupancyMap(root / "map.yaml")
            lidar = OccupancyGridLidar(
                occupancy_map,
                {
                    "num_beams": 5,
                    "fov_rad": np.pi,
                    "range_min_m": 0.1,
                    "range_max_m": 3.0,
                    "x_offset_m": 0.0,
                },
            )
            scan = lidar.scan([0.0, 0.0, 0.0])
            self.assertEqual(len(scan["ranges"]), 5)
            self.assertGreater(scan["ranges"][2], 0.8)
            self.assertLess(scan["ranges"][2], 1.1)


if __name__ == "__main__":
    unittest.main()
