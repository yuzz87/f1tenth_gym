import math
import unittest  # test framework

import numpy as np

from f110_gym.envs.lidar_config import (
    get_lidar_extrinsics,
    get_scan_angles,
    resolve_lidar_config,
    transform_pose_to_lidar,
)


class LidarConfigTest(unittest.TestCase):
    def test_default_profile_preserves_legacy_values(self):
        config = resolve_lidar_config()

        self.assertEqual(config["profile"], "legacy")
        self.assertEqual(config["num_beams"], 1080)
        self.assertAlmostEqual(config["fov"], 4.7)
        self.assertAlmostEqual(config["range_max"], 30.0)

    def test_a1_profile_has_spec_based_interface_values(self):
        config = resolve_lidar_config("rplidar_a1m8_r6")
        angles = get_scan_angles(config)

        self.assertEqual(config["num_beams"], 360)
        self.assertAlmostEqual(config["fov"], 2.0 * math.pi)
        self.assertAlmostEqual(config["range_min"], 0.15)
        self.assertAlmostEqual(config["range_max"], 12.0)
        self.assertAlmostEqual(config["scan_rate_hz"], 5.5)
        self.assertEqual(angles.shape, (360,))
        self.assertAlmostEqual(angles[0], -math.pi)
        self.assertAlmostEqual(angles[-1], math.pi)
        self.assertAlmostEqual(
            angles[1] - angles[0], config["angle_increment"]
        )

    def test_mapping_overrides_profile_fields(self):
        config = resolve_lidar_config(
            {
                "profile": "rplidar_a1m8_r6",
                "num_beams": 180,
                "range_max": 8.0,
            }
        )

        self.assertEqual(config["num_beams"], 180)
        self.assertAlmostEqual(config["range_max"], 8.0)
        self.assertEqual(len(get_scan_angles(config)), 180)

    def test_invalid_profile_and_range_are_rejected(self):
        with self.assertRaises(ValueError):
            resolve_lidar_config("unknown")

        with self.assertRaises(ValueError):
            resolve_lidar_config({"range_min": 12.0, "range_max": 1.0})

        with self.assertRaises(ValueError):
            resolve_lidar_config({"dropout_probability": 1.1})

        with self.assertRaises(ValueError):
            resolve_lidar_config({"scan_delay": -0.1})

    def test_lidar_extrinsics_transform_pose(self):
        extrinsics = get_lidar_extrinsics(
            {"x_offset": 0.1, "y_offset": 0.05, "yaw_offset": 0.2}
        )
        pose = transform_pose_to_lidar(
            [1.0, 2.0, math.pi / 2.0], extrinsics
        )

        self.assertAlmostEqual(pose[0], 0.95)
        self.assertAlmostEqual(pose[1], 2.1)
        self.assertAlmostEqual(pose[2], math.pi / 2.0 + 0.2)

    def test_legacy_lidar_dist_is_used_when_x_offset_is_omitted(self):
        extrinsics = get_lidar_extrinsics({}, legacy_lidar_dist=0.2)

        self.assertAlmostEqual(extrinsics["x_offset"], 0.2)
        self.assertAlmostEqual(extrinsics["y_offset"], 0.0)


if __name__ == "__main__":
    unittest.main()
