import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from f110_gym.envs.laser_models import ScanSimulator2D
from f110_gym.envs.lidar_config import get_scan_angles, resolve_lidar_config


class LidarStep2Test(unittest.TestCase):
    """Step 2の理想LiDARと単純な矩形マップを検証する。"""

    def _create_simulator(self, close_obstacle=False):
        # 0.1 m/cell の正方形マップを作る。
        map_image = np.full((120, 120), 255, dtype=np.uint8)
        map_image[0, :] = 0
        map_image[-1, :] = 0
        map_image[:, 0] = 0
        map_image[:, -1] = 0

        if close_obstacle:
            # LiDARから0.1 mの障害物を置き、range_minのクリップを確認する。
            map_image[60, 61] = 0

        temp_dir = tempfile.TemporaryDirectory()
        temp_path = Path(temp_dir.name)
        image_path = temp_path / "room.png"
        yaml_path = temp_path / "room.yaml"
        Image.fromarray(map_image).save(image_path)
        yaml_path.write_text(
            "resolution: 0.1\n"
            "origin: [0.0, 0.0, 0.0]\n",
            encoding="utf-8",
        )

        config = resolve_lidar_config("rplidar_a1m8_r6")
        simulator = ScanSimulator2D(
            config["num_beams"],
            config["fov"],
            angle_min=config["angle_min"],
            range_min=config["range_min"],
            max_range=config["range_max"],
        )
        simulator.set_map(str(yaml_path), ".png")
        return temp_dir, config, simulator

    def test_scan_angles_match_a1_config(self):
        temp_dir, config, simulator = self._create_simulator()
        self.addCleanup(temp_dir.cleanup)

        np.testing.assert_allclose(
            simulator.scan_angles,
            get_scan_angles(config),
        )
        self.assertEqual(simulator.scan_angles.shape, (360,))
        self.assertAlmostEqual(simulator.scan_angles[0], -np.pi)
        self.assertAlmostEqual(simulator.scan_angles[-1], np.pi)

    def test_rectangular_walls_have_expected_distances(self):
        temp_dir, _, simulator = self._create_simulator()
        self.addCleanup(temp_dir.cleanup)

        # マップ中央から、ノイズなしで壁までの距離を測る。
        scan = simulator.scan(np.array([6.0, 6.0, 0.0]), rng=None)

        self.assertEqual(scan.shape, (360,))
        self.assertTrue(np.all(scan >= 0.15))
        self.assertTrue(np.all(scan <= 12.0))
        # -pi と pi は同じ向きなので、両端の距離は一致する。
        self.assertAlmostEqual(scan[0], scan[-1], places=6)
        # 0 rad付近のビームは、右側の壁まで約5.9 mになる。
        self.assertAlmostEqual(scan[179], 5.9, delta=0.15)
        self.assertAlmostEqual(scan[180], 5.9, delta=0.15)

    def test_range_min_is_applied_to_close_obstacles(self):
        temp_dir, _, simulator = self._create_simulator(close_obstacle=True)
        self.addCleanup(temp_dir.cleanup)

        scan = simulator.scan(np.array([6.0, 6.0, 0.0]), rng=None)

        self.assertAlmostEqual(np.min(scan), 0.15)


if __name__ == "__main__":
    unittest.main()
