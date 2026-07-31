import unittest
from argparse import Namespace
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.localization.map_localizer import BruteForceMapLocalizer
from f110_gym.envs.f110_env import F110Env


MAP_PATH = "experiments/maps/homur_oval"
MAP_EXT = ".png"
INITIAL_POSE = np.array([3.0, 0.0, np.pi / 2.0])


class LidarStep4Test(unittest.TestCase):
    def make_env(self):
        return F110Env(
            map=MAP_PATH,
            map_ext=MAP_EXT,
            num_agents=1,
            lidar_config="rplidar_a1m8_r6",
        )

    def make_localizer(self, scan_angles, **localizer_overrides):
        conf = Namespace(
            map_path=MAP_PATH,
            map_ext=MAP_EXT,
            timestep=0.01,
            car_params={"lf": 0.15875, "lr": 0.17145},
        )
        localizer_conf = {
            "type": "map_localizer",
            "scan_beams": 180,
            "xy_search_radius": 0.04,
            "theta_search_radius": 0.08,
            "xy_candidates": 5,
            "theta_candidates": 7,
            "preserve_scan_angles": True,
            "initialization_noise": [0.0, 0.0, 0.0],
        }
        localizer_conf.update(localizer_overrides)
        return BruteForceMapLocalizer(
            conf,
            localizer_conf,
            scan_angles,
            lidar_config="rplidar_a1m8_r6",
        )

    def test_known_pose_scan_matches_map_localizer(self):
        env = self.make_env()
        try:
            obs, _, _, _ = env.reset(INITIAL_POSE[None, :])
            localizer = self.make_localizer(env.sim.agents[0].scan_angles)
            estimated_pose = localizer.initialize(INITIAL_POSE)
            estimated_pose = localizer.update(obs)

            np.testing.assert_allclose(estimated_pose, INITIAL_POSE, atol=1e-12)
            self.assertEqual(estimated_pose.shape, (3,))
            self.assertTrue(np.isfinite(localizer.debug_info()["scan_error"]))
            self.assertLess(localizer.debug_info()["scan_error"], 0.01)
        finally:
            del env

    def test_initialization_error_is_refined_by_scan_matching(self):
        env = self.make_env()
        try:
            obs, _, _, _ = env.reset(INITIAL_POSE[None, :])
            localizer = self.make_localizer(
                env.sim.agents[0].scan_angles,
                initialization_noise=[0.02, -0.02, 0.04],
            )
            initial_pose = localizer.initialize(INITIAL_POSE)
            estimated_pose = localizer.update(obs)

            initial_error = np.linalg.norm(initial_pose[:2] - INITIAL_POSE[:2])
            estimated_error = np.linalg.norm(estimated_pose[:2] - INITIAL_POSE[:2])
            self.assertLessEqual(estimated_error, initial_error)
            self.assertLess(abs(estimated_pose[2] - INITIAL_POSE[2]), 0.08)
        finally:
            del env

    def test_scan_length_mismatch_is_rejected(self):
        env = self.make_env()
        try:
            obs, _, _, _ = env.reset(INITIAL_POSE[None, :])
            localizer = self.make_localizer(env.sim.agents[0].scan_angles)
            localizer.initialize(INITIAL_POSE)
            invalid_obs = dict(obs)
            invalid_obs["scans"] = [obs["scans"][0][:-1]]

            with self.assertRaises(ValueError):
                localizer.update(invalid_obs)
        finally:
            del env

    def test_scan_time_compensation_uses_motion_history(self):
        env = self.make_env()
        try:
            localizer = self.make_localizer(
                env.sim.agents[0].scan_angles,
                scan_time_compensation=True,
                scan_motion_history_compensation=True,
            )
            localizer.initialize(INITIAL_POSE)
            localizer._localizer_time = 0.03
            localizer._motion_history.extend(
                (
                    (0.01, 1.0, 0.0, 0.01),
                    (0.02, 2.0, 0.0, 0.01),
                    (0.03, 3.0, 0.0, 0.01),
                )
            )

            rewound = localizer._rewind_pose(
                np.array([0.30, 0.0, 0.0]),
                0.02,
                3.0,
                0.0,
            )

            # 0.03秒時点から0.01秒時点まで、速度3と2の区間を戻す。
            self.assertAlmostEqual(rewound[0], 0.25)
            self.assertAlmostEqual(rewound[1], 0.0)
        finally:
            del env


if __name__ == "__main__":
    unittest.main()
