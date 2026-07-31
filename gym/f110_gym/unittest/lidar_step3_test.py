import unittest

import numpy as np

from f110_gym.envs.f110_env import F110Env
from f110_gym.envs.lidar_config import resolve_lidar_config


class LidarStep3Test(unittest.TestCase):
    def make_env(self, seed=123, **overrides):
        config = {
            "profile": "rplidar_a1m8_r6",
            "noise_std": 0.0,
            "noise_std_per_meter": 0.0,
            "dropout_probability": 0.0,
            "scan_delay": 0.0,
        }
        config.update(overrides)
        return F110Env(
            seed=seed,
            num_agents=1,
            timestep=0.01,
            lidar_config=config,
        )

    def reset_env(self, env):
        return env.reset(np.array([[0.0, 0.0, 0.0]]))[0]

    def test_noise_is_reproducible_with_same_seed(self):
        env_a = self.make_env(seed=7, noise_std=0.01)
        env_b = self.make_env(seed=7, noise_std=0.01)

        scan_a = self.reset_env(env_a)["scans"][0]
        scan_b = self.reset_env(env_b)["scans"][0]

        np.testing.assert_array_equal(scan_a, scan_b)

    def test_distance_dependent_noise_changes_the_measurement(self):
        env_without_distance_term = self.make_env(seed=7, noise_std=0.01)
        scan_a = self.reset_env(env_without_distance_term)["scans"][0]

        env_with_distance_term = self.make_env(
            seed=7,
            noise_std=0.01,
            noise_std_per_meter=0.01,
        )
        scan_b = self.reset_env(env_with_distance_term)["scans"][0]

        self.assertFalse(np.array_equal(scan_a, scan_b))

    def test_dropout_can_mark_all_measurements_invalid(self):
        env = self.make_env(dropout_probability=1.0)
        obs = self.reset_env(env)

        np.testing.assert_allclose(obs["scans"][0], 12.0)
        self.assertFalse(np.any(obs["scan_valid"][0]))

    def test_scan_valid_mask_is_true_without_dropout(self):
        env = self.make_env(dropout_probability=0.0)
        obs = self.reset_env(env)

        self.assertTrue(np.all(obs["scan_valid"][0]))

    def test_a1_scan_updates_at_sensor_rate(self):
        env = self.make_env()
        obs = self.reset_env(env)
        update_steps = []

        for step in range(1, 25):
            obs, _, _, _ = env.step(np.zeros((1, 2)))
            if obs["scan_updated"][0]:
                update_steps.append(step)

        # 5.5 Hzは0.01秒の車体更新より遅く、約0.18秒ごとに更新される。
        self.assertGreaterEqual(len(update_steps), 1)
        self.assertLessEqual(len(update_steps), 2)
        self.assertGreaterEqual(update_steps[0], 17)
        self.assertLessEqual(update_steps[0], 20)

    def test_scan_delay_keeps_previous_measurement_until_ready(self):
        env = self.make_env(scan_delay=0.05)
        obs = self.reset_env(env)
        initial_scan_time = obs["scan_times"][0]
        update_steps = []

        for step in range(1, 32):
            obs, _, _, _ = env.step(np.zeros((1, 2)))
            if obs["scan_updated"][0]:
                update_steps.append(step)

        self.assertEqual(initial_scan_time, 0.0)
        self.assertGreaterEqual(len(update_steps), 1)
        # 周期の約18ステップに遅延5ステップが加わる。
        self.assertGreaterEqual(update_steps[0], 22)
        self.assertLessEqual(update_steps[0], 25)
        self.assertGreater(obs["scan_ages"][0], 0.0)

    def test_new_step3_fields_have_valid_defaults(self):
        config = resolve_lidar_config("rplidar_a1m8_r6")

        self.assertEqual(config["noise_std"], 0.0)
        self.assertEqual(config["noise_std_per_meter"], 0.0)
        self.assertEqual(config["dropout_probability"], 0.0)
        self.assertEqual(config["scan_delay"], 0.0)


if __name__ == "__main__":
    unittest.main()
