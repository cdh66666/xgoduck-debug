import os
import unittest
from unittest.mock import patch
import numpy as np
from native_env import NativeEpisode, manifest
from runtime_profile import HOME_DEG, POLICY_INDEX, CALIBRATED
from train import config

class RuntimeProfileTests(unittest.TestCase):
    def test_deployed_home_obs_filter_and_50hz_are_the_same_contract(self):
        with patch.dict(os.environ, {'XGODUCK_RUNTIME_PROFILE': CALIBRATED}):
            frozen = manifest()
            cfg = config()
        env = NativeEpisode(frozen, 11, curriculum='stand', auto_reset=False)
        try:
            policy = env.sim.policies['scratch']
            np.testing.assert_allclose(np.rad2deg(cfg['home_rad']), HOME_DEG, atol=1e-5)
            np.testing.assert_allclose(np.rad2deg(env.sim.home), HOME_DEG, atol=1e-5)
            home = np.zeros(15, dtype=np.float32)
            home[POLICY_INDEX] = HOME_DEG
            obs = policy.build_obs([0,0,-1], [0,0,0], home, home*0, [0,0,0])
            np.testing.assert_array_equal(obs[6:20], np.zeros(14))
            np.testing.assert_allclose(policy.action_to_deg(np.zeros(14)), home)
            env.sessions['scratch'].action[:] = 1
            action = policy.infer([0,0,-1], [0,0,0], home, home*0, [0,0,0])
            np.testing.assert_allclose(action[[0,1,2,3,4,9,10,11,12,13]], .55)
            np.testing.assert_allclose(action[5:9], .8)
            policy.reset_action_filter()
            before = env.sessions['scratch'].calls
            for _ in range(10):
                env.step(np.zeros(14, dtype=np.float32))
            self.assertEqual(env.sessions['scratch'].calls-before, 10)
            self.assertAlmostEqual(env.sim.data.time, .2)
            self.assertEqual(env.sim.feedback_filter_samples, 21)
        finally:
            env.close()

    def test_speed_course_can_reach_one_without_changing_other_commands(self):
        with patch.dict(os.environ, {'XGODUCK_MAX_FORWARD': '1.0'}):
            env = NativeEpisode(manifest(), 15, curriculum='speed')
            try:
                seen = []
                for _ in range(200):
                    env._resample()
                    if env.command_index == 2:
                        seen.append(float(env.command[0]))
                    elif env.command_index == 1:
                        self.assertAlmostEqual(float(env.command[0]), .2)
                self.assertGreater(max(seen), .97)
                self.assertGreaterEqual(min(seen), .7)
            finally:
                env.close()

if __name__ == '__main__': unittest.main()
