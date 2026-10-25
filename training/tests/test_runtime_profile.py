import os
import unittest
from unittest.mock import patch
import numpy as np
from native_env import NativeEpisode, manifest
from runtime_profile import HOME_DEG, POLICY_INDEX, CALIBRATED
from train import config

class RuntimeProfileTests(unittest.TestCase):
    def test_target_rate_cost_changes_reward_not_physics_or_clock(self):
        with patch.dict(os.environ, {'XGODUCK_RUNTIME_PROFILE': CALIBRATED,'XGODUCK_TARGET_RATE_COST':'0'}):
            baseline=NativeEpisode(manifest(),91,curriculum='stand',auto_reset=False)
        with patch.dict(os.environ, {'XGODUCK_RUNTIME_PROFILE': CALIBRATED,'XGODUCK_TARGET_RATE_COST':'1'}):
            smooth=NativeEpisode(manifest(),91,curriculum='stand',auto_reset=False)
        try:
            action=np.full(14,.05,dtype=np.float32)
            _,r0,_,_,i0=baseline.step(action);_,r1,_,_,i1=smooth.step(action)
            np.testing.assert_array_equal(baseline.sim.data.qpos,smooth.sim.data.qpos)
            self.assertEqual(baseline.sim.data.time,smooth.sim.data.time)
            self.assertLess(i1['reward_terms']['filtered_target_rate'],0)
            self.assertAlmostEqual(r1-r0,.02*i1['reward_terms']['filtered_target_rate'])
        finally:baseline.close();smooth.close()

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

    def test_command_ramp_matches_observed_command_and_keeps_50hz(self):
        from unittest.mock import Mock
        env=NativeEpisode(manifest(),15,curriculum='ramped',auto_reset=False)
        try:
            env.reset();env.command=np.zeros(3,dtype=np.float32)
            env.rng=Mock(wraps=env.rng);env.rng.choice.return_value=1
            env._resample();before=env.sim.data.time
            for _ in range(15):
                requested=env.command.copy();obs=env.observe()
                np.testing.assert_array_equal(obs[48:51],requested)
                _,_,done,_,info=env.step(np.zeros(14,dtype=np.float32))
                np.testing.assert_allclose(info['command'],requested)
                np.testing.assert_allclose(env.sessions[env.sim.task].last_obs[0,48:51],requested)
                self.assertLessEqual(float(np.max(np.abs(env.command-requested))),.020001)
                self.assertFalse(done)
            self.assertAlmostEqual(float(env.command[0]),.2)
            self.assertGreater(env.command_age_ticks,0)
            self.assertLess(env.command_age_ticks,15)
            self.assertAlmostEqual(env.sim.data.time-before,.3)
            self.assertEqual(env.total_resets,1)
        finally:env.close()

if __name__ == '__main__': unittest.main()
