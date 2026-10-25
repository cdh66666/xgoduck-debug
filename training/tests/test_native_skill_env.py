import os
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from butt_sit import SitPlant,XML
from native_env import manifest
from native_skill_env import NativeSkillEpisode
from evaluate_skill_chain import predictor

class NativeSkillTests(unittest.TestCase):
    def test_phase_distinguishes_start_from_end_without_changing_50hz(self):
        with patch.dict(os.environ,{'XGODUCK_TRAIN_XML':str(XML),'XGODUCK_RUNTIME_PROFILE':'calibrated-20261025','XGODUCK_RANDOMIZE_PHYSICS':'0'}):frozen=manifest()
        env=NativeSkillEpisode(frozen,17,curriculum='roll_phase',auto_reset=False)
        try:
            self.assertEqual(env.observe()[55],0.)
            env.step(np.zeros(14,dtype=np.float32))
            self.assertAlmostEqual(float(env.observe()[55]),1/95)
            self.assertAlmostEqual(env.sim.data.time,.02)
        finally:env.close()

    def test_single_roll_progress_does_not_reward_an_extra_turn(self):
        import math
        self.assertGreater(NativeSkillEpisode.single_roll_progress(1.,1.1),0)
        self.assertLess(NativeSkillEpisode.single_roll_progress(2*math.pi,2*math.pi+.1),0)
        self.assertLess(NativeSkillEpisode.single_roll_progress(4*math.pi,4*math.pi+.1),0)

    def test_roll_can_cross_inverted_attitude_without_walking_recovery_or_reset(self):
        with patch.dict(os.environ,{'XGODUCK_TRAIN_XML':str(XML),'XGODUCK_RUNTIME_PROFILE':'calibrated-20261025','XGODUCK_RANDOMIZE_PHYSICS':'0'}):frozen=manifest()
        env=NativeSkillEpisode(frozen,41,auto_reset=False);policy=predictor(Path(__file__).resolve().parents[2]/'robot-app/python/xgoduck_roulade.onnx')
        count=env.total_resets;tilts=[]
        try:
            for tick in range(95):
                _,_,done,timeout,info=env.step(policy(env.observe()));tilts.append(info['tilt'])
                self.assertEqual(done,tick==94);self.assertFalse(timeout)
            self.assertGreater(max(tilts),150)
            self.assertEqual(env.total_resets,count);self.assertAlmostEqual(env.sim.data.time,1.9)
            self.assertEqual(info['contact_sampling_hz'],200);self.assertGreater(info['head_peak_n'],0)
        finally:env.close()

    def test_requested_torque_is_separate_from_actual_force_limit(self):
        plant=SitPlant();plant.reset(7)
        # A deliberately low test-only force cap guarantees saturation; the
        # normal 1Nm plant does not saturate for every static target.
        plant.sim.model.actuator_forcerange[:]=[-.1,.1]
        for _ in range(6):plant.advance(np.full(14,4.,dtype=np.float32))
        bound=float(np.max(np.abs(plant.sim.model.actuator_forcerange)))
        self.assertLessEqual(plant.peak_torque_nm,bound+1e-8)
        self.assertGreater(plant.peak_command_torque_nm,plant.peak_torque_nm)

if __name__=='__main__':unittest.main()
