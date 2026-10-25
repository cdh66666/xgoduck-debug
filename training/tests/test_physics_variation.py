import os,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from native_env import NativeEpisode,manifest

class PhysicsVariationTests(unittest.TestCase):
    def test_randomized_plant_keeps_mass_clock_and_repeatability(self):
        xml=Path(__file__).resolve().parents[1]/'model/robot_inertial_candidate_20261025.xml'
        with patch.dict(os.environ,{'XGODUCK_TRAIN_XML':str(xml),'XGODUCK_RUNTIME_PROFILE':'calibrated-20261025','XGODUCK_RANDOMIZE_PHYSICS':'1'}):
            frozen=manifest()
        env=NativeEpisode(frozen,31,curriculum='stand');other=NativeEpisode(frozen,31,curriculum='stand')
        try:
            env.reset();other.reset()
            self.assertEqual(env.variation.sample,other.variation.sample)
            np.testing.assert_allclose(env.sim.model.body_ipos,other.sim.model.body_ipos)
            self.assertAlmostEqual(float(env.sim.model.body_mass.sum()),.826)
            sample=env.variation.sample
            self.assertEqual(len(env.sim.delay_queue),sample['transport_delay_physics_steps'])
            self.assertTrue(7.2<=sample['voltage_v']<=8.1)
            self.assertTrue(.55<=sample['floor_friction']<=1)
            before=env.sessions['scratch'].calls
            for _ in range(5):env.step(np.zeros(14,dtype=np.float32))
            self.assertEqual(env.sessions['scratch'].calls-before,5)
            self.assertAlmostEqual(env.sim.data.time,.1)
            env.reset();self.assertNotEqual(sample,env.variation.sample)
        finally:env.close();other.close()

if __name__=='__main__':unittest.main()
