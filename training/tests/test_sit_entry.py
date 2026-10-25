import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from butt_sit import SitPlant
from sit_reference import SEAT_DEG,SEAT_HEIGHT
from sit_training import HOME
from sit_entry import advance_walk_stop


class EntryPhysicsTests(unittest.TestCase):
    def test_prefix_clock_commands_and_state_survive_without_reset(self):
        # A constant seated policy isolates entry scheduling from gait quality;
        # the real MuJoCo plant still advances and settles under gravity.
        plant=SitPlant();plant.reset(17);sim=plant.sim;q=np.deg2rad(SEAT_DEG)
        try:
            sim.data.qpos[:3]=[0,0,SEAT_HEIGHT];sim.data.qpos[3:7]=[1,0,0,0]
            sim.data.qpos[sim.qadr]=q;sim.data.qvel.fill(0);plant.mj.mj_forward(sim.model,sim.data)
            sim.motor.reset(sim.data.qpos);sim.bam.actuator.q_target_smooth=q.copy()
            action=q-np.deg2rad(HOME);plant.previous=action.copy();plant.filtered=action.copy();plant.delay=[q.copy() for _ in range(4)]
            observations=[];times=[];resets=sim.reset_calls
            def predict(obs):observations.append(obs.copy());times.append(sim.data.time);return action
            elapsed=advance_walk_stop(plant,predict)
            self.assertAlmostEqual(elapsed,9.);self.assertEqual(len(observations),450)
            self.assertEqual(sim.reset_calls,resets);np.testing.assert_allclose(np.diff(times),.02,atol=1e-10)
            obs=np.asarray(observations);self.assertEqual(np.count_nonzero(obs[:,55:61]),0)
            np.testing.assert_allclose(obs[99,48:51],[0,0,0]);np.testing.assert_allclose(obs[200,48:51],[.2,0,0]);np.testing.assert_allclose(obs[-1,48:51],[0,0,0],atol=1e-7)
            self.assertLessEqual(float(np.max(np.abs(np.diff(obs[:,48:51],axis=0)))),.0200001)
            np.testing.assert_allclose(plant.previous,action,atol=1e-7);np.testing.assert_allclose(plant.filtered,action,atol=1e-7)
            self.assertGreater(plant.measure()['forces_n']['feet'],1.)
        finally:plant.adapter.close()

if __name__=='__main__':unittest.main()
