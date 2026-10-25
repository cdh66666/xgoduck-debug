import os
import unittest
from unittest.mock import patch
import numpy as np
from butt_sit import SitPlant,INDEX,HOME
from search_butt_raise import rollout,raise_teacher
from train_butt_sit import teacher as sit_teacher

PARAMETERS=[46.70310170041174,76.40411983212645,70.33832955536383,-8.104151552506403,
            .6246970744434912,3.7318099657674444,.6842257690322426,.23755905051403006,
            -48.12318649688403,1.9329865677460158]

class SitCycleTests(unittest.TestCase):
    def test_measured_pose_matches_final_substep_and_torque_is_live(self):
        plant=SitPlant();plant.reset(5)
        for _ in range(5):plant.advance(plant.previous+.03)
        s=plant.sim;positions=s.data.geom_xpos.copy();rotations=s.data.xmat.copy()
        plant.mj.mj_forward(s.model,s.data)
        np.testing.assert_allclose(s.data.geom_xpos,positions,atol=1e-12)
        np.testing.assert_allclose(s.data.xmat,rotations,atol=1e-12)
        self.assertGreater(plant.peak_torque_nm,.01)
        self.assertGreaterEqual(plant.peak_torque_nm,float(np.max(np.abs(s.data.ctrl[s.motor.act_indexes]))))

    def test_teacher_sits_then_rises_without_reset_at_50hz(self):
        plant=SitPlant();calls=[]
        def predict(obs):
            t=len(calls)*.02;calls.append(t)
            return sit_teacher(obs,plant.start) if t<3 else raise_teacher(obs,plant.start,PARAMETERS,t-3)
        _,metrics,trace=rollout(plant,PARAMETERS,5000,variant=True,predict=predict)
        self.assertTrue(metrics['passed'],metrics['failures'])
        self.assertEqual(len(calls),437)
        self.assertAlmostEqual(plant.sim.data.time,len(calls)/50)
        self.assertEqual(plant.sim.feedback_filter_samples,2*len(calls))
        self.assertGreater(metrics['seated_butt_force_n'],1.)
        self.assertLess(metrics['butt_force_n'],.1)

    def test_early_failure_is_recorded_instead_of_empty_target_crash(self):
        plant=SitPlant();measure=plant.measure
        def bad_contact():
            row=measure();row['head_clearance_mm']=0.;return row
        plant.measure=bad_contact
        _,metrics,trace=rollout(plant,PARAMETERS)
        self.assertFalse(metrics['passed']);self.assertEqual(len(trace),1)
        self.assertIn('tilt_or_head_clearance',metrics['failures'])
        self.assertIn('did_not_sit_supported_before_rising',metrics['failures'])

    def test_sit_plant_matches_calibrated_policy_obs_and_filter(self):
        with patch.dict(os.environ,{'XGODUCK_RUNTIME_PROFILE':'calibrated-20261025'}):plant=SitPlant()
        plant.reset(12);obs,g=plant.observation(0,0);s=plant.sim;p=s.policies['scratch']
        p.prev_action[:]=plant.previous;p._act_filt[:]=plant.filtered
        angles=np.zeros(15,dtype=np.float32);vels=angles.copy()
        angles[INDEX]=np.rad2deg(s.data.qpos[s.qadr]);vels[INDEX]=np.rad2deg(s.filtered_velocity)
        expected=p.build_obs(g,s.filtered_gyro,angles,vels,[0,0,0])
        np.testing.assert_allclose(obs,expected,atol=1e-6)
        raw=np.linspace(-.2,.2,14,dtype=np.float32);p.session.action[:]=raw
        filtered=p.infer(g,s.filtered_gyro,angles,vels,[0,0,0]);plant.advance(raw)
        np.testing.assert_allclose(plant.filtered,filtered,atol=1e-7)

if __name__=='__main__':unittest.main()
