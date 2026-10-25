import os,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import torch
from tensordict import TensorDict
from butt_sit import XML
from sit_training import HOME
from sit_reference import SIT_STEPS,SIT_DURATION,RISE_DURATION,SEAT_HEIGHT,SEAT_DEG
from native_env import manifest
from native_sit_env import NativeSitEpisode
from sit_refine_model import SitActor
from sit_refine import initialize


class NativeSitTests(unittest.TestCase):
    def episode(self):
        with patch.dict(os.environ,{'XGODUCK_TRAIN_XML':str(XML),'XGODUCK_RUNTIME_PROFILE':'calibrated-20261025','XGODUCK_RANDOMIZE_PHYSICS':'0'}):frozen=manifest()
        return NativeSitEpisode(frozen,17,auto_reset=False)

    def test_sit_rise_phase_boundary_preserves_clock_and_state(self):
        env=self.episode()
        try:
            self.assertEqual(env.observe()[55],0);self.assertEqual(env.observe()[56],1)
            env.steps=SIT_STEPS-1;before=env.sim.data.time;count=env.total_resets
            obs,_,done,_,info=env.step(env.plant.previous.copy())
            self.assertFalse(done);self.assertEqual(obs[55],0);self.assertEqual(obs[56],-1)
            self.assertEqual(env.total_resets,count);self.assertAlmostEqual(env.sim.data.time-before,.02)
            self.assertEqual(info['contact_sampling_hz'],200)
        finally:env.close()

    def test_real_seated_support_qualifies_without_reset(self):
        env=self.episode();s=env.sim;p=env.plant
        try:
            q=np.deg2rad(SEAT_DEG);s.data.qpos[:3]=[0,0,SEAT_HEIGHT];s.data.qpos[3:7]=[1,0,0,0]
            s.data.qpos[s.qadr]=q;s.data.qvel.fill(0);p.mj.mj_forward(s.model,s.data);s.motor.reset(s.data.qpos)
            s.bam.actuator.q_target_smooth=q.copy();s._feedback_filter_time=None
            p.previous=(q-np.deg2rad(HOME)).astype(np.float32);p.filtered=p.previous.copy();p.delay=[q.copy() for _ in range(4)]
            env.previous_targets=p.filtered.copy();env.steps=SIT_STEPS-100
            count=env.total_resets
            for _ in range(100):_,_,_,_,info=env.step(q-np.deg2rad(HOME))
            self.assertTrue(info['seat_achieved']);self.assertGreater(info['seat_support_n'],1)
            self.assertEqual(env.total_resets,count);self.assertAlmostEqual(s.data.time,2.)
        finally:env.close()

    def test_full_actor_import_export_and_trainability(self):
        obs=TensorDict({'actor':torch.randn(3,61)},batch_size=[3])
        actor=SitActor(obs,{'actor':['actor']},'actor',14,hidden_dims=[128,128],activation='tanh',obs_normalization=False,
            distribution_cfg={'class_name':'GaussianDistribution','init_std':.005,'std_type':'scalar'})
        source={'features':actor.obs_normalizer.features.clone(),'mean':torch.ones(5)*.1,'std':torch.ones(5)*.7}
        for key,value in actor.mlp.state_dict().items():source['net.'+key]=value.clone()
        cfg=dict(physics_sha256='a'*64,role='official-reference-sit-rise-candidate',phase_index=55,direction_index=56,
            sit_duration_s=SIT_DURATION,rise_duration_s=RISE_DURATION,home_deg=HOME.tolist())
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'candidate.pt';torch.save({'config':cfg,'state_dict':source},path)
            with patch('native_env.manifest',return_value={'physical_model':{'sha256':'a'*64}}):report=initialize(SimpleNamespace(actor=actor),path)
            from train import export_policy
            from sit_refine import config,KIND
            import onnx
            model=Path(folder)/'actor.onnx'
            exported=export_policy(actor,model,config(),architecture=KIND,physical_model={'sha256':'a'*64})
            meta={p.key:p.value for p in onnx.load(str(model)).metadata_props}
            self.assertEqual(meta['role'],'official-reference-sit-rise-candidate')
            self.assertEqual(meta['phase_index'],'55');self.assertEqual(meta['direction_index'],'56')
            self.assertEqual(meta['physical_model_sha256'],'a'*64);self.assertLess(exported['max_abs_error'],2e-5)
        self.assertLess(report['initial_actor_parity_error'],2e-5);self.assertEqual(report['trainable_actor_parameters'],19086)
        torch.testing.assert_close(actor(obs),actor.as_onnx(False)(obs['actor']))
        actor(obs).sum().backward()
        self.assertTrue(all(p.requires_grad and p.grad is not None for p in actor.mlp.parameters()))
        self.assertTrue(all(not p.requires_grad for p in actor.distribution.parameters()))


if __name__=='__main__':unittest.main()
