"""Offline environment contract tests; no model checkpoint, hardware or network."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from native_env import NativeEpisode,manifest

class Contract(unittest.TestCase):
 def test_evaluation_horizon_is_explicit_without_resets_or_physics_changes(self):
  env=NativeEpisode(manifest(),42,curriculum='stand',auto_reset=False,max_episode_steps=2)
  try:
   env.reset();env.command_ticks=10000
   _,_,done,timeout,_=env.step(np.zeros(14,dtype=np.float32))
   self.assertFalse(done);self.assertFalse(timeout)
   _,_,done,timeout,_=env.step(np.zeros(14,dtype=np.float32))
   self.assertTrue(done);self.assertTrue(timeout)
   self.assertEqual(env.total_resets,1)
   self.assertEqual(env.sim.physics_steps,8)
  finally:env.close()
 def test_ppo_overrides_do_not_change_fixed_physical_contract(self):
  import copy
  from train import config,check_contract
  cfg=config();override=copy.deepcopy(cfg)
  override['ppo']['actor']['distribution_cfg'].update(init_std=.05,std_type='log')
  override['ppo']['algorithm']['learning_rate']=1e-4
  self.assertEqual(check_contract(override),override)
  override['policy_hz']=100
  with self.assertRaisesRegex(ValueError,'fixed physical'):
   check_contract(override)
 def test_random_action_preserves_history_rate_and_body_frame_contract(self):
  env=NativeEpisode(manifest(),42,curriculum='stand')
  try:
   obs=env.observe();self.assertEqual(obs.shape,(61,));self.assertEqual(env.sim.extra_contacts,13)
   self.assertEqual(env.sim.model.opt.timestep,.005);self.assertEqual(env.sim.model.nu,14)
   raw=np.linspace(-.02,.02,14,dtype=np.float32);out,_,_,_,_=env.step(raw)
   np.testing.assert_allclose(out[34:48],raw,rtol=0,atol=1e-7)
   np.testing.assert_allclose(env.sim.policies[env.sim.task]._act_filt,.55*raw,rtol=0,atol=1e-7)
   self.assertEqual(env.sim.policy_steps,1);self.assertEqual(env.sim.physics_steps,4)
   self.assertEqual(env.sim.feedback_filter_samples,3)
   self.assertTrue(np.isfinite(env.critic_linear_velocity()).all())
  finally:env.close()
 def test_scratch_physics_has_no_parent_weight_dependency(self):
  m=manifest();self.assertFalse(m['neural_weights_in_asset_bindings']);self.assertTrue(m['weights_included'])
  for v in m['bindings'].values():self.assertNotIn(Path(v['path']).suffix,{'.pt','.onnx','.pth','.ckpt'})
 def test_all_six_float32_commands_can_be_set_without_resampling_physics(self):
  from native_env import COMMANDS
  for command in COMMANDS:
   env=NativeEpisode(manifest(),42,curriculum='stand')
   try:
    env.command=np.asarray(command,dtype=np.float32);env.motion_window.reset(env.command);env.command_ticks=10000
    env.command_index=COMMANDS.index(command);env.step(np.zeros(14,dtype=np.float32))
    self.assertEqual(env.sim.policy_steps,1)
   finally:env.close()
 def test_packaged47500_composite_loads_without_parent_and_matches_onnx(self):
  import torch,json
  from tensordict import TensorDict
  from refine_model import BackskipActor
  from refine import config,parity
  root=Path(__file__).resolve().parents[1];saved=torch.load(root/'checkpoints/model_047500.pt',map_location='cpu',weights_only=False)
  c=config()['ppo']['actor'];c.pop('class_name');actor=BackskipActor(TensorDict({'actor':torch.zeros(1,61)},batch_size=[1]),{'actor':['actor']},'actor',14,**c)
  actor.load_state_dict(saved['actor_state_dict'],strict=True)
  result=parity(actor,root/'checkpoints/model_047500.onnx');self.assertLess(result['max_abs_error'],2e-5)
  self.assertEqual(sum(p.numel() for p in actor.parameters() if p.requires_grad),868)
 def test_full47500_zero_command_has_base_gradient_and_fixed_sigma(self):
  import torch
  from tensordict import TensorDict
  from full_refine import Full47500Actor,config
  root=Path(__file__).resolve().parents[1]
  saved=torch.load(root/'checkpoints/model_047500.pt',map_location='cpu',weights_only=False)
  cfg=config()['ppo']['actor'];cfg.pop('class_name')
  obs=TensorDict({'actor':torch.zeros(1,61)},batch_size=[1])
  actor=Full47500Actor(obs,{'actor':['actor']},'actor',14,**cfg)
  actor.load_state_dict(saved['actor_state_dict'],strict=True)
  actor(obs).sum().backward()
  self.assertGreater(sum(float(p.grad.abs().sum()) for p in actor.mlp.base.parameters() if p.grad is not None),0.)
  self.assertTrue(all(not p.requires_grad for p in actor.distribution.parameters()))
  self.assertEqual(actor.obs_normalizer.until,0)
 def test_zero_command_correction_learns_without_changing_original_moving_outputs(self):
  import torch,copy
  from tensordict import TensorDict
  from standstop import StandstopActor,config
  from refine_model import BackskipActor
  root=Path(__file__).resolve().parents[1]
  saved=torch.load(root/'checkpoints/model_047500.pt',map_location='cpu',weights_only=False)
  cfg=config()['ppo']['actor'];cfg.pop('class_name');torch.manual_seed(47500)
  values=torch.randn(12,61)*.25;values[:,48:51]=0.
  obs=TensorDict({'actor':values},batch_size=[12])
  actor=StandstopActor(obs,{'actor':['actor']},'actor',14,**copy.deepcopy(cfg))
  original=BackskipActor(obs,{'actor':['actor']},'actor',14,**copy.deepcopy(cfg))
  original.load_state_dict(saved['actor_state_dict'],strict=True)
  load=actor.load_state_dict(saved['actor_state_dict'],strict=False)
  self.assertTrue(all(k.startswith('mlp.zero_skip.') for k in load.missing_keys));self.assertFalse(load.unexpected_keys)
  self.assertTrue(torch.equal(actor(obs),original(obs)))
  actor(obs).sum().backward()
  self.assertGreater(sum(float(p.grad.abs().sum()) for p in actor.mlp.zero_skip.parameters() if p.grad is not None),0.)
  self.assertEqual(sum(p.numel() for p in actor.parameters() if p.requires_grad),4878)
  with torch.no_grad():actor.mlp.zero_skip[-1].bias.fill_(.5)
  self.assertGreater(float((actor(obs)-original(obs)).detach().abs().max()),.04)
  for command in ([.2,0.,0.],[.3,0.,0.],[-.3,0.,0.],[0.,0.,1.],[0.,0.,-1.]):
   values[:,48:51]=torch.tensor(command)
   self.assertTrue(torch.equal(actor(obs),original(obs)))
 def test_dynamic_brake_preserves_quiet_standing_and_nonzero_commands(self):
  import torch,copy
  from tensordict import TensorDict
  from standstop import StandstopActor,config
  from brake_model import BrakeActor
  cfg=config()['ppo']['actor'];cfg.pop('class_name')
  values=torch.zeros(2,61);values[:,5]=-1.
  obs=TensorDict({'actor':values},batch_size=[2])
  standing=StandstopActor(obs,{'actor':['actor']},'actor',14,**copy.deepcopy(cfg))
  source=torch.load(Path(__file__).resolve().parents[1]/'checkpoints/model_047500.pt',map_location='cpu',weights_only=False)
  standing.load_state_dict(source['actor_state_dict'],strict=False)
  with torch.no_grad():standing.mlp.zero_skip[-1].bias.fill_(.2)
  actor=BrakeActor(obs,{'actor':['actor']},'actor',14,**copy.deepcopy(cfg))
  load=actor.load_state_dict(standing.state_dict(),strict=False)
  self.assertFalse(load.unexpected_keys);actor.mlp.set_statistics(actor.obs_normalizer)
  with torch.no_grad():actor.mlp.brake_skip[-1].bias.fill_(.5)
  self.assertTrue(torch.equal(actor(obs),standing(obs)))
  values[:,1]=.6
  difference=(actor(obs)-standing(obs)).detach().abs().max()
  self.assertGreater(float(difference),.06)
  actor(obs).sum().backward()
  self.assertGreater(sum(float(p.grad.abs().sum()) for p in actor.mlp.brake_skip.parameters() if p.grad is not None),0.)
  self.assertTrue(all(not p.requires_grad for p in actor.mlp.zero_skip.parameters()))
  values[:,48]=-.3
  self.assertTrue(torch.equal(actor(obs),standing(obs)))
if __name__=='__main__':unittest.main()
