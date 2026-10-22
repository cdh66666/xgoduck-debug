"""Offline environment contract tests; no model checkpoint, hardware or network."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from native_env import NativeEpisode,manifest

class Contract(unittest.TestCase):
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
if __name__=='__main__':unittest.main()
