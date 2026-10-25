"""Train the full self-contained 47500 actor, including its zero-command base.

Original sigma and observation statistics stay fixed. The full-actor Adam is
new; the original ten-entry backskip Adam cannot initialize this larger scope.
"""
import hashlib
import json
from pathlib import Path
from train import ROOT,check_contract,config as scratch_config,main as train_main

KIND='xgoduck-full-47500-bodyframe-v2'
ACTOR_CLASS='full_refine:Full47500Actor'


def __getattr__(name):
 # Keep the saved actor-class path while Windows spawn imports this entry in
 # each physics worker. Only the parent/evaluator resolves the Torch model.
 if name=='Full47500Actor':
  from full_refine_model import Full47500Actor
  return Full47500Actor
 raise AttributeError(name)


def config(path=None):
 if path is not None:
  return check_contract(json.loads(Path(path).read_text('utf-8')),ACTOR_CLASS)
 cfg=scratch_config()
 cfg['ppo']['actor']['class_name']=ACTOR_CLASS
 cfg['ppo']['actor']['distribution_cfg']['init_std']=.02
 cfg['ppo']['algorithm'].update(learning_rate=1e-7,entropy_coef=0.,schedule='fixed')
 return check_contract(cfg,ACTOR_CLASS)


def initialize(alg,checkpoint=None):
 import torch
 from refine import parity
 source=Path(checkpoint) if checkpoint else ROOT/'checkpoints/model_047500.pt'
 saved=torch.load(source,map_location='cpu',weights_only=False)
 if saved.get('kind')==KIND:
  # Deliberate new optimization run: retain every learned actor/critic tensor
  # but start a fresh optimizer with this run's explicitly selected PPO config.
  state=saved['algorithm']
  alg.actor.load_state_dict(state['actor_state_dict'],strict=True)
  alg.critic.load_state_dict(state['critic_state_dict'],strict=True)
  for module,key in ((alg.actor,'actor_state_dict'),(alg.critic,'critic_state_dict')):
   for name,value in state[key].items():
    if not torch.equal(value,module.state_dict()[name]):raise ValueError('Transfer changed '+name)
  if alg.optimizer.state:raise ValueError('Transfer optimizer must start fresh')
  return {'initialization':'all own full-actor/critic tensors; new Adam with explicit run config',
   'source_checkpoint_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
   'source_iteration':int(saved['iteration']),'actor_load_exact':True,'critic_load_exact':True,
   'optimizer_reset_for_new_scope':True,'initial_optimizer_entries':0,
   'trainable_actor_parameters':sum(p.numel() for p in alg.actor.parameters() if p.requires_grad),
   'trainable_scope':'all actor weights + critic; original sigma/normalizer fixed'}
 if saved.get('kind')!='xgoduck-backskip-47500-bodyframe-v2':raise ValueError('Expected packaged47500 source')
 alg.actor.load_state_dict(saved['actor_state_dict'],strict=True)
 alg.critic.load_state_dict(saved['critic_state_dict'],strict=True)
 for key,value in saved['actor_state_dict'].items():
  if not torch.equal(value,alg.actor.state_dict()[key]):raise ValueError('Full actor initial load changed '+key)
 for key,value in saved['critic_state_dict'].items():
  if not torch.equal(value,alg.critic.state_dict()[key]):raise ValueError('Full critic initial load changed '+key)
 if alg.optimizer.state:raise ValueError('Full actor optimizer must start fresh')
 return {'initialization':'all packaged47500 actor/critic tensors; new full-actor Adam',
  'source_checkpoint_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
  'source_iteration':int(saved['iter']),'actor_load_exact':True,'critic_load_exact':True,
  'optimizer_reset_for_new_scope':True,'initial_optimizer_entries':0,
  'trainable_actor_parameters':sum(p.numel() for p in alg.actor.parameters() if p.requires_grad),
  'trainable_scope':'all base/residual/head/back_skip weights + Critic64; original sigma/normalizer fixed',
  'initial_onnx_parity':parity(alg.actor,ROOT/'checkpoints/model_047500.onnx')}


def verify_actor(before,actor):
 import torch
 after=actor.state_dict()
 protected=[k for k in before if k.startswith(('obs_normalizer.','distribution.'))]
 if not all(torch.equal(before[k],after[k]) for k in protected):raise ValueError('Full actor changed frozen sigma/normalizer')
 changed=any(not torch.equal(before[k],after[k]) for k in before if k.startswith('mlp.base.'))
 return {'fixed_sigma_and_normalizer_unchanged':True,'zero_command_base_parameters_changed':changed}


if __name__=='__main__':
 train_main(kind=KIND,config_loader=config,initialize=initialize,verify_actor=verify_actor,description=__doc__)
