"""Train native standing and stop transients while preserving the original gait."""
import hashlib,json
from pathlib import Path
from train import ROOT,check_contract,config as scratch_config,main as train_main

KIND='xgoduck-standstop-47500-bodyframe-v2'
ACTOR_CLASS='standstop:StandstopActor'


def __getattr__(name):
 if name=='StandstopActor':
  from standstop_model import StandstopActor
  return StandstopActor
 raise AttributeError(name)


def config(path=None):
 if path is not None:return check_contract(json.loads(Path(path).read_text('utf-8')),ACTOR_CLASS)
 cfg=scratch_config();cfg['ppo']['actor']['class_name']=ACTOR_CLASS
 cfg['ppo']['actor']['distribution_cfg']['init_std']=.005
 cfg['ppo']['algorithm'].update(learning_rate=1e-5,entropy_coef=0.,schedule='fixed')
 return check_contract(cfg,ACTOR_CLASS)


def initialize(alg,checkpoint=None):
 import torch
 from refine import parity
 source=Path(checkpoint) if checkpoint else ROOT/'checkpoints/model_047500.pt';saved=torch.load(source,map_location='cpu',weights_only=False)
 exploration={k:v.detach().clone() for k,v in alg.actor.distribution.state_dict().items()}
 if saved.get('kind')!='xgoduck-backskip-47500-bodyframe-v2':raise ValueError('Expected packaged47500 source')
 load=alg.actor.load_state_dict(saved['actor_state_dict'],strict=False)
 expected={key for key in alg.actor.state_dict() if key.startswith('mlp.zero_skip.')}
 if set(load.missing_keys)!=expected or load.unexpected_keys:raise ValueError('Unexpected original actor load mismatch')
 alg.critic.load_state_dict(saved['critic_state_dict'],strict=True)
 if not all(torch.equal(value,alg.actor.state_dict()[key]) for key,value in saved['actor_state_dict'].items()):raise ValueError('Original weights changed on load')
 alg.actor.distribution.load_state_dict(exploration,strict=True)
 if alg.optimizer.state:raise ValueError('New residual optimizer must start fresh')
 return dict(initialization='packaged47500 frozen; zero-initialized zero-command correction and fresh Adam',
  source_checkpoint_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),original_actor_load_exact=True,
  trainable_actor_parameters=sum(p.numel() for p in alg.actor.parameters() if p.requires_grad),
  trainable_scope='4878-parameter zero-command-only bounded correction + Critic64; original actor weights/normalizer fixed; sampling sigma fixed at configured value',
  training_sampling_distribution={k:v.tolist() for k,v in exploration.items()},
  initial_onnx_parity=parity(alg.actor,ROOT/'checkpoints/model_047500.onnx'))


def verify_actor(before,actor):
 import torch
 after=actor.state_dict();protected=[k for k in before if not k.startswith('mlp.zero_skip.')]
 if not all(torch.equal(before[k],after[k]) for k in protected):raise ValueError('Frozen actor/normalizer or fixed sampling distribution changed')
 return dict(original_actor_weights_normalizer_unchanged=True,fixed_sampling_distribution_unchanged=True,
  zero_command_correction_changed=any(not torch.equal(before[k],after[k]) for k in before if k.startswith('mlp.zero_skip.')))


if __name__=='__main__':
 train_main(kind=KIND,config_loader=config,initialize=initialize,verify_actor=verify_actor,description=__doc__)
