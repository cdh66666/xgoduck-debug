"""Learn braking while freezing the previously validated standing correction."""
import hashlib,json
from pathlib import Path
from train import ROOT,check_contract,main as train_main
KIND='xgoduck-brake-47500-bodyframe-v2'
ACTOR_CLASS='brake:BrakeActor'


def __getattr__(name):
 if name=='BrakeActor':
  from brake_model import BrakeActor
  return BrakeActor
 raise AttributeError(name)


def config(path=None):
 if path is not None:return check_contract(json.loads(Path(path).read_text('utf-8')),ACTOR_CLASS)
 from standstop import config as standing_config
 cfg=standing_config();cfg['ppo']['actor']['class_name']=ACTOR_CLASS
 cfg['ppo']['algorithm']['learning_rate']=3e-4
 return check_contract(cfg,ACTOR_CLASS)


def initialize(alg,checkpoint=None):
 import torch
 if checkpoint is None:raise ValueError('Supply --initial-checkpoint with the validated standstop checkpoint')
 source=Path(checkpoint);saved=torch.load(source,map_location='cpu',weights_only=False)
 if saved.get('kind')!='xgoduck-standstop-47500-bodyframe-v2':raise ValueError('Expected a self-contained standstop source')
 check_contract(saved['config'],'standstop:StandstopActor')
 expected=saved['algorithm']['actor_state_dict'];loaded=alg.actor.load_state_dict(expected,strict=False)
 missing={k for k in alg.actor.state_dict() if k.startswith('mlp.brake_skip.') or k in ('mlp.raw_mean','mlp.raw_scale')}
 if set(loaded.missing_keys)!=missing or loaded.unexpected_keys:raise ValueError('Unexpected standing actor load mismatch')
 if not all(torch.equal(v,alg.actor.state_dict()[k]) for k,v in expected.items()):raise ValueError('Standing actor load changed weights')
 alg.actor.mlp.set_statistics(alg.actor.obs_normalizer)
 alg.critic.load_state_dict(saved['algorithm']['critic_state_dict'],strict=True)
 if alg.optimizer.state:raise ValueError('The new braking scope requires fresh Adam')
 return dict(initialization='frozen validated standing actor; zero-initialized dynamic brake and fresh Adam',
  source_checkpoint_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),standing_actor_load_exact=True,
  source_iteration=saved['iteration'],optimizer_reset_for_new_scope=True,
  trainable_actor_parameters=sum(p.numel() for p in alg.actor.parameters() if p.requires_grad),
  trainable_scope='4878-parameter dynamic zero-command brake + Critic64; quiet-standing/original actor frozen',
  brake_gate=dict(gyro_threshold_radps=.15,tilt_sine_threshold=.02618,leg_velocity_rms_threshold_radps=.3,
   ramp='zero below every threshold, linear ramp to twice any threshold',max_dynamic_correction_rad=.15))


def verify_actor(before,actor):
 import torch
 after=actor.state_dict();protected=[k for k in before if not k.startswith('mlp.brake_skip.')]
 if not all(torch.equal(before[k],after[k]) for k in protected):raise ValueError('Validated standing actor or fixed statistics changed')
 return dict(validated_standing_actor_and_statistics_unchanged=True,
  dynamic_brake_changed=any(not torch.equal(before[k],after[k]) for k in before if k.startswith('mlp.brake_skip.')))


if __name__=='__main__':
 train_main(kind=KIND,config_loader=config,initialize=initialize,verify_actor=verify_actor,description=__doc__)
