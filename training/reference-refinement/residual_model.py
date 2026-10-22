"""Frozen parent0 MLP plus bounded residual, through the normal RSL model API.

The original normalizer runs once in MLPModel.get_latent. Both branches receive
that same normalized61 input. This is a Composite architecture, not one MLP.
"""
from __future__ import annotations
import hashlib
from pathlib import Path
import torch
from torch import nn
from rsl_rl.models import MLPModel


class ResidualComposite(nn.Module):
    def __init__(self,base,normalizer,residual_scale=.2):
        super().__init__()
        self.base=base.requires_grad_(False)
        self.residual=nn.Sequential(nn.Linear(61,128),nn.ELU(),nn.Linear(128,128),nn.ELU(),nn.Linear(128,14))
        nn.init.zeros_(self.residual[-1].weight)
        nn.init.zeros_(self.residual[-1].bias)
        scale=(normalizer._std[:,48:51]+normalizer.eps).detach().clone()
        self.register_buffer('zero_norm_command',(-normalizer._mean[:,48:51]/scale).detach().clone())
        self.register_buffer('command_scale',scale)
        self.residual_scale=float(residual_scale)
        self.head=nn.Linear(128,14)
        nn.init.zeros_(self.head.weight)
        nn.init.zeros_(self.head.bias)
        self.residual.requires_grad_(False);self.head.requires_grad_(False)
        self.back_skip=nn.Linear(61,14)
        nn.init.zeros_(self.back_skip.weight);nn.init.zeros_(self.back_skip.bias)

    def correction(self,normalized):
        # Equivalent to recovering raw command, using the EXACT normalized-zero
        # coordinate to make raw zero algebraically zero.1e-6 only absorbs
        # float32/export arithmetic; all six trained commands are0 or>=.2.
        command=(normalized[:,48:51]-self.zero_norm_command)*self.command_scale
        active=torch.any(torch.abs(command)>1e-6,dim=-1,keepdim=True)
        delta=self.residual_scale*torch.tanh(self.residual(normalized))
        return torch.where(active,delta,torch.zeros_like(delta))

    def base_features(self,normalized):
        # Execute existing registered layers; do not register an alias Sequential.
        h=normalized
        # Sequential can reuse the same ELU object. children() deduplicates it;
        # indices preserve every original execution position without aliases.
        for index in range(len(self.base)-1):h=self.base[index](h)
        return h

    def head_correction(self,normalized,features=None):
        if features is None:features=self.base_features(normalized)
        command=(normalized[:,48:51]-self.zero_norm_command)*self.command_scale
        active=torch.any(torch.abs(command)>1e-6,dim=-1,keepdim=True)
        delta=self.head(features)
        return torch.where(active,delta,torch.zeros_like(delta))

    def forward(self,normalized):
        h=self.base_features(normalized)
        base_u=self.base[-1](h)
        old_u=base_u+self.correction(normalized)
        frozen_u=old_u+self.head_correction(normalized,h)
        command=(normalized[:,48:51]-self.zero_norm_command)*self.command_scale
        delta=self.back_skip(normalized)
        return frozen_u+torch.where(command[:,0:1]<-.04,delta,torch.zeros_like(delta))


class ResidualActor(MLPModel):
    """Only MLPModel.mlp changes. Distribution/output/as_onnx APIs stay native."""
    def __init__(self,obs,obs_groups,obs_set,output_dim,parent_checkpoint,parent_checkpoint_sha256,
                 residual_scale=.2,**kwargs):
        super().__init__(obs,obs_groups,obs_set,output_dim,**kwargs)
        parent=Path(parent_checkpoint)
        digest=hashlib.sha256(parent.read_bytes()).hexdigest()
        if digest!=parent_checkpoint_sha256:
            raise ValueError('parent0_checkpoint_sha_changed')
        state=torch.load(parent,map_location='cpu',weights_only=False)['actor_state_dict']
        self.load_state_dict(state,strict=True)
        self.obs_normalizer.until=0
        self._expected_base={key:value.detach().clone() for key,value in self.mlp.state_dict().items()}
        self._expected_normalizer={key:value.detach().clone() for key,value in self.obs_normalizer.state_dict().items()}
        self.mlp=ResidualComposite(self.mlp,self.obs_normalizer,residual_scale)
        self.parent_checkpoint_sha256=digest
        with torch.no_grad():self.distribution.std_param.fill_(.02)
        self.distribution.std_param.requires_grad_(False)
        self.assert_parent_unchanged()

    def assert_source_actor_unchanged(self):
        expected=self._expected_full_source_actor
        actual=self.state_dict();added=['mlp.back_skip.weight','mlp.back_skip.bias']
        if set(actual)!=set(expected)|set(added):raise ValueError('old Actor keys changed')
        if not all(torch.equal(actual[key],value.to(actual[key].device)) for key,value in expected.items()):
            raise ValueError('fully frozen44500 Actor tensor changed')
        if any(p.requires_grad for name,p in self.named_parameters() if not name.startswith('mlp.back_skip.')):
            raise ValueError('old44500 Actor parameter trainable')
        return dict(all_old_Actor_tensors_exact=True,all_old_Actor_parameters_frozen=True,
            source_checkpoint_sha256=self._full_source_checkpoint_sha256,source_state_keys=sorted(expected),added_back_skip_keys=added)

    def assert_parent_unchanged(self):
        base=self.mlp.base.state_dict()
        normalizer=self.obs_normalizer.state_dict()
        if not all(torch.equal(value,self._expected_base[key].to(value.device)) for key,value in base.items()):
            raise ValueError('frozen_parent_base_changed')
        if not all(torch.equal(value,self._expected_normalizer[key].to(value.device)) for key,value in normalizer.items()):
            raise ValueError('frozen_parent_normalizer_changed')
        if any(parameter.requires_grad for parameter in self.mlp.base.parameters()):
            raise ValueError('parent_base_requires_grad')
        return dict(base_parent_equal=True,normalizer_parent_equal=True,
                    normalizer_buffers=list(normalizer),parent_checkpoint_sha256=self.parent_checkpoint_sha256)


def fresh_optimizer(algorithm,learning_rate):
    actor=algorithm.actor
    for name,parameter in actor.named_parameters():parameter.requires_grad_(name.startswith('mlp.back_skip.'))
    actor.obs_normalizer.until=0
    actor_parameters=[p for p in actor.parameters() if p.requires_grad]
    critic_parameters=[p for p in algorithm.critic.parameters() if p.requires_grad]
    if len(actor_parameters)!=2 or sum(p.numel() for p in actor_parameters)!=868:raise ValueError('backskip only2Actor objects')
    algorithm.optimizer=torch.optim.Adam(actor_parameters+critic_parameters,lr=learning_rate)
    if len(actor_parameters)+len(critic_parameters)!=10:raise ValueError('freshAdam10 required')
    return dict(sigma_requires_grad=False,sigma_excluded=True,optimizer_parameter_objects=10,
        actor_trainable_parameters=868,critic_trainable_parameters=sum(p.numel() for p in critic_parameters),
        optimizer='fresh Adam back_skip+Critic64 only; entire oldActor frozen',source_actor_excluded=True,parent_base_excluded=True)


def load_backskip_weights(algorithm,loaded,source_checkpoint_sha256):
    old=loaded['actor_state_dict'];state=dict(old)
    keys=['mlp.back_skip.weight','mlp.back_skip.bias']
    if any(key in old for key in keys):raise ValueError('initializer must be old44500 without backskip')
    current=algorithm.actor.state_dict()
    for key in keys:
        if torch.count_nonzero(current[key]):raise ValueError('backskip initializer must be zero')
        state[key]=current[key].detach().clone()
    adapted=dict(loaded);adapted['actor_state_dict']=state
    algorithm.load(adapted,dict(actor=True,critic=True,optimizer=False,iteration=False,rnd=False),strict=True)
    actor=algorithm.actor
    actor._expected_full_source_actor={key:value.detach().clone() for key,value in old.items()}
    actor._full_source_checkpoint_sha256=source_checkpoint_sha256
    actor.obs_normalizer.until=0
    if not torch.equal(actor.distribution.std_param,torch.full_like(actor.distribution.std_param,.02)):raise ValueError('sourceSigma.02 required')
    proof=actor.assert_source_actor_unchanged()
    critic=all(torch.equal(algorithm.critic.state_dict()[key],value) for key,value in loaded['critic_state_dict'].items())
    if not critic or algorithm.optimizer.state:raise ValueError('Critic exact and freshAdam empty required')
    return dict(strict_load=True,added_back_skip_keys=keys,all_old_Actor_tensors_exact=True,Critic_exact=True,
        source_head_nonzero=bool(torch.count_nonzero(old['mlp.head.weight'])) and bool(torch.count_nonzero(old['mlp.head.bias'])),
        source_sigma=.02,actual_initial_std=actor.distribution.std_param.detach().cpu().tolist(),
        optimizer_state_loaded=False,Adam_initial_state_entries=0,full_frozen_actor_proof=proof)
