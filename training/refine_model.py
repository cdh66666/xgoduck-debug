"""47500 composite graph loader; all weights come from the self-contained final checkpoint."""
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

class BackskipActor(MLPModel):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  self.mlp=ResidualComposite(self.mlp,self.obs_normalizer,residual_scale=.2)
  self.obs_normalizer.until=0
  for name,parameter in self.named_parameters():parameter.requires_grad_(name.startswith('mlp.back_skip.'))
