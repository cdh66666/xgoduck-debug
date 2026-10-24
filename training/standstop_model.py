"""Frozen original walking actor with a bounded, zero-command balance correction."""
import torch
from torch import nn
from rsl_rl.models import MLPModel
from refine_model import ResidualComposite


class StandstopComposite(ResidualComposite):
 def __init__(self,base,normalizer):
  super().__init__(base,normalizer,residual_scale=.2)
  self.zero_skip=nn.Sequential(nn.Linear(61,64),nn.ELU(),nn.Linear(64,14))
  nn.init.zeros_(self.zero_skip[-1].weight);nn.init.zeros_(self.zero_skip[-1].bias)

 def forward(self,normalized):
  original=super().forward(normalized)
  command=(normalized[:,48:51]-self.zero_norm_command)*self.command_scale
  stopped=torch.all(torch.abs(command)<=1e-6,dim=-1,keepdim=True)
  correction=.10*torch.tanh(self.zero_skip(normalized))
  return original+torch.where(stopped,correction,torch.zeros_like(correction))


class StandstopActor(MLPModel):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  self.mlp=StandstopComposite(self.mlp,self.obs_normalizer)
  self.obs_normalizer.until=0
  for name,parameter in self.named_parameters():
   parameter.requires_grad_(name.startswith('mlp.zero_skip.'))
