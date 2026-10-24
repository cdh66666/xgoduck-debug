"""Keep the validated quiet-standing actor fixed and learn only dynamic braking."""
import torch
from torch import nn
from rsl_rl.models import MLPModel
from standstop_model import StandstopComposite


class BrakeComposite(StandstopComposite):
 def __init__(self,base,normalizer):
  super().__init__(base,normalizer)
  self.brake_skip=nn.Sequential(nn.Linear(61,64),nn.ELU(),nn.Linear(64,14))
  nn.init.zeros_(self.brake_skip[-1].weight);nn.init.zeros_(self.brake_skip[-1].bias)
  self.register_buffer('raw_mean',normalizer._mean.detach().clone())
  self.register_buffer('raw_scale',(normalizer._std+normalizer.eps).detach().clone())

 def set_statistics(self,normalizer):
  with torch.no_grad():
   self.raw_mean.copy_(normalizer._mean)
   self.raw_scale.copy_(normalizer._std+normalizer.eps)

 def forward(self,normalized):
  standing=super().forward(normalized)
  raw=normalized*self.raw_scale+self.raw_mean
  gyro=torch.linalg.vector_norm(raw[:,:3],dim=-1,keepdim=True)/.15
  tilt=torch.linalg.vector_norm(raw[:,3:5],dim=-1,keepdim=True)/.02618
  legs=torch.cat((raw[:,20:25],raw[:,29:34]),dim=-1)
  joints=torch.sqrt(torch.mean(legs*legs,dim=-1,keepdim=True)+1e-12)/.3
  activity=torch.maximum(torch.maximum(gyro,tilt),joints)
  gate=torch.clamp(activity-1.,0.,1.)
  command=(normalized[:,48:51]-self.zero_norm_command)*self.command_scale
  stopped=torch.all(torch.abs(command)<=1e-6,dim=-1,keepdim=True)
  correction=.15*gate*torch.tanh(self.brake_skip(normalized))
  return standing+torch.where(stopped,correction,torch.zeros_like(correction))


class BrakeActor(MLPModel):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  self.mlp=BrakeComposite(self.mlp,self.obs_normalizer)
  self.obs_normalizer.until=0
  for name,parameter in self.named_parameters():
   parameter.requires_grad_(name.startswith('mlp.brake_skip.'))
