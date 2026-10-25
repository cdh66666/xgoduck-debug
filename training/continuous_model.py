"""Fully trainable 47500 graph with continuous command-conditioned corrections.

The legacy graph remains intact for reproducibility. Tensor names and the six
original command endpoints are preserved; zero and backward threshold jumps
are removed inside the actor, not hidden by an extra control/output buffer.
"""
import torch
from refine_model import ResidualComposite
from full_refine_model import Full47500Actor

class ContinuousComposite(ResidualComposite):
    @staticmethod
    def blend(value):
        u=torch.clamp(value,0.,1.)
        return u*u*(3.-2.*u)

    def forward(self,normalized):
        h=self.base_features(normalized)
        command=(normalized[:,48:51]-self.zero_norm_command)*self.command_scale
        reference=command.new_tensor([.2,.2,1.])
        moving=self.blend(torch.amax(torch.abs(command)/reference,dim=-1,keepdim=True))
        backward=self.blend(-command[:,0:1]/.3)
        residual=self.residual_scale*torch.tanh(self.residual(normalized))
        return self.base[-1](h)+moving*(residual+self.head(h))+backward*self.back_skip(normalized)

class Continuous47500Actor(Full47500Actor):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        previous=self.mlp
        self.mlp=ContinuousComposite(previous.base,self.obs_normalizer,previous.residual_scale)
        self.mlp.load_state_dict(previous.state_dict(),strict=True)
        for parameter in self.mlp.parameters():parameter.requires_grad_(True)
