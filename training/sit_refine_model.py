"""Full 19086-parameter sit actor; feature statistics fixed, no frozen actor."""
import torch
from rsl_rl.models import MLPModel


class SitFeatures(torch.nn.Module):
    def __init__(self):
        super().__init__();self.register_buffer('features',torch.tensor([1,3,5,55,56]))
        self.register_buffer('mean',torch.zeros(5));self.register_buffer('std',torch.ones(5))
    def forward(self,x):return (x[:,self.features]-self.mean)/self.std


class SitActor(MLPModel):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        if self.obs_dim!=61 or self.obs_normalization:raise ValueError('Sit actor requires61 inputs and fixed feature statistics')
        self.obs_normalizer=SitFeatures()
        for name,p in self.named_parameters():p.requires_grad_(not name.startswith('distribution.'))
    def _get_latent_dim(self):return 5
