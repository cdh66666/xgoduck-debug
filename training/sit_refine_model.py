"""Full sit actor (5 or 61 feedback features); fixed statistics, no frozen actor."""
import torch
from rsl_rl.models import MLPModel


class SitFeatures(torch.nn.Module):
    def __init__(self,full_state=False):
        super().__init__();features=torch.arange(61) if full_state else torch.tensor([1,3,5,55,56])
        self.register_buffer('features',features)
        self.register_buffer('mean',torch.zeros(len(features)));self.register_buffer('std',torch.ones(len(features)))
    def forward(self,x):return (x[:,self.features]-self.mean)/self.std


class SitActor(MLPModel):
    def __init__(self,*args,full_state=False,**kwargs):
        self.full_state=bool(full_state)
        super().__init__(*args,**kwargs)
        if self.obs_dim!=61 or self.obs_normalization:raise ValueError('Sit actor requires61 inputs and fixed feature statistics')
        self.obs_normalizer=SitFeatures(self.full_state)
        for name,p in self.named_parameters():p.requires_grad_(not name.startswith('distribution.'))
    def _get_latent_dim(self):return 61 if self.full_state else 5
