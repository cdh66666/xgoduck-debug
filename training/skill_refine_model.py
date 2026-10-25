"""Complete actor weights remain trainable; inherited observation statistics fixed."""
from rsl_rl.models import MLPModel

class SkillActor(MLPModel):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.obs_normalizer.until=0
        for name,parameter in self.named_parameters():parameter.requires_grad_(not name.startswith('distribution.'))
