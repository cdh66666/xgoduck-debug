"""Parent-process full47500 model; physics workers never need this import."""
from refine_model import BackskipActor


class Full47500Actor(BackskipActor):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  for name,parameter in self.named_parameters():
   parameter.requires_grad_(not name.startswith('distribution.'))
