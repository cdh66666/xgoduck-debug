"""Weight-free action source; exact inherited observation/action/filter math."""
import numpy as np
from policy_runtime import Policy as RuntimePolicy
class RawSession:
 def __init__(self):self.action=np.zeros((1,14),dtype=np.float32)
 def run(self,outputs,feed):return [self.action.copy()]
class Policy(RuntimePolicy):
 def load(self,path=None):
  self.session=RawSession();self.in_name='obs';self._outputs=['actions'];self.file='scratch-action-source'
  self._feed={self.in_name:self._obs.reshape(1,61)};self.reset_action_filter()
  return True,'external raw PPO action, no weights loaded'
