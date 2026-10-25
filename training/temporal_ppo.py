"""PPO with an alternating, differentiable temporal action regularizer.

Inspired by CAPS (https://ai.bu.edu/caps/), not an exact reproduction. Uses
consecutive native observations, never a runtime action buffer. Episode resets
and command changes are excluded. PPO still trains all actor/critic weights;
Additional actor-only steps penalize deterministic action differences.
"""
import torch
from tensordict import TensorDict
from rsl_rl.algorithms import PPO


def consecutive_pairs(observations, dones):
    left, right = observations[:-1], observations[1:]
    valid = ~dones[:-1].squeeze(-1).bool()
    valid &= (left[..., 48:51] == right[..., 48:51]).all(-1)
    return left[valid].detach().clone(), right[valid].detach().clone()


def temporal_loss(actor, left, right, scale=.1, indices=None):
    a = actor(TensorDict({'actor': left}, batch_size=[len(left)]))
    b = actor(TensorDict({'actor': right}, batch_size=[len(right)]))
    difference=(a-b)/scale
    if indices is not None:difference=difference[:,indices]
    return difference.square().mean()


class TemporalPPO(PPO):
    def __init__(self, *args, temporal_coef=1., temporal_steps=1, **kwargs):
        super().__init__(*args, **kwargs)
        self.temporal_coef = float(temporal_coef)
        self.temporal_steps = int(temporal_steps)
        if not 0 <= self.temporal_coef <= 10 or not 1 <= self.temporal_steps <= 10:
            raise ValueError('Invalid temporal regularizer config')

    def update(self):
        count = self.storage.step
        left, right = consecutive_pairs(self.storage.observations['actor'][:count], self.storage.dones[:count])
        result = super().update()
        result['temporal_pairs'] = len(left)
        if not len(left) or not self.temporal_coef:
            result['temporal_action_loss'] = 0.
            return result
        values = []
        for _ in range(self.temporal_steps):
            loss = temporal_loss(self.actor, left, right)
            self.optimizer.zero_grad()
            (self.temporal_coef * loss).backward()
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(), self.max_grad_norm)
            self.optimizer.step()
            values.append(float(loss.detach()))
        result['temporal_action_loss'] = sum(values)/len(values)
        return result
