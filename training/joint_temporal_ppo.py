# Copyright (c) 2021-2026, ETH Zurich and NVIDIA CORPORATION
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# Fixed-schedule PPO update adapted from rsl_rl 5.0.1; license in licenses/rsl_rl.txt.
"""Optimize tracking and temporal continuity in the same PPO gradient step.

Unlike the exploratory alternating regularizer, no update discards the PPO
objective. Only the feed-forward, fixed-schedule configuration used here is
implemented. The rollout, inference clock and runtime filtering are unchanged.
"""
import math
import torch
from rsl_rl.algorithms import PPO
from temporal_ppo import consecutive_pairs, temporal_loss


class JointTemporalPPO(PPO):
    def __init__(self, *args, temporal_coef=.01, temporal_indices=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.temporal_coef = float(temporal_coef)
        self.temporal_indices=temporal_indices
        if temporal_indices is not None and (not temporal_indices or len(set(temporal_indices))!=len(temporal_indices) or any(type(i)!=int or not 0<=i<14 for i in temporal_indices)):
            raise ValueError('temporal_indices must select distinct actor joints in [0,14)')
        if not 0 <= self.temporal_coef <= 1:
            raise ValueError('temporal_coef must be in [0,1]')
        if (self.schedule != 'fixed' or self.actor.is_recurrent or self.critic.is_recurrent
                or self.rnd or self.symmetry or self.is_multi_gpu):
            raise ValueError('JointTemporalPPO supports this project fixed-schedule feed-forward configuration only')

    def update(self):
        count = self.storage.step
        left, right = consecutive_pairs(self.storage.observations['actor'][:count], self.storage.dones[:count])
        pair_batch_size = max(1, math.ceil(len(left) / self.num_mini_batches))
        totals = dict(value=0., surrogate=0., entropy=0., temporal_action_loss=0.)
        batches = self.storage.mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)
        updates = 0
        for index, batch in enumerate(batches):
            if self.normalize_advantage_per_mini_batch:
                with torch.no_grad():
                    batch.advantages = (batch.advantages-batch.advantages.mean())/(batch.advantages.std()+1e-8)
            self.actor(batch.observations, masks=batch.masks,
                       hidden_state=batch.hidden_states[0], stochastic_output=True)
            log_prob = self.actor.get_output_log_prob(batch.actions)
            entropy = self.actor.output_entropy
            values = self.critic(batch.observations, masks=batch.masks, hidden_state=batch.hidden_states[1])
            ratio = torch.exp(log_prob-torch.squeeze(batch.old_actions_log_prob))
            advantage = torch.squeeze(batch.advantages)
            surrogate = torch.max(-advantage*ratio, -advantage*ratio.clamp(1-self.clip_param,1+self.clip_param)).mean()
            if self.use_clipped_value_loss:
                clipped = batch.values+(values-batch.values).clamp(-self.clip_param,self.clip_param)
                value = torch.max((values-batch.returns).square(), (clipped-batch.returns).square()).mean()
            else:
                value = (batch.returns-values).square().mean()
            loss = surrogate+self.value_loss_coef*value-self.entropy_coef*entropy.mean()
            temporal = loss.new_zeros(())
            start = (index % self.num_mini_batches)*pair_batch_size
            if self.temporal_coef and start < len(left):
                temporal = temporal_loss(self.actor,left[start:start+pair_batch_size],right[start:start+pair_batch_size],indices=self.temporal_indices)
                loss = loss+self.temporal_coef*temporal
            self.optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(),self.max_grad_norm)
            torch.nn.utils.clip_grad_norm_(self.critic.parameters(),self.max_grad_norm)
            self.optimizer.step()
            for key, tensor in [('value',value),('surrogate',surrogate),('entropy',entropy.mean()),('temporal_action_loss',temporal)]:
                totals[key] += tensor.item()
            updates += 1
        self.storage.clear()
        result = {key: value/updates for key,value in totals.items()}
        result['temporal_pairs'] = len(left)
        return result
