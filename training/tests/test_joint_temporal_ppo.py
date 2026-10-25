import copy
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import torch
from tensordict import TensorDict
from rsl_rl.algorithms import PPO
from rsl_rl.models import MLPModel
from rsl_rl.storage import RolloutStorage
from joint_temporal_ppo import JointTemporalPPO


def algorithm(kind, **kwargs):
    torch.manual_seed(7)
    obs=TensorDict({'actor':torch.zeros(2,61),'critic':torch.zeros(2,64)},batch_size=[2])
    groups={'actor':['actor'],'critic':['critic']}
    actor=MLPModel(obs,groups,'actor',14,hidden_dims=[16],distribution_cfg={'class_name':'GaussianDistribution','init_std':.2,'std_type':'scalar'})
    critic=MLPModel(obs,groups,'critic',1,hidden_dims=[16])
    storage=RolloutStorage('rl',2,6,obs,[14])
    alg=kind(actor,critic,storage,num_learning_epochs=2,num_mini_batches=2,schedule='fixed',**kwargs)
    with torch.no_grad():
        for tick in range(6):
            alg.act(obs)
            nxt=obs.clone();nxt['actor'][:,0]=tick+1;nxt['critic'][:,0]=tick+1
            done=torch.tensor([tick==2,False])
            alg.process_env_step(nxt,torch.tensor([.2,.4]),done,{})
            obs=nxt
        alg.compute_returns(obs)
    return alg


class JointTemporalTests(unittest.TestCase):
    def test_zero_coefficient_matches_upstream_ppo_update(self):
        upstream=algorithm(PPO);joint=algorithm(JointTemporalPPO,temporal_coef=0)
        torch.manual_seed(20);expected=upstream.update()
        torch.manual_seed(20);actual=joint.update()
        for name in ('actor','critic'):
            for key,value in getattr(upstream,name).state_dict().items():
                torch.testing.assert_close(value,getattr(joint,name).state_dict()[key],rtol=0,atol=0)
        for key,value in expected.items():self.assertAlmostEqual(value,actual[key],places=7)

    def test_regularizer_shares_tracking_steps_and_preserves_critic_learning(self):
        alg=algorithm(JointTemporalPPO,temporal_coef=.01)
        before=copy.deepcopy(alg.critic.state_dict());steps=[]
        handle=alg.optimizer.register_step_post_hook(lambda *_:steps.append(1))
        result=alg.update();handle.remove()
        self.assertEqual(len(steps),4)
        self.assertEqual(result['temporal_pairs'],9)
        self.assertGreater(result['temporal_action_loss'],0)
        self.assertTrue(any(not torch.equal(value,alg.critic.state_dict()[key]) for key,value in before.items()))
        self.assertEqual(alg.storage.step,0)


if __name__=='__main__':unittest.main()
