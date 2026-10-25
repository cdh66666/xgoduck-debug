import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import torch
from temporal_ppo import consecutive_pairs,temporal_loss


class TemporalRegularizerTests(unittest.TestCase):
    def test_pairs_exclude_resets_and_command_changes(self):
        obs=torch.zeros(4,2,61);obs[1,:,0]=1;obs[2,:,0]=2;obs[3,:,0]=3
        obs[2:,1,48]=.2
        dones=torch.zeros(4,2,1,dtype=torch.uint8);dones[0,0]=1
        left,right=consecutive_pairs(obs,dones)
        self.assertEqual(len(left),4)
        self.assertTrue(torch.all(right[:,0]-left[:,0]==1))
        self.assertTrue(torch.equal(left[:,48:51],right[:,48:51]))
        left[0,0]=100
        self.assertTrue(torch.all(obs[...,0]<100))

    def test_direct_gradient_reduces_action_difference(self):
        class Actor(torch.nn.Module):
            def __init__(self):
                super().__init__();self.net=torch.nn.Linear(61,14,bias=False)
            def forward(self,obs):return self.net(obs['actor'])
        torch.manual_seed(1);actor=Actor();left=torch.zeros(12,61);right=left.clone();right[:,0]=1
        optimizer=torch.optim.SGD(actor.parameters(),lr=.001)
        before=temporal_loss(actor,left,right).item()
        for _ in range(10):
            loss=temporal_loss(actor,left,right);optimizer.zero_grad();loss.backward();optimizer.step()
        self.assertLess(temporal_loss(actor,left,right).item(),before)
        self.assertEqual(temporal_loss(actor,left,left).item(),0.)

if __name__=='__main__':unittest.main()
