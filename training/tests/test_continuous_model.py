import copy,sys,unittest,tempfile
from types import SimpleNamespace
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import torch
from tensordict import TensorDict
from full_refine import config
from full_refine_model import Full47500Actor
from continuous_model import Continuous47500Actor

class ContinuousCommandTests(unittest.TestCase):
    def test_own_checkpoint_initializes_every_tensor_with_fresh_optimizer(self):
        from continuous_refine import KIND,initialize
        from full_refine import initialize as legacy_initialize
        actor=torch.nn.Linear(3,2);critic=torch.nn.Linear(4,1)
        def algorithm():
            a=torch.nn.Linear(3,2);c=torch.nn.Linear(4,1)
            return SimpleNamespace(actor=a,critic=c,optimizer=torch.optim.Adam([*a.parameters(),*c.parameters()]))
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'checkpoint.pt'
            torch.save({'kind':KIND,'iteration':6000,'algorithm':{'actor_state_dict':actor.state_dict(),'critic_state_dict':critic.state_dict()}},path)
            alg=algorithm();result=initialize(alg,path)
            for source,target in ((actor,alg.actor),(critic,alg.critic)):
                for key,value in source.state_dict().items():self.assertTrue(torch.equal(value,target.state_dict()[key]))
            self.assertEqual(result['source_iteration'],6000)
            self.assertEqual(result['initial_optimizer_entries'],0)
            self.assertEqual(len(alg.optimizer.state),0)
            self.assertTrue(result['actor_load_exact'] and result['critic_load_exact'])
            with self.assertRaisesRegex(ValueError,'Expected packaged47500 source'):legacy_initialize(algorithm(),path)

    def test_original_endpoints_identical_and_thresholds_are_continuous(self):
        torch.manual_seed(47500);cfg=config()['ppo']['actor'];cfg.pop('class_name')
        inputs=TensorDict({'actor':torch.zeros(1,61)},batch_size=[1])
        old=Full47500Actor(inputs,{'actor':['actor']},'actor',14,**copy.deepcopy(cfg))
        new=Continuous47500Actor(inputs,{'actor':['actor']},'actor',14,**copy.deepcopy(cfg))
        # Nonzero correction weights make the threshold test meaningful.
        for name,p in old.mlp.named_parameters():
            if not name.startswith('base.'):torch.nn.init.normal_(p,0,.04)
        new.load_state_dict(old.state_dict(),strict=True)
        samples=torch.randn(6,61)*.2
        samples[:,48:51]=torch.tensor([[0,0,0],[.2,0,0],[.3,0,0],[-.3,0,0],[0,0,1],[0,0,-1]])
        with torch.no_grad():
            td=TensorDict({'actor':samples},batch_size=[6])
            torch.testing.assert_close(new(td),old(td),rtol=1e-5,atol=1e-6)
            for left,right in [(0,1e-6),(-.039999,-.040001)]:
                pair=samples[:2].clone();pair[1]=pair[0];pair[:,48]=torch.tensor([left,right]);pair[:,49:51]=0
                out=new(TensorDict({'actor':pair},batch_size=[2]))
                self.assertLess(float((out[1]-out[0]).abs().max()),1e-4)
        self.assertTrue(all(p.requires_grad for p in new.mlp.parameters()))

if __name__=='__main__':unittest.main()
