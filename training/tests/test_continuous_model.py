import copy,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import torch
from tensordict import TensorDict
from full_refine import config
from full_refine_model import Full47500Actor
from continuous_model import Continuous47500Actor

class ContinuousCommandTests(unittest.TestCase):
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
