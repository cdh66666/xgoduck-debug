"""Export a saved standing/stop snapshot for independent native-physics review."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 a=p.parse_args()
 if a.output.exists():p.error('Use a new output path')
 import torch
 from tensordict import TensorDict
 from standstop import KIND,config,StandstopActor
 from train import export_policy
 torch.set_num_threads(1);saved=torch.load(a.checkpoint,map_location='cpu',weights_only=False)
 if saved['kind']=='xgoduck-brake-47500-bodyframe-v2':
  from brake import KIND,config,BrakeActor as StandstopActor
 elif saved['kind']!=KIND:raise ValueError('Expected a standstop/brake snapshot')
 cfg=config(a.checkpoint.parent/'config.json')
 if cfg!=saved['config']:raise ValueError('Snapshot/config mismatch')
 actor_cfg=dict(cfg['ppo']['actor']);actor_cfg.pop('class_name')
 actor=StandstopActor(TensorDict({'actor':torch.zeros(1,61)},batch_size=[1]),{'actor':['actor']},'actor',14,**actor_cfg)
 actor.load_state_dict(saved['algorithm']['actor_state_dict'],strict=True);actor.eval()
 a.output.parent.mkdir(parents=True,exist_ok=True)
 print(json.dumps(dict(iteration=saved['iteration'],export=export_policy(actor,a.output,saved['config'],architecture=KIND,initialization='frozen original + trained zero-command balance correction'))))


if __name__=='__main__':main()
