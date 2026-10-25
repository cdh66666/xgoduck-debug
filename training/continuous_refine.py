"""Full actor refinement after removing the legacy command-gate discontinuities."""
import json
from pathlib import Path
from train import check_contract,main as train_main
from full_refine import KIND as FULL_KIND,config as full_config,initialize as load_full,verify_actor

KIND='xgoduck-continuous-command-full-v1'
ACTOR_CLASS='continuous_model:Continuous47500Actor'

def config(path=None):
    if path:return check_contract(json.loads(Path(path).read_text()),ACTOR_CLASS)
    cfg=full_config();cfg['ppo']['actor']['class_name']=ACTOR_CLASS
    cfg['ppo']['algorithm']['learning_rate']=3e-6
    return check_contract(cfg,ACTOR_CLASS)

def initialize(alg,path=None):
    # New runs may warm-start from this entry's own complete checkpoints too.
    # Keep the legacy entry's accepted kinds unchanged to avoid silently
    # switching a continuous policy back to discontinuous command gates.
    report=load_full(alg,path,accepted_full_kinds=(FULL_KIND,KIND))
    report['architecture_change']='Same actor tensors; continuous smoothstep command gates replace zero/nonzero and backward threshold switches'
    report['command_gate_reference']={'vx_mps':.2,'vy_mps':.2,'yaw_radps':1.,'backward_mps':.3}
    return report

if __name__=='__main__':
    train_main(kind=KIND,config_loader=config,initialize=initialize,verify_actor=verify_actor,description=__doc__)
