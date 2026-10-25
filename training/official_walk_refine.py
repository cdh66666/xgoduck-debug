"""Retrain every official Walk actor weight on the corrected physical plant."""
from skill_refine import config as skill_config,initialize,verify,ACTOR_CLASS
from train import main as train_main

KIND='xgoduck-official-walk-full-v1'
def config(path=None):
    cfg=skill_config(path)
    if path is None:cfg['ppo']['algorithm']['learning_rate']=1e-6
    return cfg

if __name__=='__main__':train_main(kind=KIND,config_loader=config,initialize=initialize,verify_actor=verify,description=__doc__)
