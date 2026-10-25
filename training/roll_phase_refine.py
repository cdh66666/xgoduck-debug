"""Full roll actor with an explicit 1.9-second phase and one-turn objective."""
from skill_refine import config as skill_config,initialize as import_official,verify,ACTOR_CLASS
from train import main as train_main

KIND='xgoduck-roll-phase-full-v2'
def config(path=None):
    cfg=skill_config(path)
    if path is None:cfg['ppo']['algorithm']['learning_rate']=1e-6
    return cfg

def initialize(alg,path=None):
    import torch
    from tensordict import TensorDict
    report=import_official(alg,path)
    sample=torch.randn(128,61)*.2;sample[:,55]=0
    obs=TensorDict({'actor':sample},batch_size=[128])
    with torch.no_grad():
        before=alg.actor(obs).clone();layer=alg.actor.mlp[0];normal=alg.actor.obs_normalizer
        zero=(-normal._mean[0,55]/(normal._std[0,55]+normal.eps))
        layer.bias.add_(layer.weight[:,55]*zero);layer.weight[:,55].zero_()
        error=float((alg.actor(obs)-before).abs().max())
    if error>2e-5:raise ValueError('Phase initialization changed the original zero-phase policy')
    report.update(initialization='official full actor warmstart; unused phase55 column zeroed with bias compensation; all MLP weights trainable',phase_index=55,phase_duration_s=1.9,phase_zero_initialization_parity=error)
    return report

if __name__=='__main__':train_main(kind=KIND,config_loader=config,initialize=initialize,verify_actor=verify,description=__doc__)
