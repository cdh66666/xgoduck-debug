"""Full-actor PPO refinement of the corrected-physics official sit/rise candidate."""
import hashlib,json
from pathlib import Path
from train import config as base_config,check_contract,main as train_main
KIND='xgoduck-official-sit-cycle-full-v1';ACTOR_CLASS='sit_refine_model:SitActor'

def config(path=None):
    if path:return check_contract(json.loads(Path(path).read_text()),ACTOR_CLASS)
    cfg=base_config();cfg['ppo']['actor'].update(class_name=ACTOR_CLASS,hidden_dims=[128,128],activation='tanh',obs_normalization=False)
    cfg['ppo']['actor']['distribution_cfg']['init_std']=.005
    cfg['ppo']['critic']['hidden_dims']=[128,128]
    cfg['ppo']['algorithm'].update(learning_rate=1e-6,entropy_coef=0.,schedule='fixed')
    return check_contract(cfg,ACTOR_CLASS)

def initialize(alg,path=None):
    import torch
    from tensordict import TensorDict
    from native_env import manifest
    if path is None:raise ValueError('Provide a full sit/rise candidate checkpoint')
    source=Path(path);saved=torch.load(source,map_location='cpu',weights_only=False)
    if saved.get('config',{}).get('physics_sha256')!=manifest()['physical_model']['sha256']:
        raise ValueError('Sit candidate physics differs from the selected plant')
    from sit_reference import SIT_DURATION,RISE_DURATION
    from sit_training import HOME
    source_cfg=saved['config']
    expected=dict(role='official-reference-sit-rise-candidate',phase_index=55,direction_index=56,
                  sit_duration_s=SIT_DURATION,rise_duration_s=RISE_DURATION,home_deg=HOME.tolist())
    if any(source_cfg.get(k)!=v for k,v in expected.items()):raise ValueError('Sit phase or HOME contract differs')
    original=saved['state_dict'];state=alg.actor.state_dict()
    for key in ('features','mean','std'):state['obs_normalizer.'+key]=original[key].clone()
    for key,value in original.items():
        if key.startswith('net.'):state['mlp.'+key[4:]]=value.clone()
    alg.actor.load_state_dict(state,strict=True)
    samples=torch.randn(128,61,generator=torch.Generator().manual_seed(31))*.2;samples[:,55]=torch.linspace(0,1,128);samples[:,56]=torch.where(torch.arange(128)%2==0,1.,-1.)
    with torch.inference_mode():
        expected=(samples[:,original['features']]-original['mean'])/original['std']
        for index in (0,2,4):
            expected=torch.nn.functional.linear(expected,original[f'net.{index}.weight'],original[f'net.{index}.bias'])
            if index!=4:expected=torch.tanh(expected)
        actual=alg.actor(TensorDict({'actor':samples},batch_size=[128]));error=float(torch.max(torch.abs(expected-actual)))
    if error>2e-5:raise ValueError('Sit full actor import parity failed')
    return dict(initialization='all sit actor weights imported; random critic; every actor network parameter trainable',source_checkpoint_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),initial_actor_parity_error=error,trainable_actor_parameters=sum(p.numel() for p in alg.actor.parameters() if p.requires_grad),hardware_commands_sent=False)

def verify(before,actor):
    import torch
    after=actor.state_dict();fixed=[k for k in before if k.startswith(('obs_normalizer.','distribution.'))]
    if not all(torch.equal(before[k],after[k]) for k in fixed):raise ValueError('Fixed feature statistics or exploration changed')
    return dict(actor_mlp_changed=any(not torch.equal(before[k],after[k]) for k in before if k.startswith('mlp.')),normalizer_and_sigma_fixed=True)

if __name__=='__main__':train_main(kind=KIND,config_loader=config,initialize=initialize,verify_actor=verify,description=__doc__)
