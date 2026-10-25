"""Full-actor PPO from the official normalized ELU ONNX, on corrected physics."""
import hashlib
from pathlib import Path
from train import config as base_config,check_contract,main as train_main

KIND='xgoduck-roll-full-v1';ACTOR_CLASS='skill_refine_model:SkillActor'
def config(path=None):
    import json
    if path:return check_contract(json.loads(Path(path).read_text()),ACTOR_CLASS)
    cfg=base_config();cfg['ppo']['actor']['class_name']=ACTOR_CLASS
    cfg['ppo']['actor']['distribution_cfg']['init_std']=.02
    cfg['ppo']['algorithm'].update(learning_rate=1e-5,entropy_coef=0.,schedule='fixed')
    return check_contract(cfg,ACTOR_CLASS)

def initialize(alg,path=None):
    import numpy as np,torch,onnx,onnxruntime as ort
    from onnx import numpy_helper
    from tensordict import TensorDict
    if path is None:raise ValueError('Provide the official roll ONNX explicitly')
    path=Path(path);graph=onnx.load(str(path));weights={v.name:numpy_helper.to_array(v).copy() for v in graph.graph.initializer}
    expected=[('obs_normalizer._mean',(1,61)),('onnx::Div_24',(1,61))]
    for index,shape in [(0,(512,61)),(2,(256,512)),(4,(128,256)),(6,(14,128))]:expected.extend([(f'mlp.{index}.weight',shape),(f'mlp.{index}.bias',(shape[0],))])
    if any(key not in weights or weights[key].shape!=shape for key,shape in expected):raise ValueError('Unsupported ONNX actor layout')
    state=alg.actor.state_dict()
    for key,value in weights.items():
        if key.startswith('mlp.'):state[key]=torch.from_numpy(value)
    state['obs_normalizer._mean']=torch.from_numpy(weights['obs_normalizer._mean'])
    scale=torch.from_numpy(weights['onnx::Div_24'])-alg.actor.obs_normalizer.eps
    state['obs_normalizer._std']=scale;state['obs_normalizer._var']=scale.square()
    alg.actor.load_state_dict(state,strict=True);alg.actor.eval()
    opts=ort.SessionOptions();opts.intra_op_num_threads=1;session=ort.InferenceSession(str(path),sess_options=opts,providers=['CPUExecutionProvider'])
    samples=np.random.default_rng(37).normal(0,.2,(128,61)).astype(np.float32)
    with torch.inference_mode():actual=alg.actor(TensorDict({'actor':torch.from_numpy(samples)},batch_size=[128])).numpy()
    expected=np.concatenate([session.run(None,{session.get_inputs()[0].name:row[None]})[0] for row in samples])
    error=float(np.max(np.abs(actual-expected)))
    if error>2e-5:raise ValueError('ONNX import parity failed: '+str(error))
    return dict(initialization='every official ONNX actor tensor imported; random critic; all actor MLP weights trainable',source_onnx_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                initial_onnx_parity_error=error,trainable_actor_parameters=sum(p.numel() for p in alg.actor.parameters() if p.requires_grad),hardware_commands_sent=False)

def verify(before,actor):
    import torch
    after=actor.state_dict();protected=[k for k in before if k.startswith(('obs_normalizer.','distribution.'))]
    if not all(torch.equal(before[k],after[k]) for k in protected):raise ValueError('Fixed normalizer/sigma changed')
    return dict(actor_mlp_changed=any(not torch.equal(before[k],after[k]) for k in before if k.startswith('mlp.')),normalizer_and_sigma_fixed=True)

if __name__=='__main__':train_main(kind=KIND,config_loader=config,initialize=initialize,verify_actor=verify,description=__doc__)
