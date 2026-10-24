"""Continue the packaged 47500 composite: only back_skip plus Critic64 learn."""
from __future__ import annotations
import argparse,copy,hashlib,json,os,time
from pathlib import Path
os.environ['CUDA_VISIBLE_DEVICES']=''
os.environ.setdefault('OMP_NUM_THREADS','1');os.environ.setdefault('OPENBLAS_NUM_THREADS','1');os.environ.setdefault('MKL_NUM_THREADS','1')
from train import ROOT,config as scratch_config,export_policy
KIND='xgoduck-backskip-47500-bodyframe-v2'

def config():
 cfg=scratch_config();cfg['ppo']['actor']['class_name']='refine_model:BackskipActor'
 cfg['ppo']['actor']['distribution_cfg']['init_std']=.02
 cfg['ppo']['algorithm'].update(learning_rate=3e-5,entropy_coef=.0005)
 return cfg

def load_algorithm(env,checkpoint):
 import torch
 from rsl_rl.algorithms import PPO
 cfg=config();saved=torch.load(checkpoint,map_location='cpu',weights_only=False)
 if saved.get('kind')!=KIND:raise ValueError('Expected a self-contained 47500 composite checkpoint, not scratch MLP')
 alg=PPO.construct_algorithm(env.get_observations(),env,copy.deepcopy(cfg['ppo']),'cpu')
 # Original backskip optimizer order: two actor parameters then eight critic parameters.
 params=[p for p in alg.actor.parameters() if p.requires_grad]+[p for p in alg.critic.parameters() if p.requires_grad]
 if len(params)!=10:raise ValueError('Expected back_skip2 + Critic8 Adam parameter objects')
 alg.optimizer=torch.optim.Adam(params,lr=3e-5)
 alg.load(saved,load_cfg=None,strict=True);alg.actor.obs_normalizer.until=0
 expected=saved['actor_state_dict'];current=alg.actor.state_dict()
 if not all(torch.equal(expected[k],current[k]) for k in expected):raise ValueError('Actor load was not exact')
 if not all(torch.equal(v,alg.critic.state_dict()[k]) for k,v in saved['critic_state_dict'].items()):raise ValueError('Critic load was not exact')
 if len(alg.optimizer.state)!=10:raise ValueError('Adam state not fully restored')
 return alg,cfg,int(saved['iter'])

def parity(actor,onnx_path):
 import numpy as np,torch,onnxruntime as ort
 module=actor.as_onnx(verbose=False).eval().cpu();x=np.random.default_rng(47500).normal(0,.25,(64,61)).astype('float32')
 from native_env import COMMANDS
 x[:,48:51]=np.asarray([COMMANDS[i%6] for i in range(len(x))],dtype=np.float32)
 with torch.inference_mode():expected=module(torch.tensor(x)).numpy()
 session=ort.InferenceSession(str(onnx_path),providers=['CPUExecutionProvider'])
 actual=np.concatenate([session.run(None,{'obs':row[None]})[0] for row in x]);error=float(np.max(np.abs(expected-actual)))
 if error>2e-5:raise ValueError('Bundled PT/ONNX parity failed: '+str(error))
 return {'samples':64,'inputs':[1,61],'outputs':[1,14],'max_abs_error':error}

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mode',choices=['inspect','smoke','train'],default='inspect')
 p.add_argument('--checkpoint',type=Path,default=ROOT/'checkpoints/model_047500.pt');p.add_argument('--output',type=Path,required=True)
 p.add_argument('--iterations',type=int,default=1);p.add_argument('--workers',type=int,default=1);p.add_argument('--envs-per-worker',type=int,default=2)
 p.add_argument('--seed',type=int,default=20261310);a=p.parse_args()
 if min(a.iterations,a.workers,a.envs_per_worker)<1:p.error('Positive iteration/worker/env counts required')
 a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
 import torch
 from native_env import NativeVecEnv
 torch.set_num_threads(1);torch.manual_seed(a.seed);env=None;started=time.monotonic()
 report={'kind':KIND,'hardware_commands_sent':False,'cpu_only':True,'parent_checkpoint_required':False,'trainable_scope':'back_skip868 parameters + Critic64 only','behavior_qualified':False,'initial_checkpoint_sha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest()}
 try:
  env=NativeVecEnv(workers=a.workers,envs_per_worker=a.envs_per_worker,seed=a.seed,curriculum='six');alg,cfg,iteration=load_algorithm(env,a.checkpoint)
  report.update(initial_iteration=iteration,actor_load_exact=True,critic_load_exact=True,optimizer_entries=len(alg.optimizer.state),optimizer_steps_before=sorted(set(float(s['step']) for s in alg.optimizer.state.values())))
  if a.checkpoint.resolve()==(ROOT/'checkpoints/model_047500.pt').resolve():report['packaged_onnx_parity']=parity(alg.actor,ROOT/'checkpoints/model_047500.onnx')
  before={k:v.detach().clone() for k,v in alg.actor.state_dict().items()};obs=env.get_observations();updates=0
  if a.mode!='inspect':
   for _ in range(a.iterations):
    alg.train_mode()
    with torch.inference_mode():
     for _ in range(cfg['ppo']['num_steps_per_env']):
      action=alg.act(obs);obs,reward,done,extras=env.step(action);alg.process_env_step(obs,reward,done,extras)
     alg.compute_returns(obs)
    loss=alg.update();iteration+=1;updates+=1;print(json.dumps({'iteration':iteration,'loss':loss}),flush=True)
   actual=alg.actor.state_dict();protected=[k for k in before if not k.startswith('mlp.back_skip.')]
   if not all(torch.equal(before[k],actual[k]) for k in protected):raise ValueError('Frozen original actor/normalizer/std changed')
   report['all_non_backskip_actor_tensors_unchanged']=True
   report['backskip_changed']=any(not torch.equal(before[k],actual[k]) for k in before if k.startswith('mlp.back_skip.'))
   out=alg.save();out.update(kind=KIND,iter=iteration,interface={'actor_input_dim':61,'actor_output_dim':14,'critic_input_dim':64,'policy_hz':50,'frame_contract':'bodyframe-v2'},metadata_sanitized=True)
   torch.save(out,a.output/'checkpoint.pt')
   report['export']=export_policy(alg.actor,a.output/'policy.onnx',cfg,architecture=KIND,initialization='continued packaged47500 composite, original base frozen')
  report.update(completed_updates=updates,final_iteration=iteration,wall_s=time.monotonic()-started,result='LOAD_PARITY_PASS' if a.mode=='inspect' else 'CONTINUATION_SMOKE_PASS_NOT_BEHAVIOR_QUALIFIED',optimizer_steps_after=sorted(set(float(s['step']) for s in alg.optimizer.state.values())))
  (a.output/'run.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
 finally:
  if env is not None:env.close()
 print(json.dumps(report),flush=True)
if __name__=='__main__':main()
