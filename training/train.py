"""Native CPU PPO from random weights, with an explicit output directory."""
from __future__ import annotations
import argparse,copy,hashlib,json,os,time
from pathlib import Path
os.environ['CUDA_VISIBLE_DEVICES']=''
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('MKL_NUM_THREADS','1')
ROOT=Path(__file__).resolve().parent
KIND='xgoduck-scratch-mlp-bodyframe-v2'

def config():
 return json.loads((ROOT/'config.json').read_text('utf-8'))

def construct(env,cfg):
 from rsl_rl.algorithms import PPO
 return PPO.construct_algorithm(env.get_observations(),env,copy.deepcopy(cfg['ppo']),'cpu')

def export_policy(actor,path,cfg,*,architecture=KIND,initialization='random, no parent weights'):
 import numpy as np,torch,onnx,onnxruntime as ort
 module=actor.as_onnx(verbose=False).eval().cpu()
 torch.onnx.export(module,torch.zeros(1,61),str(path),input_names=['obs'],output_names=['actions'],opset_version=17,dynamo=False)
 graph=onnx.load(str(path));onnx.checker.check_model(graph)
 metadata=dict(default_joint_pos=','.join(str(v) for v in cfg['home_rad']),joint_names=','.join(cfg['joint_names']),
  action_scale='1.0',runtime_hz='50',action_alpha='.45',gyro_alpha='.5',velocity_alpha='.4',kp_fw='6',kd_fw='20',
  runtime_interface_version='bodyframe-v2',architecture=architecture,training_initialization=initialization,
  observation_names='base_ang_vel,projected_gravity,joint_pos,joint_vel,actions,command,head_command,body_command',
  body_command='zero',hardware_deployment_ready='false',kd_model_limitation='BAM has no identified Feetech D20 mapping')
 for key,value in metadata.items():item=graph.metadata_props.add();item.key=key;item.value=value
 onnx.save(graph,str(path));session=ort.InferenceSession(str(path),providers=['CPUExecutionProvider'])
 samples=np.random.default_rng(17).normal(0,.25,(64,61)).astype('float32')
 with torch.inference_mode():expected=module(torch.tensor(samples)).numpy()
 actual=np.concatenate([session.run(None,{'obs':row[None]})[0] for row in samples])
 error=float(np.max(np.abs(actual-expected)));assert error<2e-5 and np.isfinite(actual).all()
 return {'max_abs_error':error,'inputs':[1,61],'outputs':[1,14],'samples':64,'behavior_qualified':False}

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--mode',choices=['smoke','train'],default='smoke')
 parser.add_argument('--output',type=Path,required=True)
 parser.add_argument('--iterations',type=int,default=1)
 parser.add_argument('--workers',type=int,default=1)
 parser.add_argument('--envs-per-worker',type=int,default=2)
 parser.add_argument('--curriculum',choices=['stand','six'],default='stand')
 parser.add_argument('--seed',type=int,default=20261022)
 parser.add_argument('--resume',type=Path,help='Only a checkpoint made by this scratch entry; optional')
 args=parser.parse_args()
 if min(args.iterations,args.workers,args.envs_per_worker)<1:parser.error('Positive iteration/worker/env counts required')
 args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
 import numpy as np,torch
 from native_env import NativeVecEnv
 torch.set_num_threads(1);torch.manual_seed(args.seed);cfg=config();env=None;started=time.monotonic()
 report={'kind':KIND,'initialization':'random','parent_checkpoint_required':False,'hardware_commands_sent':False,
  'deployment_allowed':False,'behavior_qualified':False,'cpu_only':True,'seed':args.seed,'curriculum':args.curriculum,
  'versions':{'torch':torch.__version__,'numpy':np.__version__},'requested_updates':args.iterations}
 try:
  env=NativeVecEnv(workers=args.workers,envs_per_worker=args.envs_per_worker,seed=args.seed,curriculum=args.curriculum)
  report['worker_metadata']=env.pool.worker_metadata
  alg=construct(env,cfg);base=0
  if args.resume:
   checkpoint=torch.load(args.resume,map_location='cpu',weights_only=False)
   if checkpoint.get('kind')!=KIND or checkpoint.get('config')!=cfg:raise ValueError('Checkpoint is not this scratch training contract')
   alg.load(checkpoint['algorithm'],load_cfg=None,strict=True);base=checkpoint['iteration'];report['initialization']='resume own scratch checkpoint'
   report['resume_sha256']=hashlib.sha256(args.resume.read_bytes()).hexdigest()
  obs=env.get_observations();assert obs['actor'].shape[-1]==61 and obs['critic'].shape[-1]==64
  before={key:value.detach().clone() for key,value in alg.actor.state_dict().items()};metrics=[]
  for iteration in range(base+1,base+args.iterations+1):
   alg.train_mode();rewards=[];terminations=0
   with torch.inference_mode():
    for _ in range(cfg['ppo']['num_steps_per_env']):
     actions=alg.act(obs);obs,reward,done,extras=env.step(actions)
     if not torch.isfinite(obs['actor']).all() or not torch.isfinite(actions).all():raise ValueError('Nonfinite training data')
     alg.process_env_step(obs,reward,done,extras);rewards.append(float(reward.mean()));terminations+=int(done.sum())
    alg.compute_returns(obs)
   losses=alg.update();row={'iteration':iteration,'mean_reward':sum(rewards)/len(rewards),'terminations':terminations,'loss':losses,'behavior_qualified':False}
   with (args.output/'metrics.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
   if iteration%100==0 or iteration==base+args.iterations:
    torch.save({'kind':KIND,'iteration':iteration,'algorithm':alg.save(),'config':cfg},args.output/'checkpoint.pt')
   metrics.append(row);print(json.dumps(row),flush=True)
  report['actor_parameters_changed']=any(not torch.equal(value,alg.actor.state_dict()[key]) for key,value in before.items())
  report['completed_updates']=args.iterations;report['export']=export_policy(alg.actor,args.output/'policy.onnx',cfg)
  report['wall_s']=time.monotonic()-started;report['result']='SMOKE_PASS_NOT_BEHAVIOR_QUALIFIED' if args.mode=='smoke' else 'TRAINED_PENDING_INDEPENDENT_EVALUATION'
  (args.output/'run.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 finally:
  if env is not None:env.close()
 print(json.dumps(report),flush=True)
if __name__=='__main__':main()
