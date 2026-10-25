"""Native CPU PPO from random weights, with an explicit output directory."""
from __future__ import annotations
import argparse,copy,gzip,hashlib,json,os,time
from collections import Counter
from pathlib import Path
os.environ['CUDA_VISIBLE_DEVICES']=''
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('MKL_NUM_THREADS','1')
ROOT=Path(__file__).resolve().parent
KIND='xgoduck-scratch-mlp-bodyframe-v2'

def check_contract(cfg,actor_class='rsl_rl.models:MLPModel'):
 expected=json.loads((ROOT/'config.json').read_text('utf-8'))
 from runtime_profile import selected,CALIBRATED,HOME_DEG
 if selected()==CALIBRATED:
  import math
  expected['home_rad']=[math.radians(v) for v in HOME_DEG]
  expected['runtime_profile']=CALIBRATED
 if {k:v for k,v in cfg.items() if k!='ppo'}!={k:v for k,v in expected.items() if k!='ppo'}:
  raise ValueError('Training config changed the fixed physical/observation/action contract')
 if cfg['ppo']['actor']['class_name']!=actor_class or cfg['ppo']['critic']['class_name']!='rsl_rl.models:MLPModel':
  raise ValueError('Training actor/critic class does not match this entry')
 return cfg

def config(path=None):
 cfg=json.loads((path or ROOT/'config.json').read_text('utf-8'))
 if path is None:
  from runtime_profile import selected,CALIBRATED,HOME_DEG
  if selected()==CALIBRATED:
   import math
   cfg['home_rad']=[math.radians(v) for v in HOME_DEG];cfg['runtime_profile']=CALIBRATED
 return check_contract(cfg)

def construct(env,cfg):
 from rsl_rl.algorithms import PPO
 return PPO.construct_algorithm(env.get_observations(),env,copy.deepcopy(cfg['ppo']),'cpu')

def export_policy(actor,path,cfg,*,architecture=KIND,initialization='random, no parent weights',physical_model=None):
 import numpy as np,torch,onnx,onnxruntime as ort
 module=actor.as_onnx(verbose=False).eval().cpu()
 torch.onnx.export(module,torch.zeros(1,61),str(path),input_names=['obs'],output_names=['actions'],opset_version=17,dynamo=False)
 graph=onnx.load(str(path));onnx.checker.check_model(graph)
 metadata=dict(default_joint_pos=','.join(str(v) for v in cfg['home_rad']),joint_names=','.join(cfg['joint_names']),
  action_scale='1.0',runtime_hz='50',action_alpha='.45',action_alpha_head='.2' if cfg.get('runtime_profile')=='calibrated-20261025' else '.45',gyro_alpha='.5',velocity_alpha='.4',kp_fw='6',kd_fw='20',
  runtime_interface_version='bodyframe-v2',architecture=architecture,training_initialization=initialization,
  observation_names='base_ang_vel,projected_gravity,joint_pos,joint_vel,actions,command,head_command,body_command',
  body_command='zero',hardware_deployment_ready='false',kd_model_limitation='BAM has no identified Feetech D20 mapping')
 metadata['runtime_profile']=cfg.get('runtime_profile','legacy')
 if architecture=='xgoduck-roll-phase-full-v2':
  metadata.update(role='forward-roll-single-cycle',phase_index='55',phase_duration_s='1.9',
   phase_definition='min(elapsed_seconds/1.9,1)',body_command='observation55 is phase; remaining body-command entries zero')
 if physical_model is not None:metadata['physical_model_sha256']=physical_model['sha256']
 for key,value in metadata.items():item=graph.metadata_props.add();item.key=key;item.value=value
 onnx.save(graph,str(path));session=ort.InferenceSession(str(path),providers=['CPUExecutionProvider'])
 samples=np.random.default_rng(17).normal(0,.25,(64,61)).astype('float32')
 from native_env import COMMANDS
 samples[:,48:51]=np.asarray([COMMANDS[i%6] for i in range(len(samples))],dtype=np.float32)
 with torch.inference_mode():expected=module(torch.tensor(samples)).numpy()
 actual=np.concatenate([session.run(None,{'obs':row[None]})[0] for row in samples])
 error=float(np.max(np.abs(actual-expected)));assert error<2e-5 and np.isfinite(actual).all()
 return {'max_abs_error':error,'inputs':[1,61],'outputs':[1,14],'samples':64,'behavior_qualified':False}

def main(*,kind=KIND,config_loader=config,initialize=None,verify_actor=None,description=__doc__):
 parser=argparse.ArgumentParser(description=description)
 parser.add_argument('--mode',choices=['smoke','train'],default='smoke')
 parser.add_argument('--output',type=Path,required=True)
 parser.add_argument('--iterations',type=int,default=1)
 parser.add_argument('--workers',type=int,default=1)
 parser.add_argument('--envs-per-worker',type=int,default=2)
 parser.add_argument('--curriculum',choices=['stand','six','stability','stop','speed','continuous','ramped','roll','roll_phase'],default='stand')
 parser.add_argument('--runtime-profile',choices=['legacy','calibrated-20261025'],default='legacy')
 parser.add_argument('--randomize-physics',action='store_true',help='Use the documented assumed voltage/contact/COM/inertia/delay stress envelope')
 parser.add_argument('--target-rate-cost',type=float,default=0.,help='Additional mean squared filtered-target rate cost, relative to 0.1 rad per 20ms')
 parser.add_argument('--head-motion-cost',type=float,default=0.,help='Locomotion reward cost for head joint speed and large head target offsets; no output clamp')
 parser.add_argument('--max-forward',type=float,default=.3,help='Speed curriculum forward upper bound, .2 to 1 m/s')
 parser.add_argument('--trace-every',type=int,default=1,help='Record one full rollout per N updates; metrics remain every update')
 parser.add_argument('--seed',type=int,default=20261022)
 parser.add_argument('--config',type=Path,help='Optional PPO hyperparameters; fixed physical and interface fields must match')
 parser.add_argument('--max-wall-seconds',type=float,help='Finish the current update and export when this training time is reached')
 parser.add_argument('--resume',type=Path,help='Only a checkpoint made by this entry with the same config')
 parser.add_argument('--initial-checkpoint',type=Path,help='Explicit source for this entry initialization; cannot combine with resume')
 parser.add_argument('--model-xml',type=Path,help='Explicit experimental physics XML; its hash is saved in checkpoint, run and ONNX')
 args=parser.parse_args()
 if min(args.iterations,args.workers,args.envs_per_worker)<1:parser.error('Positive iteration/worker/env counts required')
 if args.max_wall_seconds is not None and args.max_wall_seconds<=0:parser.error('Positive wall time required')
 if args.resume and args.initial_checkpoint:parser.error('Use resume or an initial checkpoint, not both')
 if args.initial_checkpoint and initialize is None:parser.error('This entry has no pretrained initialization')
 if not .2<=args.max_forward<=1. or args.trace_every<1:parser.error('Invalid speed or trace interval')
 if not 0<=args.target_rate_cost<=10:parser.error('Target rate cost must be in [0,10]')
 if not 0<=args.head_motion_cost<=10:parser.error('Head motion cost must be in [0,10]')
 os.environ['XGODUCK_RUNTIME_PROFILE']=args.runtime_profile
 os.environ['XGODUCK_RANDOMIZE_PHYSICS']='1' if args.randomize_physics else '0'
 os.environ['XGODUCK_MAX_FORWARD']=str(args.max_forward)
 os.environ['XGODUCK_TARGET_RATE_COST']=str(args.target_rate_cost)
 os.environ['XGODUCK_HEAD_MOTION_COST']=str(args.head_motion_cost)
 if args.model_xml:os.environ['XGODUCK_TRAIN_XML']=str(args.model_xml.resolve())
 args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
 import numpy as np,torch
 from native_env import NativeVecEnv
 torch.set_num_threads(1);torch.manual_seed(args.seed);cfg=config_loader(args.config);env=None;trace=None;started=time.monotonic()
 report={'kind':kind,'initialization':'random','parent_checkpoint_required':False,'hardware_commands_sent':False,
  'deployment_allowed':False,'behavior_qualified':False,'cpu_only':True,'seed':args.seed,'curriculum':args.curriculum,
  'versions':{'torch':torch.__version__,'numpy':np.__version__},'requested_updates':args.iterations,
  'max_wall_seconds':args.max_wall_seconds,'workers':args.workers,'envs_per_worker':args.envs_per_worker}
 report['trace_every_updates']=args.trace_every
 (args.output/'config.json').write_text(json.dumps(cfg,indent=2),encoding='utf-8')
 try:
  env=NativeVecEnv(workers=args.workers,envs_per_worker=args.envs_per_worker,seed=args.seed,curriculum=args.curriculum)
  report['worker_metadata']=env.pool.worker_metadata
  report['training_environment']=env.cfg
  alg=construct(env,cfg);base=0
  if args.resume:
   checkpoint=torch.load(args.resume,map_location='cpu',weights_only=False)
   if checkpoint.get('kind')!=kind or checkpoint.get('config')!=cfg:raise ValueError('Checkpoint is not this entry/config contract')
   old_model=checkpoint.get('training_environment',{}).get('physical_model',{})
   expected_model=old_model.get('sha256',env.cfg['physical_model']['reference_model_sha256'])
   if expected_model!=env.cfg['physical_model']['sha256']:raise ValueError('Resume physics model differs; use a deliberate pretrained initialization for a new plant')
   alg.load(checkpoint['algorithm'],load_cfg=None,strict=True);base=checkpoint['iteration'];report['initialization']='resume own '+kind+' checkpoint'
   report['resume_sha256']=hashlib.sha256(args.resume.read_bytes()).hexdigest()
  elif initialize is not None:
   report.update(initialize(alg,args.initial_checkpoint))
  obs=env.get_observations();assert obs['actor'].shape[-1]==61 and obs['critic'].shape[-1]==64
  before={key:value.detach().clone() for key,value in alg.actor.state_dict().items()};updates=0
  trace=gzip.open(args.output/'rollout.jsonl.gz','wt',encoding='utf-8')
  for iteration in range(base+1,base+args.iterations+1):
   iteration_started=time.monotonic();alg.train_mode();rewards=[];terminations=0;failures=Counter()
   with torch.inference_mode():
    for tick in range(cfg['ppo']['num_steps_per_env']):
     actions=alg.act(obs);obs,reward,done,extras=env.step(actions)
     if not torch.isfinite(obs['actor']).all() or not torch.isfinite(actions).all():raise ValueError('Nonfinite training data')
     alg.process_env_step(obs,reward,done,extras);rewards.append(float(reward.mean()));terminations+=int(done.sum())
     rows=extras['native_rows']
     for row in rows:failures.update(row['failure_reasons'])
     fields=('velocity','tilt','height','standing_height','command','command_age_s','zero_release','failure_reasons','resets','steps')
     if args.curriculum in ('roll','roll_phase'):fields+=('roll_rotation_deg','roll_complete','head_peak_n','head_impulse_ns','contact_sampling_hz')
     if (iteration-base-1)%args.trace_every==0:
      trace.write(json.dumps({'iteration':iteration,'tick':tick,'t':env.common_step_counter*.02,
       'actions':actions.tolist(),'rewards':reward.tolist(),'done':done.tolist(),
       'envs':[{k:row[k] for k in fields} for row in rows]},allow_nan=False)+'\n')
    alg.compute_returns(obs)
   losses=alg.update();updates+=1
   elapsed=time.monotonic()-started
   budget_reached=args.max_wall_seconds is not None and elapsed>=args.max_wall_seconds
   row={'iteration':iteration,'mean_reward':sum(rewards)/len(rewards),'terminations':terminations,'failure_counts':dict(failures),
    'elapsed_s':elapsed,'update_wall_s':time.monotonic()-iteration_started,
    'transitions':updates*cfg['ppo']['num_steps_per_env']*env.num_envs,'loss':losses,'behavior_qualified':False}
   with (args.output/'metrics.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
   if iteration%100==0 or iteration==base+args.iterations or budget_reached:
    saved={'kind':kind,'iteration':iteration,'algorithm':alg.save(),'config':cfg,'training_environment':env.cfg}
    torch.save(saved,args.output/'checkpoint.pt')
    if iteration%100==0:torch.save(saved,args.output/f'checkpoint_{iteration:06d}.pt')
    trace.flush()
   print(json.dumps(row),flush=True)
   if budget_reached:break
  report['actor_parameters_changed']=any(not torch.equal(value,alg.actor.state_dict()[key]) for key,value in before.items())
  if verify_actor is not None:report.update(verify_actor(before,alg.actor))
  report['completed_updates']=updates;report['final_iteration']=iteration;report['stop_reason']='wall_budget' if budget_reached else 'iteration_budget'
  report['transitions']=updates*cfg['ppo']['num_steps_per_env']*env.num_envs
  report['export']=export_policy(alg.actor,args.output/'policy.onnx',cfg,architecture=kind,initialization=report['initialization'],physical_model=env.cfg['physical_model'])
  report['wall_s']=time.monotonic()-started;report['result']='SMOKE_PASS_NOT_BEHAVIOR_QUALIFIED' if args.mode=='smoke' else 'TRAINED_PENDING_INDEPENDENT_EVALUATION'
  (args.output/'run.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 except Exception as error:
  # A failed export must not leave the dependent evaluator waiting forever for
  # a completion marker. Preserve checkpoints and the actual failure result.
  report.update(result='FAILED',error_type=type(error).__name__,error_message=str(error),wall_s=time.monotonic()-started)
  (args.output/'run.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
  raise
 finally:
  if trace is not None:trace.close()
  if env is not None:env.close()
 print(json.dumps(report),flush=True)
if __name__=='__main__':main()
