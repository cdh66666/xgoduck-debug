"""Independent deterministic stand/six-point/stop evaluation, no automatic reset."""
from __future__ import annotations
import argparse,json,math,os
from pathlib import Path
os.environ['CUDA_VISIBLE_DEVICES']=''
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
from train import KIND,check_contract,config

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--seeds',type=int,default=8);p.add_argument('--first-seed',type=int,default=20261030)
 p.add_argument('--model-xml',type=Path,help='Explicit evaluation plant; otherwise reuse the checkpoint physics model')
 p.add_argument('--runtime-profile',choices=['legacy','calibrated-20261025'],help='Explicit evaluation contract, otherwise the checkpoint profile')
 p.add_argument('--randomize-physics',action='store_true',help='Independent randomized plant stress tests; nominal is the default')
 p.add_argument('--max-forward',type=float,default=.3,help='Replace the Shift test speed, up to 1 m/s')
 a=p.parse_args()
 if a.seeds<1:p.error('At least one seed required')
 if not .2<=a.max_forward<=1.:p.error('Forward speed must be .2..1 m/s')
 import numpy as np,torch
 from tensordict import TensorDict
 from native_env import NativeEpisode,manifest,COMMANDS
 from rsl_rl.models import MLPModel
 torch.set_num_threads(1);saved=torch.load(a.checkpoint,map_location='cpu',weights_only=False)
 profile=a.runtime_profile or saved.get('training_environment',{}).get('runtime_profile','legacy')
 os.environ['XGODUCK_RUNTIME_PROFILE']=profile
 os.environ['XGODUCK_RANDOMIZE_PHYSICS']='1' if a.randomize_physics else '0'
 cfg=config()
 commands=[list(c) for c in COMMANDS];commands[2][0]=a.max_forward
 actor_class=MLPModel
 if saved.get('kind')==KIND:
  cfg=check_contract(saved['config'])
  state=saved['algorithm']['actor_state_dict']
 elif saved.get('kind')=='xgoduck-full-47500-bodyframe-v2':
  from full_refine import ACTOR_CLASS,Full47500Actor
  cfg=check_contract(saved['config'],ACTOR_CLASS);state=saved['algorithm']['actor_state_dict'];actor_class=Full47500Actor
 elif saved.get('kind')=='xgoduck-continuous-command-full-v1':
  from continuous_refine import ACTOR_CLASS
  from continuous_model import Continuous47500Actor
  cfg=check_contract(saved['config'],ACTOR_CLASS);state=saved['algorithm']['actor_state_dict'];actor_class=Continuous47500Actor
 elif saved.get('kind')=='xgoduck-official-walk-full-v1':
  from official_walk_refine import ACTOR_CLASS
  from skill_refine_model import SkillActor
  cfg=check_contract(saved['config'],ACTOR_CLASS);state=saved['algorithm']['actor_state_dict'];actor_class=SkillActor
 elif saved.get('kind')=='xgoduck-standstop-47500-bodyframe-v2':
  from standstop import ACTOR_CLASS,StandstopActor
  cfg=check_contract(saved['config'],ACTOR_CLASS);state=saved['algorithm']['actor_state_dict'];actor_class=StandstopActor
 elif saved.get('kind')=='xgoduck-brake-47500-bodyframe-v2':
  from brake import ACTOR_CLASS,BrakeActor
  cfg=check_contract(saved['config'],ACTOR_CLASS);state=saved['algorithm']['actor_state_dict'];actor_class=BrakeActor
 elif saved.get('kind')=='xgoduck-backskip-47500-bodyframe-v2':
  from refine import config as refine_config
  from refine_model import BackskipActor
  cfg=refine_config();state=saved['actor_state_dict'];actor_class=BackskipActor
 else:raise ValueError('Expected own scratch/full47500 or packaged47500 composite checkpoint')
 trained_model=saved.get('training_environment',{}).get('physical_model',{})
 explicit_model=a.model_xml is not None or bool(os.environ.get('XGODUCK_TRAIN_XML'))
 if a.model_xml:os.environ['XGODUCK_TRAIN_XML']=str(a.model_xml.resolve())
 elif not explicit_model and trained_model.get('path'):
  os.environ['XGODUCK_TRAIN_XML']=trained_model['path']
 dummy=TensorDict({'actor':torch.zeros(1,61)},batch_size=[1]);actor_cfg=dict(cfg['ppo']['actor']);actor_cfg.pop('class_name')
 actor=actor_class(dummy,{'actor':['actor']},'actor',14,**actor_cfg);actor.load_state_dict(state);actor.eval()
 cases=[];frozen=manifest()
 if not explicit_model and trained_model and frozen['physical_model']['sha256']!=trained_model['sha256']:
  raise ValueError('Checkpoint physics XML changed on disk')
 for seed in range(a.first_seed,a.first_seed+a.seeds):
  for command in commands:
   env=NativeEpisode(frozen,seed,curriculum='stand',auto_reset=False);env.reset();rows=[];failure=None;previous_command=None
   try:
    for tick in range(750 if command==[0.,0.,0.] else 900):
     moving=tick<600 and command!=[0.,0.,0.];current=command if moving else [0.,0.,0.]
     target=np.asarray(current,dtype=np.float32)
     if current!=previous_command:
      env.motion_window.reset(target);env.command_age_ticks=0;env.zero_release=previous_command is not None and current==[0.,0.,0.]
      previous_command=list(current)
     env.command=target;env.command_ticks=1000000;env.command_index=commands.index(current)
     obs=env.observe()
     with torch.inference_mode():action=actor(TensorDict({'actor':torch.tensor(obs[None])},batch_size=[1])).numpy()[0]
     _,_,done,_,info=env.step(action)
     rows.append({'t':(tick+1)*.02,'velocity':info['velocity'],'tilt':info['tilt'],'height':info['height'],'standing_height':info['standing_height'],'command':current})
     if done:failure=info['failure_reasons'];break
   except Exception as e:failure=[str(e)]
   finally:env.close()
   move=[r for r in rows if 3<=r['t']<=12 and command!=[0.,0.,0.]]
   stop=[r for r in rows if (r['t']>=15 if command!=[0.,0.,0.] else r['t']>=3)]
   finite=bool(rows) and all(r['velocity'] is not None for r in rows)
   complete=len(rows)==(750 if command==[0.,0.,0.] else 900)
   tilt=max((r['tilt'] for r in rows if r['tilt'] is not None),default=float('inf'))
   mean=np.mean([r['velocity'] for r in move],axis=0) if move else None
   rmse=np.sqrt(np.mean([(np.asarray(r['velocity'])-command)**2 for r in move],axis=0)) if move else None
   stop_ok=bool(stop) and all(math.hypot(*r['velocity'][:2])<=.03 and abs(r['velocity'][2])<=.08 for r in stop)
   tracking=True
   if command!=[0.,0.,0.]:
    axis=2 if command[2] else 0;target=command[axis]
    tracking=mean is not None and .8<=mean[axis]/target<=1.2 and rmse[axis]<=(.2 if axis==2 else .05)
    if axis==0:tracking=tracking and rmse[1]<=.05 and rmse[2]<=.2
    else:tracking=tracking and math.hypot(rmse[0],rmse[1])<=.05
   passed=finite and complete and failure is None and tilt<=25 and stop_ok and tracking
   case={'seed':seed,'command':command,'passed':bool(passed),'ticks':len(rows),'failure':failure,'max_tilt_deg':tilt if math.isfinite(tilt) else None,'mean_velocity':mean.tolist() if mean is not None else None,'rmse':rmse.tolist() if rmse is not None else None,'stop_passed':stop_ok,'resets_during_recording':0,'rows':rows}
   if env.variation:case['physics_sample']=env.variation.sample.copy()
   cases.append(case);print(json.dumps({k:v for k,v in case.items() if k!='rows'}),flush=True)
 report={'kind':saved['kind'],'result':'PASS_THIS_MATRIX' if all(c['passed'] for c in cases) else 'NOT_PASSED','cases':cases,'seeds':a.seeds,
  'physical_model':frozen['physical_model'],'training_physical_model':trained_model or None,'runtime_profile':profile,'max_forward_mps':a.max_forward,
  'checkpoint_sha256':__import__('hashlib').sha256(a.checkpoint.read_bytes()).hexdigest(),'hardware_deployment_ready':False,
  'randomized_physics':a.randomize_physics,
  'scope':'stand15s, independent six commands12s then stop6s; no reset/recovery allowed. Does not include continuous126s or disturbance/getup acceptance.'}
 a.output.parent.mkdir(parents=True,exist_ok=True)
 with a.output.open('x',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
if __name__=='__main__':main()
