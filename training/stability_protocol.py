"""Local native-physics standing/stop cases, shared by metrics and video.

All forces are applied inside MuJoCo. No hardware, PC control, or network API.
"""
from pathlib import Path
import hashlib
import math
import numpy as np
import onnxruntime as ort
from native_env import NativeEpisode,manifest,COMMANDS


class OnnxPolicy:
 def __init__(self,path):
  self.path=Path(path)
  opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
  self.session=ort.InferenceSession(str(self.path),sess_options=opts,providers=['CPUExecutionProvider'])
  self.input=self.session.get_inputs()[0].name
  self.sha256=hashlib.sha256(self.path.read_bytes()).hexdigest()
 def act(self,obs):
  return self.session.run(None,{self.input:np.asarray(obs,dtype=np.float32)[None]})[0][0]


def scenarios(push_n=.8):
 result={
  'stand4':dict(title='静止站立',segments=[(4.,[0.,0.,0.])]),
  'stand':dict(title='静止站立',segments=[(15.,[0.,0.,0.])]),
  'forward_stop':dict(title='前进 0.3 → 松键停止',segments=[(6.,[.3,0.,0.]),(6.,[0.,0.,0.])]),
  'slow_stop':dict(title='前进 0.2 → 松键停止',segments=[(6.,[.2,0.,0.]),(6.,[0.,0.,0.])]),
  'backward_stop':dict(title='后退 −0.3 → 松键停止',segments=[(6.,[-.3,0.,0.]),(6.,[0.,0.,0.])]),
  'left_stop':dict(title='左转 +1 → 松键停止',segments=[(5.,[0.,0.,1.]),(5.,[0.,0.,0.])]),
  'right_stop':dict(title='右转 −1 → 松键停止',segments=[(5.,[0.,0.,-1.]),(5.,[0.,0.,0.])]),
  'switches':dict(title='连续前后走、转向与停止',segments=[(3.,[.3,0.,0.]),(2.,[0.,0.,0.]),
   (3.,[-.3,0.,0.]),(2.,[0.,0.,0.]),(3.,[0.,0.,1.]),(2.,[0.,0.,0.]),(3.,[0.,0.,-1.]),(2.,[0.,0.,0.])])}
 for name,force,title in [('push_front',[push_n,0.,0.],'站立受向前推力'),('push_back',[-push_n,0.,0.],'站立受向后推力'),
  ('push_left',[0.,push_n,0.],'站立受向左推力'),('push_right',[0.,-push_n,0.],'站立受向右推力')]:
  result[name]=dict(title=title,segments=[(12.,[0.,0.,0.])],pushes=[dict(t=3.,duration=.16,force=force)])
 return result


def duration(case):return sum(s[0] for s in case['segments'])


def command_at(case,t):
 end=0.
 for seconds,command in case['segments']:
  start=end;end+=seconds
  if t<end-1e-8:return list(command),start
 return list(case['segments'][-1][1]),end-case['segments'][-1][0]


def set_command(env,command):
 target=np.asarray(command,dtype=np.float32)
 if not np.array_equal(target,env.command):
  previous=env.command.copy();env.motion_window.reset(target);env.command_age_ticks=0
  env.zero_release=bool(np.any(previous) and not np.any(target))
 env.command=target;env.command_ticks=1000000;env.command_index=COMMANDS.index(command)


def tick(env,policy,case,index):
 t=index*.02;command,segment_start=command_at(case,t)
 set_command(env,command)
 for pulse in case.get('pushes',[]):
  if index==round(pulse['t']/.02):
   env.sim.command(dict(op='push',force=pulse['force'],duration=pulse['duration'],body='trunk_base'))
 obs=env.observe();action=policy.act(obs)
 _,reward,done,timeout,info=env.step(action)
 row=dict(t=(index+1)*.02,command=command,segment_start=segment_start,velocity=info['velocity'],
  tilt=info['tilt'],height=info['height'],standing_height=info['standing_height'],
  position=env.sim.data.qpos[:3].tolist(),action=action.tolist(),
  force=env.sim.force.tolist() if env.sim.data.time<=env.sim.force_until else [0.,0.,0.],
  failure=info['failure_reasons'] if done else None,resets=env.total_resets)
 if done and not row['failure']:row['failure']=['episode_timeout' if timeout else 'unknown_termination']
 return row,done


def new_episode(seed,*,max_episode_steps=1200):
 env=NativeEpisode(manifest(),seed,curriculum='stand',auto_reset=False,max_episode_steps=max_episode_steps);env.reset()
 return env


def case_metrics(rows,case):
 def rms(values):return math.sqrt(sum(x*x for x in values)/len(values)) if values else None
 finite=[r for r in rows if r['velocity'] is not None and r['tilt'] is not None]
 quiet=[r for r in finite if not any(r['command']) and r['t']>=3]
 metrics=dict(complete=len(rows)==round(duration(case)/.02),
  failure=next((r['failure'] for r in rows if r['failure']),None),
  max_tilt_deg=max((r['tilt'] for r in finite),default=None),
  quiet_speed_rms_mps=rms([math.hypot(*r['velocity'][:2]) for r in quiet]),
  quiet_yaw_rms_radps=rms([r['velocity'][2] for r in quiet]),
  quiet_tilt_rms_deg=rms([r['tilt'] for r in quiet]),stops=[])
 end=0.;previous=None
 for seconds,command in case['segments']:
  start=end;end+=seconds
  if previous is not None and any(previous) and not any(command):
   first=[r for r in finite if start<r['t']<=min(start+1.,end)+1e-8]
   full=[r for r in finite if start<r['t']<=end+1e-8]
   settled=None
   for j,r in enumerate(full):
    window=full[j:j+25]
    if len(window)==25 and all(math.hypot(*x['velocity'][:2])<=.03 and abs(x['velocity'][2])<=.08 and x['tilt']<=5 for x in window):
     settled=r['t']-start;break
   metrics['stops'].append(dict(start_s=start,previous_command=previous,
    first_second_planar_path_m=sum(math.hypot(*r['velocity'][:2])*.02 for r in first),
    first_second_yaw_path_rad=sum(abs(r['velocity'][2])*.02 for r in first),
    first_second_tilt_rms_deg=rms([r['tilt'] for r in first]),
    max_tilt_deg=max((r['tilt'] for r in full),default=None),settling_s=settled))
  previous=list(command)
 return metrics
