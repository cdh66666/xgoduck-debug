"""Long native-physics qualification of a conservative local simulation baseline."""
import argparse,concurrent.futures,gzip,json,math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from stability_protocol import OnnxPolicy,new_episode,tick,duration,case_metrics,scenarios


def cases():
 zero=[0.,0.,0.]
 cycle=scenarios()['switches']['segments']
 return {
  'standing_60s':dict(title='连续站立 60 秒',segments=[(60.,zero)]),
  'backward_stop_12x':dict(title='连续后退→停止 12 次',segments=[(4.,[-.3,0.,0.]),(3.,zero)]*12),
  'mixed_126s':dict(title='连续前后走、转向、停止 126 秒',segments=cycle*6+[(6.,zero)]),
  'back_push_6x':dict(title='站立承受 6 次向后推力',segments=[(60.,zero)],
   pushes=[dict(t=t,duration=.16,force=[-3.,0.,0.]) for t in (5.,15.,25.,35.,45.,55.)])}


def trial(task):
 path,seed,key,folder=task;policy=OnnxPolicy(path);case=cases()[key]
 ticks=round(duration(case)/.02);env=new_episode(seed,max_episode_steps=ticks+1);rows=[]
 failure=None;max_substep_tilt=0.;max_substep_height_error=0.
 try:
  for index in range(ticks):
   row,done=tick(env,policy,case,index)
   rotation=env.sim.data.xmat[env.sim.root_id].reshape(3,3)
   row['backward_pitch_deg']=math.degrees(math.atan2(rotation[2,0],math.hypot(rotation[0,0],rotation[1,0])))
   max_substep_tilt=max(max_substep_tilt,float(env.sim.max_substep_tilt_deg))
   max_substep_height_error=max(max_substep_height_error,float(env.sim.max_substep_height_error_m))
   rows.append(row)
   if done:break
 except Exception as exc:
  failure=repr(exc)
 finally:env.close()
 metrics=case_metrics(rows,case)
 resets=max((r['resets'] for r in rows),default=0)-min((r['resets'] for r in rows),default=0)
 result=dict(seed=seed,scenario=key,**metrics,seconds=len(rows)*.02,
  max_backward_pitch_deg=max((r['backward_pitch_deg'] for r in rows),default=None),
  max_substep_tilt_deg=max_substep_tilt,max_substep_height_error_m=max_substep_height_error,
  resets_during_recording=resets,error=failure)
 result['passed']=bool(result['complete'] and not result['failure'] and not failure and resets==0
  and result['max_backward_pitch_deg']<=15. and max_substep_tilt<25. and max_substep_height_error<.025)
 with gzip.open(Path(folder)/f'{seed}-{key}.json.gz','wt',encoding='utf-8') as f:
  json.dump(dict(**result,rows=rows),f,ensure_ascii=False,allow_nan=False)
 return result


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--policy',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--seeds',type=int,default=8);p.add_argument('--first-seed',type=int,default=20261420)
 p.add_argument('--workers',type=int,default=2)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 policy=OnnxPolicy(a.policy);results=[]
 tasks=[(str(a.policy.resolve()),seed,key,str(a.output.resolve()))
  for seed in range(a.first_seed,a.first_seed+a.seeds) for key in cases()]
 with concurrent.futures.ProcessPoolExecutor(max_workers=a.workers) as executor:
  for result in executor.map(trial,tasks):
   results.append(result)
   print(json.dumps({k:v for k,v in result.items() if k!='stops'}),flush=True)
 report=dict(policy=str(a.policy.resolve()),sha256=policy.sha256,hardware_operated=False,
  scope='Native MuJoCo/BAM only; level original ground, original mass/actuators/filter/delay; no resets or recovery inside a case. A longer explicit evaluation timeout leaves all physical failure thresholds enabled.',
  criteria=dict(backward_pitch_max_deg=15.,substep_tilt_max_deg=25.,substep_height_error_max_m=.025,
   all_cases_complete=True,resets_allowed=0),seeds=a.seeds,first_seed=a.first_seed,
  cases=results,passed=sum(r['passed'] for r in results),total=len(results),
  simulated_seconds=sum(r['seconds'] for r in results))
 report['result']='PASS_SIMULATION_BASELINE' if report['passed']==report['total'] else 'NOT_PASSED'
 (a.output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
 print(json.dumps({k:v for k,v in report.items() if k!='cases'}),flush=True)


if __name__=='__main__':main()
