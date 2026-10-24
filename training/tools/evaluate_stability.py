"""Measure native standing/perturbation/stop transients, never resetting a case."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from stability_protocol import OnnxPolicy,scenarios,duration,new_episode,tick,case_metrics


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--policy',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--seeds',type=int,default=8);p.add_argument('--first-seed',type=int,default=20261120)
 p.add_argument('--push-n',type=float,default=.8)
 p.add_argument('--scenarios',default='stand,slow_stop,forward_stop,backward_stop,left_stop,right_stop,switches,push_front,push_back,push_left,push_right')
 a=p.parse_args()
 if a.output.exists():p.error('Use a new output path')
 if a.seeds<1 or not 0<=a.push_n<=3:p.error('Invalid seed count or local force')
 policy=OnnxPolicy(a.policy);results=[]
 for seed in range(a.first_seed,a.first_seed+a.seeds):
  for key in a.scenarios.split(','):
   case=scenarios(a.push_n)[key];env=new_episode(seed);rows=[]
   try:
    for index in range(round(duration(case)/.02)):
     row,done=tick(env,policy,case,index);rows.append(row)
     if done:break
   finally:env.close()
   metrics=case_metrics(rows,case)
   result=dict(seed=seed,scenario=key,**metrics,resets_during_recording=max((r['resets'] for r in rows),default=0)-min((r['resets'] for r in rows),default=0),rows=rows)
   results.append(result)
   print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
 report=dict(policy_sha256=policy.sha256,policy=str(a.policy),seeds=a.seeds,first_seed=a.first_seed,push_n=a.push_n,
  scope='Native MuJoCo/BAM only; each case has no reset or recovery; raw per-tick data retained. Not hardware qualification.',
  physical_failures=sum(not r['complete'] or r['failure'] is not None for r in results),cases=results)
 a.output.parent.mkdir(parents=True,exist_ok=True)
 with a.output.open('x',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
 print(json.dumps({'output':str(a.output),'cases':len(results),'physical_failures':report['physical_failures']}),flush=True)


if __name__=='__main__':main()
