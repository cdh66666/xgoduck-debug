"""Bounded native teacher search: shift weight, lift right foot, hold, return.

No hardware I/O. A good optimizer score is never a claim of a trained model.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
from butt_sit import SitPlant,HOME,INDEX,smooth

def teacher(obs,start,p,t):
    bias,roll_goal,rkp,rkd,pitch_goal,pkp,pkd,hip,knee,ankle=p
    shift=smooth(t/1.5)*(1-smooth((t-6.)/1.5))
    lift=smooth((t-1.5)/1.2)*(1-smooth((t-5.)/1.))
    target=start.copy();target[[1,6]]+=bias*shift
    target[[7,8,9]]+=(np.array([hip,knee,ankle])-start[[7,8,9]])*lift
    roll=np.rad2deg(np.arctan2(-obs[4],-obs[5]));pitch=np.rad2deg(np.arctan2(obs[3],-obs[5]))
    correction=np.clip(rkp*(roll-roll_goal*shift)+rkd*np.rad2deg(obs[0]),-15,15)
    target[[1,6]]=np.clip(target[[1,6]]+correction,-22,22)
    pitch_c=np.clip(pkp*(pitch-pitch_goal)+pkd*np.rad2deg(obs[1]),-20,20)
    target[4]+=pitch_c;target[9]-=pitch_c
    return np.deg2rad(target[INDEX]-HOME).astype(np.float32)

def rollout(plant,parameters,seed=0,variant=False):
    rng=np.random.default_rng(seed+6000)
    plant.sim.bam.actuator.vin=float(rng.uniform(7.2,8.1)) if variant else 7.4
    plant.sim.model.geom_friction[:,0]=float(rng.uniform(.55,1.)) if variant else .8
    plant.reset(seed);trace=[];failure=None
    for k in range(425):
        obs,_=plant.observation(0,0);plant.advance(teacher(obs,plant.start,parameters,k*.02));row=plant.measure()
        row['stage']='single_support';row['left_force_n']=sum(c['force_n'] for c in row['contacts'] if c['geom']=='left_foot_collision')
        row['right_force_n']=sum(c['force_n'] for c in row['contacts'] if c['geom']=='right_foot_collision')
        row['right_clearance_mm']=float(plant.vertices(plant.foot_ids[1])[:,2].min()*1000)
        row['peak_torque_nm']=plant.peak_torque_nm;row['peak_speed_radps']=plant.peak_joint_speed_radps
        trace.append(row)
        if row['head_clearance_mm']<50 or abs(row['roll_deg'])>40 or abs(row['pitch_deg'])>45:
            failure='tilt_or_head_clearance';break
    hold=[r for r in trace if 3.5<=r['t']<=5];tail=trace[-50:]
    avg=lambda key:float(np.mean([r[key] for r in hold])) if hold else 0.
    m=dict(seed=seed,ticks=len(trace),hold_samples=len(hold),right_clearance_mm=avg('right_clearance_mm'),
           right_force_n=avg('right_force_n'),left_force_n=avg('left_force_n'),
           single_support_fraction=float(np.mean([r['right_clearance_mm']>8 and r['right_force_n']<.1 and r['left_force_n']>6 for r in hold])) if hold else 0.,
           max_unwanted_force_n=max(r['forces_n']['butt']+r['forces_n']['head']+r['forces_n']['other'] for r in trace),
           min_head_clearance_mm=min(r['head_clearance_mm'] for r in trace),
           max_torque_nm=max(r['peak_torque_nm'] for r in trace),max_speed_radps=max(r['peak_speed_radps'] for r in trace),
           final_pitch_deg=float(np.mean([r['pitch_deg'] for r in tail])),final_roll_deg=float(np.mean([r['roll_deg'] for r in tail])),
           final_speed_mps=float(np.mean([np.linalg.norm(r['velocity'][:3]) for r in tail])))
    failures=[]
    if failure:failures.append(failure)
    if m['single_support_fraction']<.98:failures.append('right_foot_not_raised_and_left_supported')
    if m['max_unwanted_force_n']>.1:failures.append('unwanted_ground_support')
    if abs(m['final_pitch_deg'])>8 or abs(m['final_roll_deg'])>5 or m['final_speed_mps']>.01:failures.append('return_not_stable')
    m.update(passed=not failures,failures=failures)
    score=(100*(1-m['single_support_fraction'])+max(0,12-m['right_clearance_mm'])*3+m['right_force_n']*5+
           max(0,7-m['left_force_n'])*5+m['max_unwanted_force_n']*5+abs(m['final_pitch_deg'])+abs(m['final_roll_deg'])+
           m['final_speed_mps']*200+(100+425-len(trace) if failure else 0))
    return float(score),m,trace

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=1200);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);plant=SitPlant();start=time.monotonic();best=None;count=0
    limits=np.array([(0,18),(-25,-5),(.3,2),(.05,.6),(-3,8),(.1,1),(.05,.5),(-45,15),(-85,-25),(-85,-10)],float)
    center=np.array([8,-15,.8,.15,0,.4,.2,-10,-50,-35]);span=limits[:,1]-limits[:,0];spread=span*.25;rng=np.random.default_rng(26)
    def objective(values):
        nonlocal best,count
        score,metrics,trace=rollout(plant,values);count+=1;record=dict(evaluation=count,score=score,parameters=list(values),metrics=metrics)
        with (a.output/'search.jsonl').open('a') as stream:stream.write(json.dumps(record)+'\n')
        if best is None or score<best['score']:
            best=record;(a.output/'best.json').write_text(json.dumps(best,indent=2));(a.output/'best-trace.json').write_text(json.dumps(trace));print(json.dumps(record),flush=True)
        return score
    objective(center)
    for _ in range(60):
        samples=np.clip(rng.normal(center,spread,(40,len(center))),limits[:,0],limits[:,1]);samples[:5]=rng.uniform(limits[:,0],limits[:,1],(5,len(center)));samples[5]=best['parameters']
        scores=np.array([objective(v) for v in samples]);elite=samples[np.argsort(scores)[:8]];center=elite.mean(0);spread=np.maximum(elite.std(0),span*.025)
        if time.monotonic()-start>=a.seconds:break
    cases=[]
    for seed in range(5000,5016):
        _,metrics,trace=rollout(plant,best['parameters'],seed,variant=True);cases.append(metrics)
        if seed==5000:(a.output/'evaluation-trace.json').write_text(json.dumps(trace))
    report=dict(teacher_only=True,hardware_commands_sent=False,parameters=best['parameters'],passed=sum(c['passed'] for c in cases),total=len(cases),cases=cases,wall_s=time.monotonic()-start)
    (a.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)

if __name__=='__main__':main()
