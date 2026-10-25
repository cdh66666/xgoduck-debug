"""Find a supported rise after a genuine butt sit in native physics, without resets.

This produces a closed-loop teacher candidate, not a trained or deployed model.
Rejected trajectories remain failures even when the optimizer reports success.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
from butt_sit import SitPlant,HOME,INDEX,smooth
from train_butt_sit import END,teacher as sit_teacher

def raise_teacher(obs,start,parameters,t):
    h,k,a,pitch_mid,split,duration,kp,kd=parameters
    mid=END.copy();mid[[2,3,4]]=[h,k,a];mid[[7,8,9]]=[-h,-k,-a]
    if t<split:
        u=smooth(t/split);target=END+(mid-END)*u;pitch_goal=-25+(pitch_mid+25)*u
    else:
        u=smooth((t-split)/(duration-split));target=mid+(start-mid)*u;pitch_goal=pitch_mid*(1-u)
    pitch=np.rad2deg(np.arctan2(obs[3],-obs[5]))
    correction=np.clip(kp*(pitch-pitch_goal)+kd*np.rad2deg(obs[1]),-25,25)
    target[4]=np.clip(target[4]+correction,-85,85)
    target[9]=np.clip(target[9]-correction,-85,85)
    return np.deg2rad(target[INDEX]-HOME).astype(np.float32)

def rollout(plant,parameters,seed=1000,variant=False):
    rng=np.random.default_rng(seed+7000)
    plant.sim.bam.actuator.vin=float(rng.uniform(7.2,8.1)) if variant else 7.4
    plant.sim.model.geom_friction[:,0]=float(rng.uniform(.55,1.)) if variant else .8
    plant.reset(seed);trace=[];targets=[];failed=None
    for tick in range(round((3+parameters[5]+2)*50)):
        t=tick*.02;obs,_=plant.observation(min(1,t),1 if t<3 else -1)
        raw=sit_teacher(obs,plant.start) if t<3 else raise_teacher(obs,plant.start,parameters,t-3)
        plant.advance(raw);row=plant.measure();row['stage']='sit' if t<3 else 'raise'
        trace.append(row);targets.append(raw.tolist())
        if row['head_clearance_mm']<40 or abs(row['pitch_deg'])>65 or abs(row['roll_deg'])>30:
            failed='tilt_or_head_clearance';break
    tail=trace[-50:];mean=lambda fn:float(np.mean([fn(r) for r in tail]))
    m=dict(seed=seed,ticks=len(trace),expected_ticks=round((3+parameters[5]+2)*50),
           head_clearance_mm=min(r['head_clearance_mm'] for r in trace),
           height_mm=mean(lambda r:r['root_height_mm']),pitch_deg=mean(lambda r:r['pitch_deg']),
           roll_deg=mean(lambda r:abs(r['roll_deg'])),
           velocity_mps=mean(lambda r:np.linalg.norm(r['velocity'][:3])),
           rotation_radps=mean(lambda r:np.linalg.norm(r['velocity'][3:])),
           butt_force_n=mean(lambda r:r['forces_n']['butt']),
           feet_force_n=mean(lambda r:r['forces_n']['feet']),
           unwanted_contact_n=max(r['forces_n']['head']+r['forces_n']['other'] for r in trace),
           target_step_deg=float(np.max(np.abs(np.diff(np.array(targets),axis=0)))*180/np.pi))
    failures=[]
    if failed:failures.append(failed)
    if not 120<m['height_mm']<155:failures.append('standing_height')
    if abs(m['pitch_deg'])>8 or m['roll_deg']>5:failures.append('standing_attitude')
    if m['velocity_mps']>.01 or m['rotation_radps']>.08:failures.append('not_settled')
    if m['butt_force_n']>.1 or m['feet_force_n']<6:failures.append('not_feet_supported')
    if m['unwanted_contact_n']>.1:failures.append('unwanted_contact')
    if m['head_clearance_mm']<60:failures.append('head_clearance')
    if m['target_step_deg']>6:failures.append('target_step')
    m.update(passed=not failures,failures=failures)
    score=(abs(m['height_mm']-140)/3+abs(m['pitch_deg'])+m['roll_deg']*2+
           m['velocity_mps']*300+m['rotation_radps']*20+m['butt_force_n']*10+
           max(0,6-m['feet_force_n'])*5+m['unwanted_contact_n']*10+
           max(0,60-m['head_clearance_mm'])+max(0,m['target_step_deg']-6)*10+
           (200+2*(m['expected_ticks']-m['ticks']) if failed else 0))
    return score,m,trace

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=1200);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);plant=SitPlant();began=time.monotonic();best=None;count=0
    bounds=[(-15,40),(20,85),(0,85),(-20,30),(.3,1.1),(1.5,3.5),(.15,.8),(.05,.35)]
    def objective(parameters):
        nonlocal best,count
        score,m,trace=rollout(plant,parameters);count+=1
        row=dict(evaluation=count,score=score,parameters=list(parameters),metrics=m)
        with (args.output/'search.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        if best is None or score<best['score']:
            best=row;(args.output/'best.json').write_text(json.dumps(best,indent=2))
            (args.output/'best-trace.json').write_text(json.dumps(trace));print(json.dumps(row),flush=True)
        return score
    # Cross-entropy trajectory search using the existing NumPy dependency.
    rng=np.random.default_rng(25);limits=np.asarray(bounds);span=limits[:,1]-limits[:,0]
    center=np.asarray([20,65,50,5,.7,2.5,.3,.2]);spread=span*.3
    objective(center)
    for generation in range(40):
        candidates=np.clip(rng.normal(center,spread,(48,len(bounds))),limits[:,0],limits[:,1])
        candidates[:6]=rng.uniform(limits[:,0],limits[:,1],(6,len(bounds)))
        candidates[6]=best['parameters']
        scores=np.array([objective(candidate) for candidate in candidates])
        elite=candidates[np.argsort(scores)[:10]]
        center=elite.mean(axis=0);spread=np.maximum(elite.std(axis=0),span*.025)
        if time.monotonic()-began>args.seconds:break
    results=[]
    for seed in range(2000,2016):
        _,m,trace=rollout(plant,best['parameters'],seed,variant=True);results.append(m)
        if seed==2000:(args.output/'evaluation-trace.json').write_text(json.dumps(trace))
    report=dict(teacher_only=True,hardware_commands_sent=False,parameters=best['parameters'],
                evaluations=results,passed=sum(m['passed'] for m in results),total=len(results),
                wall_s=time.monotonic()-began)
    (args.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)

if __name__=='__main__':main()
