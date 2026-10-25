"""Paired old/new roll evaluation with continuous landing and 200Hz contacts."""
import argparse,hashlib,json,os
from pathlib import Path
import numpy as np
from butt_sit import XML
from native_env import manifest
from native_skill_env import NativeSkillEpisode
from evaluate_skill_chain import predictor

def case(frozen,seed,roll,walk):
    env=NativeSkillEpisode(frozen,seed,auto_reset=False);trace=[];resets=env.total_resets
    for tick in range(295):
        obs=env.observe()
        if tick<95:
            _,_,done,_,info=env.step(roll(obs))
        else:env.plant.advance(walk(obs))
        row=env.plant.measure();row['stage']='roll' if tick<95 else 'stand'
        row['peak_contacts_n']=env.plant.peak_contact_forces_n.copy();row['contact_impulse_ns']=env.plant.contact_impulse_ns.copy()
        row['peak_torque_nm']=env.plant.peak_torque_nm;row['peak_requested_torque_nm']=env.plant.peak_command_torque_nm;row['peak_joint_speed_radps']=env.plant.peak_joint_speed_radps
        trace.append(row)
    tail=trace[-50:];metrics=dict(seed=seed,roll_rotation_deg=info['roll_rotation_deg'],
        end_pitch_deg=float(np.mean([r['pitch_deg'] for r in tail])),end_roll_deg=float(np.mean([r['roll_deg'] for r in tail])),
        end_height_mm=float(np.mean([r['root_height_mm'] for r in tail])),end_speed_mps=float(np.mean([np.linalg.norm(r['velocity'][:3]) for r in tail])),
        peak_head_force_n=max(r['peak_contacts_n']['head'] for r in trace),head_impulse_ns=sum(r['contact_impulse_ns']['head'] for r in trace),
        peak_torque_nm=max(r['peak_torque_nm'] for r in trace),peak_requested_torque_nm=max(r['peak_requested_torque_nm'] for r in trace),
        peak_joint_speed_radps=max(r['peak_joint_speed_radps'] for r in trace),resets_during_recording=env.total_resets-resets,policy_steps=len(trace),sim_seconds=float(env.sim.data.time))
    metrics['cycle_completed']=bool(abs(metrics['roll_rotation_deg']-360)<30 and abs(metrics['end_pitch_deg'])<10 and abs(metrics['end_roll_deg'])<10 and metrics['end_height_mm']>120 and metrics['end_speed_mps']<.03)
    if env.variation:metrics['physics_sample']=env.variation.sample.copy()
    env.close();return metrics,trace

def main():
    p=argparse.ArgumentParser();p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True);p.add_argument('--walk',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seeds',type=int,default=16);p.add_argument('--randomize-physics',action='store_true');a=p.parse_args()
    os.environ['XGODUCK_TRAIN_XML']=str(XML);os.environ['XGODUCK_RUNTIME_PROFILE']='calibrated-20261025';os.environ['XGODUCK_RANDOMIZE_PHYSICS']='1' if a.randomize_physics else '0'
    a.output.mkdir(parents=True,exist_ok=False);frozen=manifest();walk=predictor(a.walk);results={}
    for name,model in [('before',a.before),('after',a.after)]:
        policy=predictor(model);rows=[]
        for seed in range(8000,8000+a.seeds):
            metrics,trace=case(frozen,seed,policy,walk);rows.append(metrics)
            if seed==8000 or not metrics['cycle_completed']:(a.output/f'{name}-{seed}.json').write_text(json.dumps(trace))
        results[name]=dict(cycles_completed=sum(m['cycle_completed'] for m in rows),total=len(rows),cases=rows,
            head_peak_p95_n=float(np.percentile([m['peak_head_force_n'] for m in rows],95)),head_impulse_mean_ns=float(np.mean([m['head_impulse_ns'] for m in rows])),onnx_sha256=hashlib.sha256(model.read_bytes()).hexdigest())
    old,new=results['before'],results['after']
    results.update(hardware_qualified=False,hardware_commands_sent=False,control_hz=50,contact_sampling_hz=200,
        scope='Paired same seeds and physics, 1.9s roll then 4s walk hold, no reset. Torque separates requested ctrl and applied force-limited torque.',
        impact_reduction_passed=bool(new['cycles_completed']==new['total'] and new['head_peak_p95_n']<=.9*old['head_peak_p95_n'] and new['head_impulse_mean_ns']<=.9*old['head_impulse_mean_ns']))
    (a.output/'result.json').write_text(json.dumps(results,indent=2));print(json.dumps({k:v for k,v in results.items() if k not in ('before','after')}))

if __name__=='__main__':main()
