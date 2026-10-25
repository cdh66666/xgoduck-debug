"""Measure an old roll policy on corrected physics before choosing retraining.

This is a diagnostic baseline, never an accepted new policy or hardware test.
"""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from butt_sit import SitPlant,XML
from evaluate_skill_chain import predictor

def main():
    p=argparse.ArgumentParser();p.add_argument('--walk',type=Path,required=True);p.add_argument('--roll',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);plant=SitPlant();walk=predictor(a.walk);roll=predictor(a.roll);cases=[]
    for seed in range(7000,7008):
        rng=np.random.default_rng(seed);plant.sim.bam.actuator.vin=float(rng.uniform(7.2,8.1));plant.sim.model.geom_friction[:,0]=float(rng.uniform(.55,1.));plant.reset(seed);trace=[]
        for k in range(400):
            obs,_=plant.observation(0,0);stage='roll' if 100<=k<195 else 'stand';plant.advance((roll if stage=='roll' else walk)(obs));row=plant.measure();row['stage']=stage
            row['peak_torque_nm']=plant.peak_torque_nm;row['peak_speed_radps']=plant.peak_joint_speed_radps;trace.append(row)
            if not np.isfinite(plant.sim.data.qpos).all():break
        tail=trace[-50:];angles=np.unwrap(np.deg2rad([r['pitch_deg'] for r in trace]));turn=float(np.ptp(angles)*180/np.pi)
        metrics=dict(seed=seed,ticks=len(trace),pitch_rotation_span_deg=turn,end_pitch_deg=float(np.mean([r['pitch_deg'] for r in tail])),end_roll_deg=float(np.mean([r['roll_deg'] for r in tail])),
                     end_height_mm=float(np.mean([r['root_height_mm'] for r in tail])),end_speed_mps=float(np.mean([np.linalg.norm(r['velocity'][:3]) for r in tail])),
                     max_head_force_n=max(r['forces_n']['head'] for r in trace),max_other_force_n=max(r['forces_n']['other'] for r in trace),
                     max_torque_nm=max(r['peak_torque_nm'] for r in trace),max_joint_speed_radps=max(r['peak_speed_radps'] for r in trace))
        metrics['cycle_completed']=bool(len(trace)==400 and turn>300 and abs(metrics['end_pitch_deg'])<10 and abs(metrics['end_roll_deg'])<10 and metrics['end_height_mm']>120 and metrics['end_speed_mps']<.03)
        metrics['hardware_qualified']=False;cases.append(metrics);(a.output/f'trace-{seed}.json').write_text(json.dumps(trace));print(json.dumps(metrics),flush=True)
    report=dict(legacy_policy_not_retrained=True,hardware_commands_sent=False,cycles_completed=sum(c['cycle_completed'] for c in cases),total=len(cases),cases=cases,
                physics_sha256=hashlib.sha256(XML.read_bytes()).hexdigest(),roll_sha256=hashlib.sha256(a.roll.read_bytes()).hexdigest(),
                scope='Corrected-plant diagnostic baseline: 2s stand, 1.9s old roll, 4.1s walk hold; no reset, brace or impulse; contact loads do not constitute mechanical strength qualification')
    (a.output/'result.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
