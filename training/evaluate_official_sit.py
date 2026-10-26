"""Independent native-physics gates for an official-reference sit/rise ONNX actor.

The geometry follows upstream SitStand, not a confirmed user video. The same
20/25deg tilt and 6deg/20ms target-step gates are retained from the teacher search.
No teacher targets, output corrections, phase-boundary reset or hardware I/O.
"""
import argparse,hashlib,json,os,time
from pathlib import Path
import numpy as np
import onnxruntime as ort
from butt_sit import SitPlant,XML
from sit_reference import (SIT_DURATION,RISE_DURATION,SIT_STEPS,RISE_STEPS,SEAT_DEG,LEG_INDICES,SEATED_GEOMS)


def score_stage(rows,sitting,seed):
    tail=rows[-50:];mean=lambda key:float(np.mean([r[key] for r in tail]))
    target_step=float(np.max(np.abs(np.diff([r['policy_target_deg'] for r in rows],axis=0)))) if len(rows)>1 else 0.
    upper=max(r['upper_contact_n'] for r in rows)
    m=dict(seed=seed,ticks=len(rows),expected_ticks=SIT_STEPS if sitting else RISE_STEPS,
        control_hz=50,physics_hz=200,head_min_mm=min(r['head_clearance_mm'] for r in rows),
        pitch_peak_deg=max(abs(r['pitch_deg']) for r in rows),pitch_tail_deg=mean('pitch_deg'),
        height_tail_mm=mean('root_height_mm'),linear_speed_mps=float(np.mean([np.linalg.norm(r['velocity'][:3]) for r in tail])),
        angular_speed_radps=float(np.mean([np.linalg.norm(r['velocity'][3:]) for r in tail])),
        target_step_deg=target_step,upper_contact_peak_n=upper,rear_cover_peak_n=max(r['forces_n']['butt'] for r in rows),
        head_substep_peak_n=max(r['substep_peak_forces_n']['head'] for r in rows),
        peak_motor_torque_nm=max(r['peak_torque_nm'] for r in rows),peak_joint_speed_radps=max(r['peak_joint_speed_radps'] for r in rows))
    failures=[]
    if len(rows)!=m['expected_ticks']:failures.append('incomplete')
    if m['head_min_mm']<60 or m['pitch_peak_deg']>(20 if sitting else 25):failures.append('head_or_tilt')
    if upper>.1:failures.append('upper_body_contact')
    if m['head_substep_peak_n']>.1:failures.append('substep_head_contact')
    if sitting:
        error=float(np.max(np.abs(np.mean([r['joint_deg'] for r in tail],axis=0)[LEG_INDICES]-SEAT_DEG[LEG_INDICES])))
        m.update(pose_error_deg=error,seat_support_n=mean('seat_support_n'))
        if not 75<m['height_tail_mm']<95 or error>8:failures.append('official_pose')
        if m['seat_support_n']<1:failures.append('seat_support')
    else:
        m.update(feet_force_n=float(np.mean([r['forces_n']['feet'] for r in tail])),
                 other_tail_force_n=float(np.mean([sum(r['forces_n'][k] for k in ('butt','head','other')) for r in tail])))
        if not 120<m['height_tail_mm']<155 or abs(m['pitch_tail_deg'])>8:failures.append('stand_pose')
        if m['other_tail_force_n']>.1 or m['feet_force_n']<6:failures.append('not_feet_supported')
    if m['linear_speed_mps']>.005 or m['angular_speed_radps']>.04:failures.append('not_settled')
    if target_step>6:failures.append('target_step')
    m.update(passed=not failures,failures=failures);return m


def cycle(plant,predict,seed,variation=None,*,entry=None):
    rng=np.random.default_rng(seed)
    if variation:variation.before_reset()
    else:
        plant.sim.bam.actuator.vin=float(rng.uniform(7.2,8.1));plant.sim.model.geom_friction[:,0]=float(rng.uniform(.55,1))
    plant.reset(seed)
    if variation:plant.delay=[plant.delay[0].copy() for _ in range(variation.sample['transport_delay_physics_steps'])]
    resets=plant.sim.reset_calls;begin=float(plant.sim.data.time)
    if entry is not None:entry(plant)
    if plant.sim.reset_calls!=resets:raise ValueError('Entry must preserve physical state')
    entry_duration=float(plant.sim.data.time-begin)
    trace=[];metrics={};all_targets=[np.rad2deg(plant.filtered).copy()]
    for stage,duration,steps,direction in [('sit',SIT_DURATION,SIT_STEPS,1.),('rise',RISE_DURATION,RISE_STEPS,-1.)]:
        rows=[]
        for tick in range(steps):
            obs,_=plant.observation(min(tick*.02/duration,1.),direction);action=predict(obs)
            plant.advance(action);r=plant.measure()
            r.update(stage=stage,policy_target_deg=np.rad2deg(plant.filtered).tolist(),joint_deg=np.rad2deg(plant.sim.data.qpos[plant.sim.qadr]).tolist(),
                seat_support_n=sum(c['force_n'] for c in r['contacts'] if c['geom'] in SEATED_GEOMS),
                upper_contact_n=sum(c['force_n'] for c in r['contacts'] if c['geom'] not in SEATED_GEOMS and c['kind'] not in ('feet','butt')),
                peak_torque_nm=plant.peak_torque_nm,peak_joint_speed_radps=plant.peak_joint_speed_radps,
                substep_peak_forces_n=plant.peak_contact_forces_n.copy(),substep_impulse_ns=plant.contact_impulse_ns.copy())
            rows.append(r);all_targets.append(np.rad2deg(plant.filtered).copy())
            if abs(r['pitch_deg'])>55 or abs(r['roll_deg'])>25 or r['head_clearance_mm']<40:break
        trace.extend(rows);metrics[stage]=score_stage(rows,stage=='sit',seed)
        if not metrics[stage]['passed']:break
    assert plant.sim.reset_calls==resets
    step=float(np.max(np.abs(np.diff(all_targets,axis=0))))
    passed='rise' in metrics and all(m['passed'] for m in metrics.values()) and step<=6
    return dict(passed=passed,stage='complete' if passed else stage,**metrics,
        cycle_max_target_step_deg=step,resets_between_skills=0,entry_duration_s=entry_duration),trace


def main():
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seeds',type=int,default=32);p.add_argument('--first-seed',type=int,default=21000);p.add_argument('--randomize-physics',action='store_true')
    p.add_argument('--entry-walk',type=Path,help='Run the fixed 9s walking-stop prefix before sitting, without resetting its state')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    options=ort.SessionOptions();options.intra_op_num_threads=1
    session=ort.InferenceSession(str(a.model),sess_options=options,providers=['CPUExecutionProvider']);name=session.get_inputs()[0].name
    meta=session.get_modelmeta().custom_metadata_map;physical_hash=hashlib.sha256(XML.read_bytes()).hexdigest()
    if meta.get('role')!='official-reference-sit-rise-candidate' or meta.get('physical_model_sha256',meta.get('physics_sha256'))!=physical_hash:raise ValueError('Wrong policy role or physical model')
    if any(float(meta.get(k,'nan'))!=v for k,v in [('phase_index',55),('direction_index',56),('sit_duration_s',SIT_DURATION),('rise_duration_s',RISE_DURATION)]):raise ValueError('Wrong phase contract')
    latencies=[]
    def predict(obs):
        start=time.perf_counter();value=session.run(None,{name:obs.reshape(1,61)})[0][0];latencies.append((time.perf_counter()-start)*1000);return value
    os.environ['XGODUCK_RUNTIME_PROFILE']='calibrated-20261025'
    plant=SitPlant();variation=None;entry=None;entry_sha=None
    if a.entry_walk:
        from sit_entry import load_walk,advance_walk_stop
        entry_sha=hashlib.sha256(a.entry_walk.read_bytes()).hexdigest()
        walk=load_walk(dict(path=str(a.entry_walk),sha256=entry_sha),dict(sha256=physical_hash))
        entry=lambda current:advance_walk_stop(current,walk)
    if a.randomize_physics:
        from physics_variation import PhysicsVariation
        variation=PhysicsVariation(plant.sim,a.first_seed)
    results=[]
    try:
        for seed in range(a.first_seed,a.first_seed+a.seeds):
            m,trace=cycle(plant,predict,seed,variation,entry=entry);results.append(m)
            if variation:m['physics_sample']=variation.sample.copy()
            if seed==a.first_seed or not m['passed']:(a.output/f'trace-{seed}.json').write_text(json.dumps(trace))
    finally:plant.adapter.close()
    report=dict(passed=sum(m['passed'] for m in results),total=len(results),cases=results,
        scope='official keyframe morphology; exact user video not confirmed; offline only',
        uncertainty='COM/inertia/delay stress envelope' if variation else 'initial pose, voltage 7.2-8.1V, friction .55-1.0',
        physical_model_sha256=physical_hash,onnx_sha256=hashlib.sha256(a.model.read_bytes()).hexdigest(),
        evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),local_onnx_p99_ms=float(np.percentile(latencies,99)),
        control_hz=50,feedback_hz=100,physics_hz=200,head_contact_sampling_hz=200,runtime_profile='calibrated-20261025',
        entry_walk_sha256=entry_sha,hardware_commands_sent=False,deployment_allowed=False)
    (a.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='cases'}))

if __name__=='__main__':main()
