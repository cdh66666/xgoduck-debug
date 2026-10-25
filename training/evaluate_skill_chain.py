"""Continuous walk/stop/sit/rise/walk test in one native plant, never reset mid-run."""
import argparse,hashlib,json,os
from pathlib import Path
import numpy as np
import onnxruntime as ort
from butt_sit import SitPlant,XML,smooth

def predictor(path):
    opts=ort.SessionOptions();opts.intra_op_num_threads=1
    session=ort.InferenceSession(str(path),sess_options=opts,providers=['CPUExecutionProvider'])
    name=session.get_inputs()[0].name
    return lambda obs:session.run(None,{name:obs.reshape(1,61)})[0][0]

def run(plant,walk,sit,rise_duration,seed,smooth_commands=False,*,sit_duration=1.,official_sit=False):
    rng=np.random.default_rng(seed)
    voltage=float(rng.uniform(7.2,8.1));friction=float(rng.uniform(.55,1.))
    plant.sim.bam.actuator.vin=voltage;plant.sim.model.geom_friction[:,0]=friction
    plant.reset(seed);resets_before=plant.sim.reset_calls
    stages=[('stand',2.,[0,0,0]),('forward',4.,[.2,0,0]),('stop',3.,[0,0,0]),
            ('sit',sit_duration+2.,[0,0,0]),('rise',rise_duration+2.,[0,0,0]),('stand_after',3.,[0,0,0]),
            ('backward',4.,[-.3,0,0]),('stop_final',3.,[0,0,0])]
    trace=[];metrics=[];failures=[];previous=plant.filtered.copy();handoffs=[];filtered_command=np.zeros(3);aborted_stage=None
    for name,duration,command in stages:
        rows=[];entry_target=plant.filtered.copy()
        for tick in range(round(duration*50)):
            phase=min(1.,tick*.02/(rise_duration if name=='rise' else sit_duration if name=='sit' else 1.))
            obs,_=plant.observation(phase,1 if name=='sit' else -1)
            if name not in ('sit','rise'):
                filtered_command+=np.clip(np.asarray(command)-filtered_command,-np.array([1.,1.,3.])*.02,np.array([1.,1.,3.])*.02) if smooth_commands else np.asarray(command)-filtered_command
                obs[55:61]=0.;obs[48:51]=filtered_command
            action=(sit if name in ('sit','rise') else walk)(obs)
            # Carry filter/history through skills. Only the actor identity switch
            # blends from its current target; command changes use acceleration.
            if smooth_commands and (name=='stand_after' or name=='sit' and not official_sit) and tick<20:
                action=entry_target+(action-entry_target)*smooth((tick+1)/20)
            plant.advance(action);row=plant.measure();row.update(stage=name,stage_t=(tick+1)*.02,command=command)
            row['target_step_deg']=float(np.max(np.abs(plant.filtered-previous))*180/np.pi)
            row['target_step_joint']=int(np.argmax(np.abs(plant.filtered-previous)))
            row['joint_velocity_radps']=plant.sim.data.qvel[plant.sim.motor.dof_indexes].tolist()
            row['motor_torque_nm']=plant.sim.data.actuator_force[plant.sim.motor.act_indexes].tolist()
            row['requested_motor_torque_nm']=plant.sim.data.ctrl[plant.sim.motor.act_indexes].tolist()
            row['substep_peak_torque_nm']=plant.peak_torque_nm
            row['substep_peak_joint_speed_radps']=plant.peak_joint_speed_radps
            if official_sit:
                from sit_reference import SEATED_GEOMS
                row.update(policy_target_deg=np.rad2deg(plant.filtered).tolist(),joint_deg=np.rad2deg(plant.sim.data.qpos[plant.sim.qadr]).tolist(),
                    seat_support_n=sum(c['force_n'] for c in row['contacts'] if c['geom'] in SEATED_GEOMS),
                    upper_contact_n=sum(c['force_n'] for c in row['contacts'] if c['geom'] not in SEATED_GEOMS and c['kind'] not in ('feet','butt')),
                    peak_torque_nm=plant.peak_torque_nm,peak_joint_speed_radps=plant.peak_joint_speed_radps,
                    substep_peak_forces_n=plant.peak_contact_forces_n.copy(),substep_impulse_ns=plant.contact_impulse_ns.copy())
            if tick==0:handoffs.append(dict(stage=name,target_step_deg=row['target_step_deg']))
            previous=plant.filtered.copy()
            v=np.zeros(6);plant.mj.mj_objectVelocity(plant.sim.model,plant.sim.data,plant.mj.mjtObj.mjOBJ_XBODY,plant.sim.root_id,v,1)
            row['body_velocity']=[float(v[3]),float(v[4]),float(v[2])]
            trace.append(row);rows.append(row)
            is_official_phase=official_sit and name in ('sit','rise')
            if row['head_clearance_mm']<40 or abs(row['pitch_deg'])>(55 if is_official_phase else 65) or abs(row['roll_deg'])>(25 if is_official_phase else 30):
                failures.append(name+':tilt_or_head_clearance');aborted_stage=name;break
        tail=rows[-50:];vel=np.array([r['body_velocity'] for r in tail]);forces=np.array([[r['forces_n'][k] for k in ('butt','feet','head','other')] for r in tail])
        m=dict(stage=name,ticks=len(rows),expected_ticks=round(duration*50),mean_velocity=vel.mean(0).tolist(),rmse=np.sqrt(((vel-command)**2).mean(0)).tolist(),
               mean_forces_n=forces.mean(0).tolist(),height_mm=float(np.mean([r['root_height_mm'] for r in tail])),
               pitch_deg=float(np.mean([r['pitch_deg'] for r in tail])),
               max_target_step_deg=max(r['target_step_deg'] for r in rows),
               entry_window_target_step_deg=max(r['target_step_deg'] for r in rows[:20]))
        metrics.append(m)
        if official_sit and name in ('sit','rise'):
            from evaluate_official_sit import score_stage
            m['official_metrics']=score_stage(rows,name=='sit',seed)
            failures.extend(name+':'+f for f in m['official_metrics']['failures'])
        elif name=='sit':
            if np.mean(forces[:,0]>.5)<.98 or min(m['mean_forces_n'][:2])<1:failures.append('sit:not_supported')
        elif name in ('forward','backward'):
            target=command[0]
            if not .8<=m['mean_velocity'][0]/target<=1.2 or m['rmse'][0]>.05 or m['rmse'][1]>.05 or m['rmse'][2]>.2:failures.append(name+':tracking')
        else:
            if not 120<m['height_mm']<155 or abs(m['pitch_deg'])>8 or m['mean_forces_n'][0]>.1 or m['mean_forces_n'][1]<6:failures.append(name+':stance')
            if np.any(np.linalg.norm(vel[:,:2],axis=1)>.03) or np.any(np.abs(vel[:,2])>.08):failures.append(name+':not_stopped')
        if aborted_stage is not None:break
    if any((r['upper_contact_n'] if official_sit and r['stage'] in ('sit','rise') else r['forces_n']['head']+r['forces_n']['other'])>.1 for r in trace):failures.append('unwanted_ground_contact')
    if min(r['head_clearance_mm'] for r in trace)<60:failures.append('head_clearance')
    if max((m['entry_window_target_step_deg'] for m in metrics[1:]),default=0)>6:failures.append('handoff_target_step')
    reset_count=plant.sim.reset_calls-resets_before
    if reset_count:failures.append('unexpected_reset')
    result=dict(seed=seed,passed=not failures,failures=failures,stages=metrics,handoffs=handoffs,
                aborted_stage=aborted_stage,complete=len(trace)==sum(round(duration*50) for _,duration,_ in stages),
                resets_during_recording=reset_count,policy_steps=len(trace),sim_time_s=float(plant.sim.data.time),
                control_hz=50,smooth_commands=smooth_commands,official_sit=official_sit,sit_duration_s=sit_duration,voltage_v=voltage,friction=friction,min_head_clearance_mm=min(r['head_clearance_mm'] for r in trace))
    return result,trace

def main():
    p=argparse.ArgumentParser();p.add_argument('--walk',type=Path,required=True)
    source=p.add_mutually_exclusive_group(required=True);source.add_argument('--sit-directory',type=Path);source.add_argument('--official-sit-model',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seeds',type=int,default=8)
    p.add_argument('--smooth-commands',action='store_true');a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    if a.official_sit_model:
        from sit_reference import SIT_DURATION,RISE_DURATION
        opts=ort.SessionOptions();opts.intra_op_num_threads=1
        session=ort.InferenceSession(str(a.official_sit_model),sess_options=opts,providers=['CPUExecutionProvider']);meta=session.get_modelmeta().custom_metadata_map
        if meta.get('role')!='official-reference-sit-rise-candidate' or meta.get('physical_model_sha256',meta.get('physics_sha256'))!=hashlib.sha256(XML.read_bytes()).hexdigest():raise ValueError('Official sit model contract mismatch')
        if any(float(meta.get(k,'nan'))!=v for k,v in [('phase_index',55),('direction_index',56),('sit_duration_s',SIT_DURATION),('rise_duration_s',RISE_DURATION)]):raise ValueError('Official sit phase contract mismatch')
        cfg=dict(rise_duration_s=RISE_DURATION,sit_duration_s=SIT_DURATION);model=a.official_sit_model
        os.environ['XGODUCK_RUNTIME_PROFILE']='calibrated-20261025'
    else:
        cfg=json.loads((a.sit_directory/'config.json').read_text());model=a.sit_directory/'sit-rise.onnx'
        if cfg['physics_model_sha256']!=hashlib.sha256(XML.read_bytes()).hexdigest():raise ValueError('Sit model physics mismatch')
    plant=SitPlant();walk=predictor(a.walk);sit=predictor(model);cases=[]
    for seed in range(4000,4000+a.seeds):
        metrics,trace=run(plant,walk,sit,cfg['rise_duration_s'],seed,a.smooth_commands,sit_duration=cfg.get('sit_duration_s',1.),official_sit=bool(a.official_sit_model));cases.append(metrics)
        if seed==4000 or not metrics['passed']:(a.output/f'trace-{seed}.json').write_text(json.dumps(trace))
        print(json.dumps(metrics),flush=True)
    result=dict(passed=sum(c['passed'] for c in cases),total=len(cases),cases=cases,hardware_commands_sent=False,
                walk_sha256=hashlib.sha256(a.walk.read_bytes()).hexdigest(),sit_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                scope='One plant and continuous action/filter/delay state; no reset at skill changes; initial pose from standing reference')
    (a.output/'result.json').write_text(json.dumps(result,indent=2))

if __name__=='__main__':main()
