"""Small supported head/body dance in native physics; not an acrobatic skill."""
import numpy as np
from butt_sit import SitPlant,HOME,INDEX,smooth

DURATION=10.
def teacher(obs,start):
    t=float(obs[55])*DURATION;envelope=smooth(t/.8)*(1-smooth((t-8.2)/.8));wave=2*np.pi*t/2.
    target=start.copy()
    target[10]+=10*np.sin(2*wave)*envelope
    target[11]+=8*np.cos(2*wave)*envelope
    target[12]+=30*np.sin(wave)*envelope
    target[13]+=12*np.sin(wave+.6)*envelope
    target[[1,6]]+=10*np.sin(wave)*envelope
    pitch=np.rad2deg(np.arctan2(obs[3],-obs[5]));roll=np.rad2deg(np.arctan2(-obs[4],-obs[5]))
    pc=np.clip(.8*pitch+.2*np.rad2deg(obs[1]),-20,20)
    rc=np.clip(.6*roll+.15*np.rad2deg(obs[0]),-12,12)
    target[4]+=pc;target[9]-=pc;target[[1,6]]=np.clip(target[[1,6]]+rc,-22,22)
    return np.deg2rad(target[INDEX]-HOME).astype(np.float32)

def rollout(plant,seed,predict=None,record=False):
    rng=np.random.default_rng(seed+8000);plant.sim.bam.actuator.vin=float(rng.uniform(7.2,8.1));plant.sim.model.geom_friction[:,0]=float(rng.uniform(.55,1.))
    plant.reset(seed);trace=[];xs=[];ys=[];before=plant.filtered.copy();failed=None
    for k in range(round(DURATION*50)):
        obs,_=plant.observation(k*.02/DURATION,0);label=teacher(obs,plant.start)
        action=label if predict is None else predict(obs);plant.advance(action);row=plant.measure();row['stage']='dance'
        row['target_step_deg']=float(np.max(np.abs(plant.filtered-before))*180/np.pi);before=plant.filtered.copy()
        row['peak_torque_nm']=plant.peak_torque_nm;row['peak_speed_radps']=plant.peak_joint_speed_radps
        row['head_yaw_deg']=float(np.rad2deg(plant.sim.data.qpos[plant.sim.qadr[7]]))
        row['head_pitch_deg']=float(np.rad2deg(plant.sim.data.qpos[plant.sim.qadr[6]]))
        xs.append(obs);ys.append(label);trace.append(row)
        if row['head_clearance_mm']<60 or abs(row['roll_deg'])>25 or abs(row['pitch_deg'])>25:failed='tilt_or_head_clearance';break
    tail=trace[-50:];middle=[r for r in trace if 2<=r['t']<=8]
    amplitude=lambda key:float(np.ptp([r[key] for r in middle])) if middle else 0.
    m=dict(seed=seed,ticks=len(trace),head_yaw_range_deg=amplitude('head_yaw_deg'),head_pitch_range_deg=amplitude('head_pitch_deg'),body_roll_range_deg=amplitude('roll_deg'),
           head_min_mm=min(r['head_clearance_mm'] for r in trace),unwanted_contact_n=max(r['forces_n']['head']+r['forces_n']['butt']+r['forces_n']['other'] for r in trace),
           final_speed_mps=float(np.mean([np.linalg.norm(r['velocity'][:3]) for r in tail])),final_rotation_radps=float(np.mean([np.linalg.norm(r['velocity'][3:]) for r in tail])),
           max_target_step_deg=max(r['target_step_deg'] for r in trace),max_torque_nm=max(r['peak_torque_nm'] for r in trace),max_speed_radps=max(r['peak_speed_radps'] for r in trace))
    failures=[]
    if failed:failures.append(failed)
    if m['head_yaw_range_deg']<40 or m['head_pitch_range_deg']<10 or m['body_roll_range_deg']<2:failures.append('dance_motion_missing')
    if m['head_min_mm']<90 or m['unwanted_contact_n']>.1:failures.append('unwanted_ground_contact')
    if m['final_speed_mps']>.01 or m['final_rotation_radps']>.08:failures.append('not_settled')
    if m['max_target_step_deg']>6:failures.append('target_step')
    m.update(passed=not failures,failures=failures)
    return np.asarray(xs,dtype=np.float32),np.asarray(ys,dtype=np.float32),m,trace
