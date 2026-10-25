"""Fit and evaluate an isolated butt-supported sit actor in native BAM physics.

No robot transports are imported. The exported candidate is sit-only: phase 55
is elapsed local time / 1 s, and the host must keep calling it at 50 Hz in hold.
"""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
from butt_sit import SitPlant,HOME,INDEX,XML,SEED_RECORD,smooth

DURATION=1.0
END=np.array([0,0,31.9232,75.98945,69.06623,0,0,-31.9232,-75.98945,-69.06623,20,20,0,0,0])
FEATURES=[1,3,5,55]

def teacher(obs,start):
    u=smooth((float(obs[55])-.2)/.8)
    q=start+(END-start)*u
    pitch=np.rad2deg(np.arctan2(obs[3],-obs[5]))
    correction=np.clip(.3*(pitch+25*u)+.2*np.rad2deg(obs[1]),-25,25)
    q[4]=np.clip(q[4]+correction,-85,85);q[9]=np.clip(q[9]-correction,-85,85)
    return np.deg2rad(q[INDEX]-HOME).astype(np.float32)

def rollout(plant,seed,predict=None,record=False,variant=True,duration=5.,phase_duration=DURATION):
    rng=np.random.default_rng(seed+7000)
    voltage=float(rng.uniform(7.2,8.1)) if variant else 7.4
    friction=float(rng.uniform(.55,1.)) if variant else .8
    plant.sim.bam.actuator.vin=voltage
    plant.sim.model.geom_friction[:,0]=friction
    plant.reset(seed);xs=[];ys=[];trace=[];latencies=[];targets=[]
    for tick in range(round(duration*50)):
        obs,g=plant.observation(min(1.,tick*.02/phase_duration),1.)
        label=teacher(obs,plant.start);xs.append(obs);ys.append(label)
        at=time.perf_counter();action=label if predict is None else predict(obs)
        latencies.append((time.perf_counter()-at)*1000)
        targets.append(np.rad2deg(action)+HOME)
        plant.advance(action);row=plant.measure();trace.append(row)
        if row['head_clearance_mm']<20 or abs(row['pitch_deg'])>65 or abs(row['roll_deg'])>30:break
    tail=trace[-100:];mean=lambda fn:float(np.mean([fn(r) for r in tail]))
    seated=[r['t'] for r in trace if r['forces_n']['butt']>.5]
    metrics={'seed':seed,'voltage_v':voltage,'friction':friction,'ticks':len(trace),'simulation_seconds':trace[-1]['t'],
             'inference_hz':len(trace)/trace[-1]['t'],'inference_mean_ms':float(np.mean(latencies)),
             'inference_p99_ms':float(np.percentile(latencies,99)),'inference_max_ms':float(max(latencies)),
             'first_butt_contact_s':seated[0] if seated else None,
             'butt_contact_fraction_last_2s':mean(lambda r:r['forces_n']['butt']>.5),
             'butt_force_n':mean(lambda r:r['forces_n']['butt']),
             'feet_force_n':mean(lambda r:r['forces_n']['feet']),
             'other_contact_peak_n':max(r['forces_n']['other']+r['forces_n']['head'] for r in trace),
             'butt_force_peak_sampled_n':max(r['forces_n']['butt'] for r in trace),
             'min_head_clearance_mm':min(r['head_clearance_mm'] for r in trace),
             'butt_clearance_mm':mean(lambda r:r['butt_clearance_mm']),
             'pitch_deg':mean(lambda r:r['pitch_deg']),
             'max_roll_deg':max(abs(r['roll_deg']) for r in trace),
             'settled_speed_m_s':mean(lambda r:np.linalg.norm(r['velocity'][:3])),
             'settled_rotation_rad_s':mean(lambda r:np.linalg.norm(r['velocity'][3:])),
             'max_leg_target_deg':float(np.max(np.abs(np.array(targets)[:,[0,1,2,3,4,9,10,11,12,13]])))}
    failures=[]
    if len(trace)!=round(duration*50):failures.append('fell_or_head_too_low')
    if metrics['butt_contact_fraction_last_2s']<.98 or metrics['butt_force_n']<1:failures.append('butt_not_supported')
    if metrics['feet_force_n']<1:failures.append('feet_not_supported')
    if metrics['other_contact_peak_n']>.1:failures.append('wrong_part_hit_floor')
    if metrics['min_head_clearance_mm']<60:failures.append('head_clearance')
    if metrics['settled_speed_m_s']>.005 or metrics['settled_rotation_rad_s']>.04:failures.append('not_settled')
    if metrics['max_leg_target_deg']>90:failures.append('joint_target_limit')
    metrics.update(passed=not failures,failures=failures)
    return np.array(xs),np.array(ys),metrics,trace if record else []

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--epochs',type=int,default=200)
    args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    plant=SitPlant();x=[];y=[];teachers=[]
    for seed in range(64):
        a,b,m,_=rollout(plant,seed);teachers.append(m)
        if m['passed']:x.append(a);y.append(b)
    (out/'teacher-evaluation.json').write_text(json.dumps(teachers,indent=2))
    print(json.dumps({'teacher_passed':sum(m['passed'] for m in teachers),'total':len(teachers)}),flush=True)
    if len(x)<58:raise RuntimeError('Teacher failed the randomized native-contact gate')
    x=np.concatenate(x);y=np.concatenate(y);np.savez_compressed(out/'demonstrations.npz',observations=x,actions=y)
    import torch
    torch.set_num_threads(2);torch.set_num_interop_threads(1);torch.manual_seed(23)
    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__();self.register_buffer('features',torch.tensor(FEATURES))
            self.register_buffer('mean',torch.from_numpy(x[:,FEATURES].mean(0)))
            self.register_buffer('std',torch.from_numpy(x[:,FEATURES].std(0).clip(.05)))
            self.net=torch.nn.Sequential(torch.nn.Linear(len(FEATURES),96),torch.nn.Tanh(),torch.nn.Linear(96,96),torch.nn.Tanh(),torch.nn.Linear(96,14))
        def forward(self,obs):return self.net((obs[:,self.features]-self.mean)/self.std)
    actor=Actor();optimizer=torch.optim.Adam(actor.parameters(),lr=.001);rng=np.random.default_rng(23)
    tx=torch.from_numpy(x);ty=torch.from_numpy(y);losses=[]
    for epoch in range(args.epochs):
        total=0.;batches=np.array_split(rng.permutation(len(x)),max(1,len(x)//512))
        for idx in batches:
            loss=(actor(tx[idx])-ty[idx]).square().mean();optimizer.zero_grad();loss.backward();optimizer.step();total+=loss.item()
        losses.append(total/len(batches))
        if epoch%40==0:print(json.dumps({'epoch':epoch,'loss':losses[-1]}),flush=True)
    config={'role':'butt-supported-sit-only','hardware_operations':0,'phase_duration_s':DURATION,'phase_index':55,
            'control_hz':50,'feedback_hz':100,'physics_hz':200,'features':FEATURES,'epochs':args.epochs,
            'xml_sha256':hashlib.sha256(XML.read_bytes()).hexdigest(),'seed_record_sha256':hashlib.sha256(SEED_RECORD.read_bytes()).hexdigest(),
            'home_deg':HOME.tolist(),'terminal_motor_deg':END.tolist(),'training':'behavior cloning from closed-loop native BAM demonstrations',
            'limits':'simulation only; initial posture +/-2 degrees and joints +/-0.7 degrees; BAM kp6, not yet identified at hardware P5/D20'}
    actor.eval();torch.save({'state_dict':actor.state_dict(),'config':config,'losses':losses},out/'checkpoint.pt')
    torch.onnx.export(actor,torch.zeros(1,61),str(out/'sit-butt-v3.onnx'),input_names=['observation'],output_names=['action'],opset_version=17,dynamo=False)
    import onnx,onnxruntime as ort
    model=onnx.load(str(out/'sit-butt-v3.onnx'))
    for k,v in {'role':config['role'],'control_hz':'50','phase_duration_s':'1.0','phase_index':'55','physics_model_sha256':config['xml_sha256'],'training':config['training']}.items():
        entry=model.metadata_props.add();entry.key=k;entry.value=v
    onnx.save(model,str(out/'sit-butt-v3.onnx'))
    options=ort.SessionOptions();options.intra_op_num_threads=1
    session=ort.InferenceSession(str(out/'sit-butt-v3.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
    predict=lambda obs:session.run(None,{'observation':obs.reshape(1,61)})[0][0]
    evaluations=[]
    for seed in range(1000,1032):
        _,_,m,trace=rollout(plant,seed,predict,record=seed==1000);evaluations.append(m)
        if trace:(out/'trace.json').write_text(json.dumps(trace))
    result={'config':config,'teacher_passed':sum(m['passed'] for m in teachers),'evaluations':evaluations,
            'passed':all(m['passed'] for m in evaluations),'onnx_sha256':hashlib.sha256((out/'sit-butt-v3.onnx').read_bytes()).hexdigest(),
            'onnx_max_error_rad':float(np.max(np.abs(actor(tx[:64]).detach().numpy()-np.concatenate([session.run(None,{'observation':row.reshape(1,61)})[0] for row in x[:64]]))))}
    (out/'result.json').write_text(json.dumps(result,indent=2));(out/'config.json').write_text(json.dumps(config,indent=2))
    print(json.dumps({'passed':sum(m['passed'] for m in evaluations),'total':len(evaluations),'worst':sorted(evaluations,key=lambda m:m['settled_rotation_rad_s'],reverse=True)[:2]}),flush=True)

if __name__=='__main__':main()
