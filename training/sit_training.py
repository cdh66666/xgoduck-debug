"""Train a separate, phase-conditioned sit/raise policy from native BAM rollouts.

No hardware I/O. The teacher is only a training reference. Deployment uses ONNX.
Observation 55 is local phase [0,1], 56 is direction (+1 sit, -1 raise).
"""
import argparse,hashlib,json,math,os,sys,time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
HOME=np.array([0,-5,-22,0,18,20,20,0,0,0,5,22,0,-18],dtype=np.float32)
INDEX=np.array([0,1,2,3,4,10,11,12,13,5,6,7,8,9])
COEFFICIENTS=np.array([
 [6.0086469356,43.1248341025,-3.9658763001,9.5254017399,-3.8392251229,3.1285991262],
 [32.8675404519,62.7334940389,-17.7534344684,9.2688861108,-4.2361592763,.5178603321],
 [26.8588935132,19.6086599526,-13.7875583225,-.2565151522,-.3969347469,-2.6107385401]])
DURATION=8.5

def smooth(u):
    u=float(np.clip(u,0,1));return u**3*(10+u*(-15+6*u))

def path_pose(u):
    h,k,a=COEFFICIENTS@np.power(float(u),np.arange(6))
    return np.array([0,0,h,k,a,0,0,-h,-k,-a,20,20,0,0,0],dtype=float)

def teacher(t,direction,start,gravity):
    if direction>0:
        if t<2:
            target=start+(path_pose(0)-start)*smooth((t-.5)/1.5)
        else:target=path_pose(smooth((t-2)/6.5))
    elif t<6.5:target=path_pose(1-smooth(t/6.5))
    else:target=path_pose(0)+(start-path_pose(0))*smooth((t-6.5)/1.5)
    pitch=math.degrees(math.atan2(gravity[0],-gravity[2]))
    correction=float(np.clip(.5*pitch,-12,12));target[4]+=correction;target[9]-=correction
    return np.deg2rad(target[INDEX]-HOME).astype(np.float32)

class Plant:
    def __init__(self,xml,start,measured,gravity):
        os.environ['XGODUCK_TRAIN_XML']=str(xml)
        from native_env import manifest
        from training_adapter import Adapter
        self.adapter=Adapter(manifest(),7,'sit-policy',enable_initial_policy=False)
        self.sim=self.adapter.sim;self.mj=self.adapter.worker.mujoco
        self.start=start;self.measured=measured;self.gravity=gravity
        self.head_ids=[self.sim.model.body(n).id for n in ('neck','neck_pitch','yaw_roll_motion','jaw_soft')]
        self.head_geoms=[i for i in range(self.sim.model.ngeom) if self.sim.model.geom_group[i]==2 and self.sim.model.geom_bodyid[i] in self.head_ids]

    def vertices(self,gid):
        s=self.sim;mid=s.model.geom_dataid[gid];a,n=s.model.mesh_vertadr[mid],s.model.mesh_vertnum[mid]
        return s.model.mesh_vert[a:a+n]@s.data.geom_xmat[gid].reshape(3,3).T+s.data.geom_xpos[gid]

    def reset(self,seed):
        s=self.sim;rng=np.random.default_rng(seed);s.reset()
        g=self.gravity.copy();g[:2]+=rng.uniform(-.035,.035,2);g/=np.linalg.norm(g)
        quat=np.r_[1-g[2],-g[1],g[0],0.];quat/=np.linalg.norm(quat)
        s.data.qpos[3:7]=quat;s.data.qpos[s.qadr]=np.deg2rad(self.measured[INDEX]+rng.uniform(-.7,.7,14));s.data.qpos[2]=0
        self.mj.mj_forward(s.model,s.data)
        low=min(self.vertices(s.model.geom(n).id)[:,2].min() for n in ('left_foot_collision','right_foot_collision'))
        s.data.qpos[2]=.001-low;s.data.qvel.fill(0);self.mj.mj_forward(s.model,s.data)
        s.motor.reset(s.data.qpos);s.bam.actuator.q_target_smooth=np.deg2rad(self.start[INDEX]).copy()
        s._feedback_filter_time=None
        self.previous=np.deg2rad(self.start[INDEX]-HOME).astype(np.float32)
        self.filtered=self.previous.copy();self.delay=[np.deg2rad(self.start[INDEX]).copy() for _ in range(4)]

    def observation(self,phase,direction):
        s=self.sim;s._receive_policy_feedback()
        gravity=s.data.xmat[s.root_id].reshape(3,3).T@np.array([0.,0.,-1.])
        x=np.zeros(61,dtype=np.float32);x[:3]=s.filtered_gyro;x[3:6]=gravity
        x[6:20]=s.data.qpos[s.qadr]-np.deg2rad(HOME);x[20:34]=s.filtered_velocity
        x[34:48]=self.previous;x[55]=phase;x[56]=direction
        return x,gravity

    def advance(self,action):
        self.previous=np.asarray(action,dtype=np.float32).copy()
        alpha=np.full(14,.45);alpha[5:9]=.2
        self.filtered=alpha*self.filtered+(1-alpha)*self.previous
        target=np.deg2rad(HOME)+self.filtered
        self.peak_torque_nm=0.;self.peak_command_torque_nm=0.;self.peak_joint_speed_radps=0.
        self.peak_contact_forces_n={k:0. for k in ('butt','feet','head','other')}
        self.contact_impulse_ns={k:0. for k in self.peak_contact_forces_n}
        for _ in range(4):
            if round(self.sim.data.time/.005)%2==0:self.sim._receive_policy_feedback()
            delayed=self.delay.pop(0);self.delay.append(target.copy())
            self.sim.motor.q_target=delayed;self.sim.motor.update();self.mj.mj_step(self.sim.model,self.sim.data)
            self._record_substep()
        # Like NativeEpisode.step, align geometry/contact observations with the
        # final qpos, rather than using the pose from the preceding 5ms substep.
        self.mj.mj_forward(self.sim.model,self.sim.data)

    def _record_substep(self):
        s=self.sim
        # ctrl is requested motor torque; actuator_force includes MuJoCo's
        # force limit (the motor gear is 1). Keep both, rather than conflating.
        self.peak_torque_nm=max(self.peak_torque_nm,float(np.max(np.abs(s.data.actuator_force[s.motor.act_indexes]))))
        self.peak_command_torque_nm=max(self.peak_command_torque_nm,float(np.max(np.abs(s.data.ctrl[s.motor.act_indexes]))))
        self.peak_joint_speed_radps=max(self.peak_joint_speed_radps,float(np.max(np.abs(s.data.qvel[s.motor.dof_indexes]))))

    def rollout(self,seed,predict=None,record=False):
        self.reset(seed);xs=[];ys=[];trace=[];failed=None
        for direction in (1.,-1.):
            for tick in range(500):
                t=tick*.02;phase=min(1.,t/DURATION)
                obs,gravity=self.observation(phase,direction);label=teacher(t,direction,self.start,gravity)
                action=label if predict is None else predict(obs)
                xs.append(obs);ys.append(label);self.advance(action)
                rotation=self.sim.data.xmat[self.sim.root_id].reshape(3,3)
                tilt=float(np.degrees(np.arccos(np.clip(rotation[2,2],-1,1))))
                clearance=float(min(self.vertices(i)[:,2].min() for i in self.head_geoms)*1000)
                row=dict(t=float(self.sim.data.time),phase=phase,direction=direction,tilt_deg=tilt,
                         head_clearance_mm=clearance,height_mm=float(self.sim.data.qpos[2]*1000))
                if record:row['qpos']=self.sim.data.qpos.tolist()
                trace.append(row)
                if tilt>25 or clearance<30:
                    failed='tilt_or_head_clearance';break
            if failed:break
        metrics=dict(seed=seed,passed=failed is None,error=failed,steps=len(trace),
                     max_tilt_deg=max(r['tilt_deg'] for r in trace),min_head_clearance_mm=min(r['head_clearance_mm'] for r in trace),
                     seated_height_mm=next((r['height_mm'] for r in trace if r['direction']>0 and r['phase']==1),None),
                     end_height_mm=trace[-1]['height_mm'])
        failures=[]
        if failed:failures.append(failed)
        if metrics['max_tilt_deg']>=20:failures.append('tilt_over_20deg')
        if metrics['seated_height_mm'] is None or not 70<metrics['seated_height_mm']<105:failures.append('did_not_reach_seated_height')
        if not 115<metrics['end_height_mm']<160:failures.append('did_not_return_to_standing_height')
        metrics['qualification_failures']=failures;metrics['passed']=not failures
        return np.asarray(xs),np.asarray(ys),metrics,trace

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seed-record',type=Path,required=True)
    parser.add_argument('--model-xml',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--episodes',type=int,default=64);parser.add_argument('--epochs',type=int,default=160)
    parser.add_argument('--demonstrations',type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    data=json.loads(args.seed_record.read_text(encoding='utf-8'));active=[r['state'] for r in data['frames'] if r['segment']=='active']
    start=np.median([s['targets'] for s in active[-8:]],axis=0);measured=np.median([s['angles'] for s in active[-8:]],axis=0)
    gravity=np.median([s['gravity'] for s in active[-8:]],axis=0);gravity/=np.linalg.norm(gravity)
    plant=Plant(args.model_xml,start,measured,gravity)
    config=dict(kind='behavior-cloning from closed-loop native-physics sit/raise teacher',
        hardware_operations=0,model_xml=str(args.model_xml.resolve()),model_sha256=hashlib.sha256(args.model_xml.read_bytes()).hexdigest(),
        seed_record_sha256=hashlib.sha256(args.seed_record.read_bytes()).hexdigest(),
        observation_dim=61,action_dim=14,phase_index=55,direction_index=56,phase_duration_s=DURATION,
        control_hz=50,feedback_hz=100,physics_hz=200,home_deg=HOME.tolist(),start_motor_deg=start.tolist(),episodes=args.episodes,epochs=args.epochs)
    (args.output/'config.json').write_text(json.dumps(config,indent=2),encoding='utf-8')
    x=[];y=[];teacher_metrics=[]
    if args.demonstrations:
        previous=json.loads((args.demonstrations.parent/'config.json').read_text())
        assert previous['model_sha256']==config['model_sha256'] and previous['seed_record_sha256']==config['seed_record_sha256']
        cached=np.load(args.demonstrations);x=[cached['observations']];y=[cached['actions']]
        teacher_metrics=json.loads((args.demonstrations.parent/'teacher-evaluation.json').read_text())
        assert all(m['passed'] and m['seated_height_mm'] is not None and 70<m['seated_height_mm']<105 and 115<m['end_height_mm']<160 for m in teacher_metrics)
    else:
        for seed in range(args.episodes):
            a,b,m,_=plant.rollout(seed);teacher_metrics.append(m)
            if m['passed']:x.append(a);y.append(b)
    (args.output/'teacher-evaluation.json').write_text(json.dumps(teacher_metrics,indent=2))
    if sum(m['passed'] for m in teacher_metrics)<args.episodes*.8:raise RuntimeError('teacher did not pass sufficient native randomized rollouts')
    x=np.concatenate(x);y=np.concatenate(y);np.savez_compressed(args.output/'demonstrations.npz',observations=x,actions=y)
    import torch
    torch.set_num_threads(2);torch.set_num_interop_threads(1);torch.manual_seed(7)
    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__();features=np.array([3,4,5,55,56]);self.register_buffer('features',torch.from_numpy(features))
            self.register_buffer('mean',torch.from_numpy(x[:,features].mean(0)));self.register_buffer('std',torch.from_numpy(x[:,features].std(0).clip(.05)))
            self.net=torch.nn.Sequential(torch.nn.Linear(5,96),torch.nn.Tanh(),torch.nn.Linear(96,96),torch.nn.Tanh(),torch.nn.Linear(96,14))
        def forward(self,obs):return self.net((obs[:,self.features]-self.mean)/self.std)
    actor=Actor();optimizer=torch.optim.Adam(actor.parameters(),lr=1e-3)
    tx=torch.from_numpy(x);ty=torch.from_numpy(y);rng=np.random.default_rng(7);losses=[]
    for epoch in range(args.epochs):
        actor.train();total=0.
        for indices in np.array_split(rng.permutation(len(x)),max(1,len(x)//1024)):
            pred=actor(tx[indices]);loss=(pred-ty[indices]).square().mean();optimizer.zero_grad();loss.backward();optimizer.step();total+=float(loss.detach())
        losses.append(total/max(1,len(x)//1024))
        if epoch%20==0:print(json.dumps({'epoch':epoch,'loss':losses[-1],'teacher_passed':len(teacher_metrics)-sum(not m['passed'] for m in teacher_metrics)}),flush=True)
    actor.eval();torch.save({'state_dict':actor.state_dict(),'config':config,'losses':losses},args.output/'checkpoint.pt')
    torch.onnx.export(actor,torch.zeros(1,61),str(args.output/'sit-down-v1.onnx'),input_names=['observation'],output_names=['action'],opset_version=17,dynamo=False)
    import onnx,onnxruntime as ort
    model=onnx.load(str(args.output/'sit-down-v1.onnx'))
    metadata={'role':'sit-down-and-supported-hold','control_hz':'50','phase_index':'55','direction_index':'56','phase_duration_s':str(DURATION),
              'default_joint_pos':','.join(map(str,np.deg2rad(HOME).tolist())),'physics_model_sha256':config['model_sha256'],'training':'native-physics behavior cloning'}
    for k,v in metadata.items():entry=model.metadata_props.add();entry.key=k;entry.value=v
    onnx.save(model,str(args.output/'sit-down-v1.onnx'))
    options=ort.SessionOptions();options.intra_op_num_threads=1;session=ort.InferenceSession(str(args.output/'sit-down-v1.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
    def predict(obs):return session.run(None,{'observation':obs.reshape(1,61)})[0][0]
    evaluations=[]
    for seed in range(1000,1016):
        _,_,metrics,trace=plant.rollout(seed,predict,record=True);evaluations.append(metrics)
        (args.output/('evaluation-%s.json'%seed)).write_text(json.dumps(dict(metrics=metrics,trace=trace)))
    result=dict(config=config,teacher_passed=sum(m['passed'] for m in teacher_metrics),teacher_total=len(teacher_metrics),
                independent_evaluations=evaluations,passed=all(m['passed'] for m in evaluations),
                model_sha256=hashlib.sha256((args.output/'sit-down-v1.onnx').read_bytes()).hexdigest(),
                features_used=[3,4,5,55,56],
                output_max_error_rad=float(np.max(np.abs(actor(tx[:32]).detach().numpy()-np.concatenate([session.run(None,{'observation':row.reshape(1,61)})[0] for row in x[:32]])))))
    (args.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)

if __name__=='__main__':main()
