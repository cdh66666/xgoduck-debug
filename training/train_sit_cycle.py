"""Train one sit/rise actor from validated native closed-loop demonstrations.

All network weights start randomly. No device or network operations. A policy
must actually sit on its butt before rising; merely staying upright cannot pass.
"""
import argparse,hashlib,json,os,time
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import numpy as np
from butt_sit import SitPlant,XML,HOME,SEED_RECORD
from search_butt_raise import rollout

FEATURES=[1,3,5,55,56]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--teacher',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--epochs',type=int,default=400)
    args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    selected=json.loads(args.teacher.read_text());parameters=selected['parameters']
    if selected['passed']!=selected['total']:raise ValueError('Teacher must pass its independent cycle gate first')
    plant=SitPlant();observations=[];labels=[];teachers=[]
    for seed in range(64):
        _,metrics,trace=rollout(plant,parameters,seed,variant=True,record_training=True)
        teachers.append(metrics)
        if metrics['passed']:
            observations.extend(r['observation'] for r in trace);labels.extend(r['teacher_action'] for r in trace)
    (out/'teacher-evaluation.json').write_text(json.dumps(teachers,indent=2))
    print(json.dumps({'teacher_passed':sum(m['passed'] for m in teachers),'total':64}),flush=True)
    if sum(m['passed'] for m in teachers)<60:raise ValueError('New training seed distribution failed teacher gate')
    x=np.asarray(observations,dtype=np.float32);y=np.asarray(labels,dtype=np.float32)
    np.savez_compressed(out/'demonstrations.npz',observations=x,actions=y)
    import torch
    torch.set_num_threads(2);torch.set_num_interop_threads(1);torch.manual_seed(20261025)
    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__();self.register_buffer('features',torch.tensor(FEATURES))
            self.register_buffer('mean',torch.from_numpy(x[:,FEATURES].mean(0)))
            self.register_buffer('std',torch.from_numpy(x[:,FEATURES].std(0).clip(.05)))
            self.net=torch.nn.Sequential(torch.nn.Linear(5,128),torch.nn.Tanh(),torch.nn.Linear(128,128),torch.nn.Tanh(),torch.nn.Linear(128,14))
        def forward(self,obs):return self.net((obs[:,self.features]-self.mean)/self.std)
    actor=Actor();optimizer=torch.optim.Adam(actor.parameters(),lr=.001)
    tx=torch.from_numpy(x);ty=torch.from_numpy(y);rng=np.random.default_rng(25);losses=[]
    for epoch in range(args.epochs):
        total=0.;batches=np.array_split(rng.permutation(len(x)),max(1,len(x)//512))
        for batch in batches:
            loss=(actor(tx[batch])-ty[batch]).square().mean()
            optimizer.zero_grad();loss.backward();optimizer.step();total+=loss.item()
        losses.append(total/len(batches))
        if epoch%40==0:print(json.dumps({'epoch':epoch,'loss':losses[-1]}),flush=True)
    cfg=dict(role='butt-supported-sit-rise',initialization='random; all network weights trained',
             control_hz=50,feedback_hz=100,physics_hz=200,phase_index=55,direction_index=56,
             sit_duration_s=1.,rise_duration_s=parameters[5],features=FEATURES,home_deg=HOME.tolist(),
             teacher_parameters=parameters,teacher_sha256=hashlib.sha256(args.teacher.read_bytes()).hexdigest(),
             physics_model_sha256=hashlib.sha256(XML.read_bytes()).hexdigest(),
             initial_pose_sha256=hashlib.sha256(SEED_RECORD.read_bytes()).hexdigest(),
             hardware_deployment_ready=False,hardware_commands_sent=False,
             scope='sit supported at 2-3s then rise and settle without reset; no walking handoff acceptance yet')
    actor.eval();torch.save({'state_dict':actor.state_dict(),'config':cfg,'losses':losses},out/'checkpoint.pt')
    model_path=out/'sit-rise.onnx'
    torch.onnx.export(actor,torch.zeros(1,61),str(model_path),input_names=['observation'],output_names=['action'],opset_version=17,dynamo=False)
    import onnx,onnxruntime as ort
    graph=onnx.load(str(model_path))
    for key in ('role','control_hz','phase_index','direction_index','sit_duration_s','rise_duration_s','physics_model_sha256'):
        entry=graph.metadata_props.add();entry.key=key;entry.value=str(cfg[key])
    onnx.checker.check_model(graph);onnx.save(graph,str(model_path))
    options=ort.SessionOptions();options.intra_op_num_threads=1
    session=ort.InferenceSession(str(model_path),sess_options=options,providers=['CPUExecutionProvider']);latencies=[]
    def predict(obs):
        started=time.perf_counter();result=session.run(None,{'observation':obs.reshape(1,61)})[0][0]
        latencies.append((time.perf_counter()-started)*1000);return result
    evaluations=[]
    for seed in range(3000,3032):
        _,metrics,trace=rollout(plant,parameters,seed,variant=True,predict=predict)
        evaluations.append(metrics)
        if seed==3000:(out/'trace.json').write_text(json.dumps(trace))
    parity=float(np.max(np.abs(actor(tx[:128]).detach().numpy()-np.stack([predict(row) for row in x[:128]]))))
    result=dict(config=cfg,passed=sum(m['passed'] for m in evaluations),total=len(evaluations),evaluations=evaluations,
                onnx_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),max_export_error_rad=parity,
                local_onnx_ms={'mean':float(np.mean(latencies)),'p99':float(np.percentile(latencies,99)),'max':float(max(latencies))})
    (out/'result.json').write_text(json.dumps(result,indent=2));(out/'config.json').write_text(json.dumps(cfg,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('config','evaluations')}),flush=True)

if __name__=='__main__':main()
