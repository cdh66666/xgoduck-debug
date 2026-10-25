"""Train a complete, randomly initialized supported-dance actor, offline only."""
import argparse,hashlib,json,os
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import numpy as np
from dance_teacher import SitPlant,rollout,DURATION
from butt_sit import XML,HOME

FEATURES=[0,1,3,4,5,55]
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--epochs',type=int,default=400);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);plant=SitPlant();xs=[];ys=[];teachers=[]
    for seed in range(48):
        x,y,m,_=rollout(plant,seed);teachers.append(m)
        if m['passed']:xs.append(x);ys.append(y)
    (a.output/'teacher-evaluation.json').write_text(json.dumps(teachers,indent=2))
    print(json.dumps({'teacher_passed':sum(m['passed'] for m in teachers),'total':48}),flush=True)
    if sum(m['passed'] for m in teachers)<46:raise ValueError('Supported dance teacher failed its varied initial conditions')
    x=np.concatenate(xs);y=np.concatenate(ys);np.savez_compressed(a.output/'demonstrations.npz',observations=x,actions=y)
    import torch
    torch.set_num_threads(2);torch.set_num_interop_threads(1);torch.manual_seed(26)
    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__();self.register_buffer('indices',torch.tensor(FEATURES))
            self.register_buffer('mean',torch.from_numpy(x[:,FEATURES].mean(0)));self.register_buffer('std',torch.from_numpy(x[:,FEATURES].std(0).clip(.05)))
            self.net=torch.nn.Sequential(torch.nn.Linear(10,128),torch.nn.Tanh(),torch.nn.Linear(128,128),torch.nn.Tanh(),torch.nn.Linear(128,14))
        def forward(self,obs):
            phase=obs[:,55:56]*(DURATION*np.pi)
            value=torch.cat(((obs[:,self.indices]-self.mean)/self.std,torch.sin(phase),torch.cos(phase),torch.sin(phase*2),torch.cos(phase*2)),dim=1)
            return self.net(value)
    actor=Actor();optimizer=torch.optim.Adam(actor.parameters(),lr=.001);tx=torch.from_numpy(x);ty=torch.from_numpy(y);rng=np.random.default_rng(26);losses=[]
    for epoch in range(a.epochs):
        total=0.;batches=np.array_split(rng.permutation(len(x)),max(1,len(x)//512))
        for batch in batches:
            loss=(actor(tx[batch])-ty[batch]).square().mean();optimizer.zero_grad();loss.backward();optimizer.step();total+=float(loss.detach())
        losses.append(total/len(batches))
        if epoch%40==0:print(json.dumps({'epoch':epoch,'loss':losses[-1]}),flush=True)
    actor.eval();config=dict(role='supported-head-body-dance',control_hz=50,feedback_hz=100,physics_hz=200,duration_s=DURATION,phase_index=55,home_deg=HOME.tolist(),
        initialization='random; all actor weights trained',physics_model_sha256=hashlib.sha256(XML.read_bytes()).hexdigest(),
        plant_source_sha256=hashlib.sha256(Path(__file__).with_name('sit_training.py').read_bytes()).hexdigest(),
        teacher_source_sha256=hashlib.sha256(Path(__file__).with_name('dance_teacher.py').read_bytes()).hexdigest(),
        scope='Small supported head/body dance; not a spin, handstand or acrobatic skill',hardware_deployment_ready=False)
    torch.save({'state_dict':actor.state_dict(),'config':config,'losses':losses},a.output/'checkpoint.pt')
    model=a.output/'dance.onnx';torch.onnx.export(actor,torch.zeros(1,61),str(model),input_names=['observation'],output_names=['action'],opset_version=17,dynamo=False)
    import onnx
    from evaluate_skill_chain import predictor
    graph=onnx.load(str(model))
    for key in ('role','control_hz','phase_index','duration_s','physics_model_sha256'):
        entry=graph.metadata_props.add();entry.key=key;entry.value=str(config[key])
    onnx.checker.check_model(graph);onnx.save(graph,str(model));predict=predictor(model);cases=[]
    for seed in range(6000,6032):
        _,_,metrics,trace=rollout(plant,seed,predict);cases.append(metrics)
        if seed==6000 or not metrics['passed']:(a.output/f'trace-{seed}.json').write_text(json.dumps(trace))
    parity=float(np.max(np.abs(actor(tx[:128]).detach().numpy()-np.stack([predict(row) for row in x[:128]]))))
    report=dict(passed=sum(m['passed'] for m in cases),total=len(cases),cases=cases,config=config,onnx_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),max_export_error_rad=parity,hardware_commands_sent=False)
    (a.output/'config.json').write_text(json.dumps(config,indent=2));(a.output/'result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('config','cases')}),flush=True)

if __name__=='__main__':main()
