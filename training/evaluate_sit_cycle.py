"""Recheck an existing sit/rise ONNX actor against the current native plant."""
import argparse,hashlib,json
from pathlib import Path
from butt_sit import SitPlant,XML
from evaluate_skill_chain import predictor
from search_butt_raise import rollout

def main():
    p=argparse.ArgumentParser();p.add_argument('--model-directory',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seeds',type=int,default=32);p.add_argument('--first-seed',type=int,default=3000)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    cfg=json.loads((a.model_directory/'config.json').read_text());model=a.model_directory/'sit-rise.onnx'
    if cfg['physics_model_sha256']!=hashlib.sha256(XML.read_bytes()).hexdigest():raise ValueError('Physics XML changed')
    plant=SitPlant();predict=predictor(model);results=[]
    for seed in range(a.first_seed,a.first_seed+a.seeds):
        _,metrics,trace=rollout(plant,cfg['teacher_parameters'],seed,variant=True,predict=predict);results.append(metrics)
        if seed==a.first_seed or not metrics['passed']:(a.output/f'trace-{seed}.json').write_text(json.dumps(trace))
    report=dict(passed=sum(m['passed'] for m in results),total=len(results),evaluations=results,
                onnx_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),config=cfg,hardware_commands_sent=False,
                plant_source_sha256=hashlib.sha256(Path(__file__).with_name('sit_training.py').read_bytes()).hexdigest())
    (a.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('config','evaluations')}))

if __name__=='__main__':main()
