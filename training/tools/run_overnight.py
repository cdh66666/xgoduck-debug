"""Bounded offline full-actor retraining; no network or hardware access.

Intermediate policies stay candidates. Independent reports, not training reward,
decide whether advancing the speed curriculum is justified.
"""
import ctypes
import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
TRAINING = ROOT / 'training'
OUT = TRAINING / 'outputs/overnight-20261025'
MODEL = TRAINING / 'model/robot_inertial_candidate_20261025.xml'
PROFILE = 'calibrated-20261025'
DEADLINE = dt.datetime(2026, 10, 26, 1, 30, tzinfo=dt.timezone.utc).timestamp()

def save(state):
    state['updated_utc'] = dt.datetime.now(dt.timezone.utc).isoformat()
    temp = OUT / 'status.tmp'
    temp.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding='utf-8')
    temp.replace(OUT / 'status.json')

def execute(args, log):
    with log.open('x', encoding='utf-8') as stream:
        result = subprocess.run([sys.executable, '-B', '-X', 'utf8', *map(str,args)],
                                cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    if result.returncode:
        raise RuntimeError(f'{args[0]} exited {result.returncode}; see {log}')

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if (OUT / 'status.json').exists():
        raise RuntimeError('Run already has a status; resume deliberately, do not duplicate it')
    if os.name == 'nt':
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)
    state = dict(pid=os.getpid(), status='starting', hardware_commands_sent=False,
                 runtime_profile=PROFILE, physical_model=str(MODEL), stages=[],
                 report_deadline='2026-10-26T10:00:00+08:00')
    save(state)
    checkpoint = None
    try:
        for number, maximum in enumerate((.3, .45, .6, .8, 1.0)):
            if time.time() >= DEADLINE:
                state['status'] = 'training_deadline_reached'; break
            name = f'stage-{number:02d}-max-{maximum:g}'
            path = OUT / name
            state.update(status='training', current_stage=name)
            save(state)
            args = ['training/full_refine.py', '--mode','train','--iterations','6000',
                    '--workers','6','--envs-per-worker','4','--curriculum','speed',
                    '--runtime-profile',PROFILE,'--model-xml',MODEL,
                    '--max-forward',maximum,'--trace-every','100',
                    '--seed',202610250 + number,'--max-wall-seconds','1800','--output',path]
            if checkpoint:
                args.extend(['--resume',checkpoint])
            execute(args, OUT / f'{name}-train.log')
            checkpoint = path / 'checkpoint.pt'
            state['status'] = 'evaluating'; save(state)
            report = path / 'evaluation.json'
            execute(['training/evaluate.py','--checkpoint',checkpoint,'--seeds','4',
                     '--max-forward',maximum,'--output',report], OUT/f'{name}-evaluation.log')
            data = json.loads(report.read_text('utf-8'))
            cases = data['cases']
            # Promotion is an exploration gate, NOT the final acceptance gate.
            # Retain all stricter failures in each independent report.
            speed_cases = [c for c in cases if c['command'][0] > 0]
            promotion = all(c['failure'] is None and c['ticks']==900 and
                            c['mean_velocity'] is not None and
                            .7 <= c['mean_velocity'][0]/c['command'][0] <= 1.3
                            for c in speed_cases)
            stage = dict(name=name, maximum_mps=maximum, report=str(report),
                         passed=sum(c['passed'] for c in cases), total=len(cases),
                         exploration_promotion=promotion, deployable=False)
            state['stages'].append(stage); save(state)
            if not promotion:
                state['status']='needs_training_adjustment'; break
        else:
            state['status']='curriculum_complete_pending_review'
    except Exception as error:
        state.update(status='failed', error=str(error))
        raise
    finally:
        save(state)
        if os.name == 'nt':
            ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)

if __name__=='__main__':main()
