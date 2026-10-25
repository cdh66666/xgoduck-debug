"""Native walking-stop entry distribution for sit training. Never resets a plant."""
import hashlib
from pathlib import Path
import numpy as np


def load_walk(binding,physical_model):
    import onnxruntime as ort
    path=Path(binding['path'])
    if hashlib.sha256(path.read_bytes()).hexdigest()!=binding['sha256']:raise ValueError('Sit entry walk model changed')
    options=ort.SessionOptions();options.intra_op_num_threads=1
    session=ort.InferenceSession(str(path),sess_options=options,providers=['CPUExecutionProvider'])
    metadata=session.get_modelmeta().custom_metadata_map
    if metadata.get('physical_model_sha256')!=physical_model['sha256'] or metadata.get('runtime_profile')!='calibrated-20261025':raise ValueError('Sit entry walk physics/HOME differs')
    name=session.get_inputs()[0].name
    return lambda obs:session.run(None,{name:obs.reshape(1,61)})[0][0]


def advance_walk_stop(plant,predict):
    """The same 2s stand /4s forward /3s stop prefix as the chain evaluator."""
    command=np.zeros(3);resets=plant.sim.reset_calls;begin=plant.sim.data.time
    for count,target in [(100,[0,0,0]),(200,[.2,0,0]),(150,[0,0,0])]:
        for _ in range(count):
            command+=np.clip(np.asarray(target)-command,-np.array([1.,1.,3.])*.02,np.array([1.,1.,3.])*.02)
            obs,_=plant.observation(0.,0.);obs[48:51]=command;obs[55:61]=0.
            action=predict(obs)
            if not np.isfinite(action).all():raise ValueError('Nonfinite sit entry walk output')
            plant.advance(action);row=plant.measure()
            if row['head_clearance_mm']<40 or abs(row['pitch_deg'])>65 or abs(row['roll_deg'])>30:
                raise RuntimeError('Sit entry prefix physically failed; preserve the failed run instead of silently resampling')
    if plant.sim.reset_calls!=resets:raise RuntimeError('Unexpected reset in walking-stop entry')
    elapsed=float(plant.sim.data.time-begin)
    if abs(elapsed-9.)>1e-7:raise RuntimeError('Sit entry physical clock differs')
    return elapsed
