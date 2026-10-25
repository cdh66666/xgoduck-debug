"""Dedicated 50 Hz sit/raise ONNX adapter; gait command slots stay zero."""
import hashlib
from pathlib import Path
import numpy as np
from policy import Policy
from rl_core import POLICY_INDEX

SIT_HOME=np.array([0,-5,-22,0,18,20,20,0,0,0,5,22,0,-18],dtype=np.float32)
SIT_SECONDS=8.5

class SitPolicy(Policy):
    def __init__(self):
        self.phase=0.;self.direction=1.;self.last_inference=None
        super().__init__()

    def load(self,path=None):
        path=Path(path) if path else Path(__file__).with_name('xgoduck_sit.onnx')
        ok,message=super().load(path)
        if not ok:return ok,message
        meta=self.session.get_modelmeta().custom_metadata_map
        if meta.get('role')!='sit-down-and-supported-hold' or meta.get('control_hz')!='50' or meta.get('phase_index')!='55' or meta.get('direction_index')!='56' or float(meta.get('phase_duration_s',0))!=SIT_SECONDS:
            self.unload();return False,'Sit model observation/time contract mismatch'
        self.home_deg=SIT_HOME.copy();self.artifact_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        self.action_scale=1.;self.action_alpha=.45;self.action_alpha_head=.2
        return True,message

    def seed_targets(self,targets):
        values=np.asarray(targets,dtype=np.float32)
        if values.shape!=(15,) or not np.isfinite(values).all():raise ValueError('Invalid sit starting targets')
        self.reset_action_filter();self.last_inference=None;action=np.deg2rad(values[POLICY_INDEX]-SIT_HOME)
        self.prev_action[:]=action;self._act_filt[:]=action

    def build_obs(self,grav,gyro_rad,angles_deg,vels_deg,cmd=None,head=None):
        obs=super().build_obs(grav,gyro_rad,angles_deg,vels_deg,[0.]*3,[0.]*4)
        obs[6:20]=np.deg2rad(np.asarray(angles_deg,dtype=np.float32)[POLICY_INDEX]-SIT_HOME)
        obs[55]=self.phase;obs[56]=self.direction
        return obs

    def infer(self,*args,**kwargs):
        action=super().infer(*args,**kwargs)
        self.last_inference=dict(model=self.file,model_sha256=self.artifact_sha256,
            observation=self._obs.tolist(),raw_action=self.prev_action.tolist(),filtered_action=action.tolist())
        return action

    def action_to_deg(self,action):
        values=SIT_HOME+np.rad2deg(np.asarray(action,dtype=np.float32))
        target=[0.]*15
        for index,value in zip(POLICY_INDEX,values):target[int(index)]=float(value)
        return target
