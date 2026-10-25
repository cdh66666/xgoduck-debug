"""ONNX policy adapter matching the Raspberry Pi deployment math."""
from __future__ import annotations
import math
import hashlib
from pathlib import Path
from typing import Optional
import numpy as np
from rl_core import (HOME_DEG, ACTION_SCALE, ACTION_ALPHA, ACTION_ALPHA_HEAD,
                     OBS_DIM, ACTION_DIM, MOTOR_IDS, POLICY_INDEX, State, build_obs as make_obs,
                     action_to_deg as deg_from_action)
HOME_RAD = np.deg2rad(HOME_DEG)
JOINT_NAMES = ('left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee',
               'left_ankle', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
               'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle')
PYTHON_DIR = Path(__file__).resolve().parent

def default_onnx() -> Optional[Path]:
    p = PYTHON_DIR / 'xgoduck_walk.onnx'
    return p if p.is_file() else None

class Policy:
    def __init__(self, *, reference_home=False):
        self.reference_home = reference_home
        self.action_scale = ACTION_SCALE
        self.action_alpha = ACTION_ALPHA
        self.action_alpha_head = ACTION_ALPHA_HEAD
        self.unload()

    @property
    def loaded(self):
        return self.session is not None

    def unload(self):
        self.session = None
        self.in_name = self.file = ''
        self.artifact_sha256 = None
        self.last_inference = None
        self.home_rad = HOME_RAD.copy()
        self.home_deg = HOME_DEG.copy()
        self.home_source = 'legacy_runtime_fallback'
        self._legacy_home_math = True
        self._obs = np.empty(OBS_DIM, dtype=np.float32)
        self.reset_action_filter()

    def load(self, path=None):
        p = Path(path) if path else default_onnx()
        if p is None:
            return False, 'default model xgoduck_walk.onnx was not found'
        if not p.is_file():
            p = PYTHON_DIR / p.name
        if not p.is_file():
            return False, f'not found: {p}'
        try:
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            model_bytes = p.read_bytes()
            sess = ort.InferenceSession(model_bytes, sess_options=options, providers=['CPUExecutionProvider'])
            inp, out = sess.get_inputs()[0], sess.get_outputs()[0]
            if len(inp.shape) != 2 or inp.shape[-1] != OBS_DIM:
                raise ValueError(f'expected observation shape [1,61], got {inp.shape}')
            if len(out.shape) != 2 or out.shape[-1] != ACTION_DIM:
                raise ValueError(f'expected action shape [1,14], got {out.shape}')
            self._set_model_home(sess.get_modelmeta().custom_metadata_map or {})
            self.session, self.in_name = sess, inp.name
            self._outputs = [out.name]
            self.file = p.name
            self.artifact_sha256 = hashlib.sha256(model_bytes).hexdigest()
            self._feed = {self.in_name: self._obs.reshape(1, OBS_DIM)}
            # Touch all graph arenas before the control loop so the first live
            # switch does not pay cold-session latency.
            result = self.warm(12)
            if result.shape != (1, ACTION_DIM) or not np.isfinite(result).all():
                raise ValueError('model must emit a finite [1,14] action')
        except Exception as exc:
            self.unload()
            return False, str(exc)
        self.reset_action_filter()
        return True, f'{self.file} in=61 out=14 threads=1'

    def _set_model_home(self, metadata):
        names = metadata.get('joint_names')
        if names is not None and tuple(value.strip() for value in names.split(',')) != JOINT_NAMES:
            raise ValueError('model joint_names do not match the 14 policy joints')
        if self.reference_home:
            # The official deployment uses one HOME for every bundled policy,
            # independently of each export's training-pose metadata.
            self.home_rad, self.home_deg = HOME_RAD.copy(), HOME_DEG.copy()
            self.home_source, self._legacy_home_math = 'official_runtime', True
            return
        values = metadata.get('default_joint_pos')
        if values is None:
            self.home_rad, self.home_deg = HOME_RAD.copy(), HOME_DEG.copy()
            self.home_source, self._legacy_home_math = 'legacy_runtime_fallback', True
            return
        home = np.asarray([float(value) for value in values.split(',')], dtype=np.float64)
        if (home.shape != (ACTION_DIM,) or not np.isfinite(home).all()
                or np.any(np.abs(home) > np.finfo(np.float32).max)):
            raise ValueError('model default_joint_pos must contain 14 finite float32 radians')
        # Ten-decimal ONNX exports round the existing float32 runtime HOME.
        # Reuse its exact math when equivalent, so the qualified walk is unchanged.
        self._legacy_home_math = bool(np.allclose(home, HOME_RAD, rtol=0., atol=5e-8))
        self.home_rad = home.astype(np.float32)
        self.home_deg = HOME_DEG.copy() if self._legacy_home_math else np.rad2deg(self.home_rad)
        self.home_source = 'onnx.default_joint_pos'

    def warm(self, runs: int = 1):
        if not self.loaded:
            return None
        self._obs.fill(0)
        last = None
        for _ in range(max(1, runs)):
            last = self.session.run(self._outputs, self._feed)[0]
        return last

    def build_obs(self, grav, gyro_rad, angles_deg, vels_deg, cmd=None, head=None):
        st = State([0.0] * 3, [0.0] * 3, list(angles_deg), list(vels_deg),
                   list(grav), list(gyro_rad))
        twist = np.zeros(3, dtype=np.float32) if cmd is None else cmd
        head_cmd = np.zeros(4, dtype=np.float32) if head is None else head
        obs = make_obs(st, self.prev_action, twist, head_cmd, out=self._obs)
        if not self._legacy_home_math:
            q_deg = np.asarray(angles_deg, dtype=np.float32)[POLICY_INDEX]
            obs[6:20] = np.deg2rad(q_deg - self.home_deg)
        if not np.isfinite(obs).all():
            raise ValueError('observation contains NaN or Inf')
        return obs

    def infer(self, grav, gyro_rad, angles_deg, vels_deg, cmd=None, head=None):
        if not self.loaded:
            return None
        self.build_obs(grav, gyro_rad, angles_deg, vels_deg, cmd, head)
        action = self.session.run(self._outputs, self._feed)[0].reshape(ACTION_DIM)
        if not np.isfinite(action).all():
            raise ValueError('model output contains NaN or Inf')
        self.prev_action[:] = action
        # Policy order has four head joints (IDs 30..33), then right leg ID20.
        for sl, alpha in ((slice(0, 5), self.action_alpha),
                          (slice(5, 9), self.action_alpha_head),
                          (slice(9, 14), self.action_alpha)):
            self._act_filt[sl] *= alpha
            self._act_filt[sl] += action[sl] * (1 - alpha)
        # Freeze the exact input/output before warm() or the next infer() reuses
        # the ONNX input arena. The motion recorder samples this immutable row.
        self.last_inference = dict(model=self.file, model_sha256=self.artifact_sha256,
            observation=self._obs.tolist(), raw_action=action.tolist(),
            filtered_action=self._act_filt.tolist())
        return self._act_filt.copy()

    def set_action_alphas(self, *, leg=None, head=None):
        # Validate the complete update first so a bad head value cannot change leg.
        leg = self.action_alpha if leg is None else float(leg)
        head = self.action_alpha_head if head is None else float(head)
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in (leg, head)):
            raise ValueError('action alpha must be within [0,1]')
        self.action_alpha, self.action_alpha_head = leg, head
        return leg, head

    def reset_action_filter(self):
        self.last_inference = None
        self.prev_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._act_filt = np.zeros(ACTION_DIM, dtype=np.float32)

    def action_to_deg(self, action):
        action = np.asarray(action, dtype=np.float32)
        if self._legacy_home_math:
            return deg_from_action(action)
        targets = np.zeros(len(MOTOR_IDS), dtype=np.float32)
        targets[POLICY_INDEX] = self.home_deg + np.rad2deg(ACTION_SCALE * action)
        return targets.tolist()
