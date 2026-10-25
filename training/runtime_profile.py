"""Explicit deployment contracts; preserve the archived engine for old results."""
import os
import numpy as np

CALIBRATED = 'calibrated-20261025'
PROFILES = ('legacy', CALIBRATED)
HOME_DEG = [0., -5., -22., 0., 18., 20., 20., 0., 0., 0., 5., 22., 0., -18.]
POLICY_INDEX = [0, 1, 2, 3, 4, 10, 11, 12, 13, 5, 6, 7, 8, 9]

def selected():
    name = os.environ.get('XGODUCK_RUNTIME_PROFILE', 'legacy')
    if name not in PROFILES:
        raise ValueError('Unknown runtime profile: ' + name)
    return name

def apply(sim, name):
    if name == 'legacy':
        return
    if name != CALIBRATED:
        raise ValueError('Unknown runtime profile: ' + name)
    from policy import Policy
    class CalibratedPolicy(Policy):
        def __init__(self):
            super().__init__()
            self.home_deg = np.asarray(HOME_DEG, dtype=np.float32)
            self.home_rad = np.deg2rad(self.home_deg)
            self.action_alpha_head = .2

        def build_obs(self, grav, gyro_rad, angles_deg, vels_deg, cmd=None, head=None):
            obs = super().build_obs(grav, gyro_rad, angles_deg, vels_deg, cmd, head)
            obs[6:20] = np.deg2rad(np.asarray(angles_deg, dtype=np.float32)[POLICY_INDEX] - self.home_deg)
            return obs

        def infer(self, *args, **kwargs):
            self.build_obs(*args, **kwargs)
            action = self.session.run(self._outputs, self._feed)[0].reshape(14)
            if not np.isfinite(action).all():
                raise ValueError('model output contains NaN or Inf')
            self.prev_action[:] = action
            for sl, alpha in ((slice(0, 5), self.action_alpha),
                              (slice(5, 9), self.action_alpha_head),
                              (slice(9, 14), self.action_alpha)):
                self._act_filt[sl] *= alpha
                self._act_filt[sl] += action[sl] * (1 - alpha)
            return self._act_filt.copy()

        def action_to_deg(self, action):
            target = np.zeros(15, dtype=np.float32)
            target[POLICY_INDEX] = self.home_deg + np.rad2deg(self.action_scale * action)
            return target.tolist()

    for key in sim.policies:
        policy = CalibratedPolicy()
        policy.load()
        sim.policies[key] = policy
    sim.home = np.deg2rad(np.asarray(HOME_DEG, dtype=np.float32)).astype(float)
    sim.session = sim.policies[sim.task].session
    sim.reset()
