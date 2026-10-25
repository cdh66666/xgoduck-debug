"""Official SitStand geometry plus the current candidate's phase contract.

Geometry source: LuwuDynamics/xgoduck_rl microduck_sitstand_env_cfg.py and
xgoduck_constants.py. Durations are this experiment's, not official video timing.
"""
import numpy as np
from sit_training import HOME

SIT_DURATION=1.8967401589126274
RISE_DURATION=3.2405872655960186
HOLD_DURATION=2.
SIT_STEPS=round((SIT_DURATION+HOLD_DURATION)*50)
RISE_STEPS=round((RISE_DURATION+HOLD_DURATION)*50)
CYCLE_STEPS=SIT_STEPS+RISE_STEPS
SEAT_HEIGHT=.08586815080318921
SEATED_GEOMS={'sandbox_contact_left_hip_pitch','sandbox_contact_right_hip_pitch'}
LEG_INDICES=np.array([1,2,3,4,10,11,12,13])
SEAT_DEG=HOME.astype(float).copy()
SEAT_DEG[LEG_INDICES]=np.rad2deg([0,-.4079,1.35,0,0,.4079,-1.35,0])

def phase(steps):
    sit=steps<SIT_STEPS
    elapsed=(steps if sit else steps-SIT_STEPS)*.02
    duration=SIT_DURATION if sit else RISE_DURATION
    return min(elapsed/duration,1.),1. if sit else -1.,elapsed,duration
