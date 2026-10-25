"""Explicit assumed stress envelope, not measurements or a motor identification.

Total mass stays at the measured 826 g. COM/inertia uncertainty covers the
unweighed board and uniform-density mesh assumptions. Each episode has one
fixed plant; observations still arrive at 100 Hz and actions at 50 Hz.
"""
from collections import deque
import numpy as np

ENVELOPE = dict(voltage_v=[7.2,8.1], floor_friction=[.55,1.],
                trunk_com_offset_m=[-.0015,.0015], trunk_inertia_scale=[.9,1.1],
                transport_delay_physics_steps=[2,6], physics_step_s=.005,
                mass_kg=.826, provenance='assumed stress envelope; D20 mapping still unidentified')

class PhysicsVariation:
    def __init__(self, sim, seed):
        if not np.isclose(sim.model.body_mass.sum(),.826,rtol=0,atol=1e-6):
            raise ValueError('This stress envelope requires the corrected 826 g plant')
        self.sim=sim;self.rng=np.random.default_rng(seed+92001)
        self.com=sim.model.body_ipos[sim.root_id].copy()
        self.inertia=sim.model.body_inertia[sim.root_id].copy()
        self.sample={}

    def before_reset(self):
        s=self.sim;r=self.rng
        delta=r.uniform(*ENVELOPE['trunk_com_offset_m'],3)
        scale=float(r.uniform(*ENVELOPE['trunk_inertia_scale']))
        voltage=float(r.uniform(*ENVELOPE['voltage_v']))
        friction=float(r.uniform(*ENVELOPE['floor_friction']))
        delay=int(r.integers(2,7))
        s.model.body_ipos[s.root_id]=self.com+delta
        s.model.body_inertia[s.root_id]=self.inertia*scale
        s.model.geom_friction[:,0]=friction
        s.bam.actuator.vin=voltage
        import mujoco
        mujoco.mj_setConst(s.model,s.data)
        self.sample=dict(voltage_v=voltage,floor_friction=friction,trunk_com_offset_m=delta.tolist(),
                         trunk_inertia_scale=scale,transport_delay_physics_steps=delay)

    def after_reset(self):
        count=self.sample['transport_delay_physics_steps']
        self.sim.delay_queue=deque([self.sim.home.copy() for _ in range(count)],maxlen=count)
