"""Native roll episode with 200Hz contact evidence; no walk fall/recovery gate."""
import math
import numpy as np
from butt_sit import SitPlant,XML

class NativeSkillEpisode:
    def __init__(self,frozen,seed,*,curriculum='roll',auto_reset=True,**kwargs):
        if curriculum not in ('roll','roll_phase'):raise ValueError('Only a complete forward roll is defined here')
        self.phase_conditioned=curriculum=='roll_phase'
        if str(XML.resolve())!=frozen['physical_model']['path']:raise ValueError('Skill plant must use the corrected XML')
        self.plant=SitPlant();self.sim=self.plant.sim;self.adapter=self.plant.adapter;self.mj=self.plant.mj;self.np=np
        self.seed=seed;self.rng=np.random.default_rng(seed);self.auto_reset=auto_reset;self.variation=None
        if frozen.get('randomize_physics'):
            from physics_variation import PhysicsVariation
            self.variation=PhysicsVariation(self.sim,seed)
        self.total_resets=0;self.reset()

    def reset(self):
        if self.variation:self.variation.before_reset()
        self.plant.reset(int(self.rng.integers(1,2000000000)))
        if self.variation:
            self.plant.delay=[self.plant.delay[0].copy() for _ in range(self.variation.sample['transport_delay_physics_steps'])]
        self.steps=0;self.total_resets+=1;self.rotation=0.;self.head_peak=0.;self.head_impulse=0.
        self.previous_angle=self.angle();self.previous_targets=self.plant.filtered.copy();self.standing_height=float(self.sim.data.qpos[2])
        return self.observe()

    def angle(self):
        r=self.sim.data.xmat[self.sim.root_id].reshape(3,3)
        return float(np.arctan2(r[0,2],r[2,2]))

    def observe(self):return self.plant.observation(min(1.,self.steps/95) if self.phase_conditioned else 0,0)[0]

    @staticmethod
    def single_roll_progress(previous,current):
        # Reward approaching one turn and penalize overshoot; otherwise a second
        # turn earns another positive progress reward despite a failed landing.
        return float(np.clip(abs(previous-2*math.pi)-abs(current-2*math.pi),-.4,.4))

    def critic_linear_velocity(self):
        velocity=np.zeros(6);self.mj.mj_objectVelocity(self.sim.model,self.sim.data,self.mj.mjtObj.mjOBJ_XBODY,self.sim.root_id,velocity,1)
        return velocity[3:6].copy()

    def step(self,action):
        self.plant.advance(action);self.steps+=1;angle=self.angle();delta=float(np.arctan2(np.sin(angle-self.previous_angle),np.cos(angle-self.previous_angle)))
        previous_rotation=self.rotation;self.rotation+=delta;self.previous_angle=angle
        contact=self.plant.peak_contact_forces_n;self.head_peak=max(self.head_peak,contact['head']);self.head_impulse+=self.plant.contact_impulse_ns['head']
        r=self.sim.data.xmat[self.sim.root_id].reshape(3,3);tilt=float(np.rad2deg(np.arccos(np.clip(r[2,2],-1,1))))
        velocity=np.zeros(6);self.mj.mj_objectVelocity(self.sim.model,self.sim.data,self.mj.mjtObj.mjOBJ_XBODY,self.sim.root_id,velocity,1)
        height=float(self.sim.data.qpos[2]);target_step=self.plant.filtered-self.previous_targets;self.previous_targets=self.plant.filtered.copy()
        # 20N is an optimization reference (~2.5 bodyweights), NOT a verified
        # strength limit. Completion, contact load and settling are all reported.
        progress=self.single_roll_progress(previous_rotation,self.rotation) if self.phase_conditioned else float(np.clip(delta,-.4,.4))
        reward=1.5*progress-.02*(3*max(0,contact['head']-20)/20+.15*np.mean((target_step/.1)**2))
        if self.phase_conditioned:reward-=.25*self.plant.contact_impulse_ns['head']
        if self.steps>=75:
            reward+=.02*(2*math.exp(-((self.rotation-2*math.pi)/.3)**2)+2*max(0,r[2,2]))
        done=self.steps>=95;failures=[]
        cycle=abs(self.rotation-2*math.pi)<math.radians(30) and tilt<15 and height>.115 and np.linalg.norm(velocity[3:])<.08
        if done:
            reward+=5 if cycle else -5
            if not cycle:failures.append('incomplete_roll_or_unsettled_landing')
        info=dict(velocity=[float(velocity[3]),float(velocity[4]),float(velocity[2])],tilt=tilt,height=height,standing_height=self.standing_height,
                  command=[0.,0.,0.],command_age_s=self.steps*.02,zero_release=False,failure_reasons=failures,resets=self.total_resets,steps=self.steps,
                  roll_rotation_deg=math.degrees(self.rotation),roll_complete=bool(cycle),head_peak_n=self.head_peak,head_impulse_ns=self.head_impulse,
                  peak_motor_torque_nm=self.plant.peak_torque_nm,peak_requested_torque_nm=self.plant.peak_command_torque_nm,
                  contact_sampling_hz=200,physical_termination=False)
        obs=self.reset() if done and self.auto_reset else self.observe()
        return obs,float(reward),done,False,info

    def close(self):self.adapter.close()
