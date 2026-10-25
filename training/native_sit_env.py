"""Continuous sit/rise PPO plant: full actor control, unchanged physical clock."""
import math
from collections import deque
import numpy as np
from butt_sit import SitPlant,XML
from sit_training import INDEX,HOME,smooth
from sit_reference import (SIT_STEPS,CYCLE_STEPS,SEAT_HEIGHT,SEATED_GEOMS,LEG_INDICES,SEAT_DEG,phase)


class NativeSitEpisode:
    def __init__(self,frozen,seed,*,curriculum='sit_cycle',auto_reset=True,**kwargs):
        if curriculum!='sit_cycle' or str(XML.resolve())!=frozen['physical_model']['path'] or frozen.get('runtime_profile')!='calibrated-20261025':
            raise ValueError('Sit cycle requires the corrected plant, calibrated HOME and explicit curriculum')
        self.plant=SitPlant();self.sim=self.plant.sim;self.adapter=self.plant.adapter;self.mj=self.plant.mj;self.np=np
        self.rng=np.random.default_rng(seed);self.auto_reset=auto_reset;self.variation=None
        self.entry_walk=None
        if frozen.get('sit_entry_walk'):
            from sit_entry import load_walk
            self.entry_walk=load_walk(frozen['sit_entry_walk'],frozen['physical_model'])
        if frozen.get('randomize_physics'):
            from physics_variation import PhysicsVariation
            self.variation=PhysicsVariation(self.sim,seed)
        self.total_resets=0;self.reset()

    def reset(self):
        if self.variation:self.variation.before_reset()
        self.plant.reset(int(self.rng.integers(1,2000000000)))
        if self.variation:
            self.plant.delay=[self.plant.delay[0].copy() for _ in range(self.variation.sample['transport_delay_physics_steps'])]
        self.entry_mode='fresh';self.entry_duration=0.
        if self.entry_walk is not None and self.total_resets%2==0:
            from sit_entry import advance_walk_stop
            self.entry_duration=advance_walk_stop(self.plant,self.entry_walk);self.entry_mode='walking_stop'
        self.steps=0;self.total_resets+=1;self.standing_height=float(self.sim.data.qpos[2])
        self.previous_targets=self.plant.filtered.copy();self.tail=deque(maxlen=50)
        self.seat_achieved=False;self.sit_peak_tilt=0.;self.rise_peak_tilt=0.;self.episode_target_step=0.
        return self.observe()

    def observe(self):
        progress,direction,_,_=phase(self.steps)
        return self.plant.observation(progress,direction)[0]

    def critic_linear_velocity(self):
        v=np.zeros(6);self.mj.mj_objectVelocity(self.sim.model,self.sim.data,self.mj.mjtObj.mjOBJ_XBODY,self.sim.root_id,v,1)
        return v[3:6].copy()

    def settled(self,sitting):
        if len(self.tail)<50:return False
        rows=list(self.tail);mean=lambda name:float(np.mean([r[name] for r in rows]))
        if mean('linear_speed')>.005 or mean('angular_speed')>.04:return False
        if sitting:
            return 75<mean('height_mm')<95 and mean('pose_error_deg')<8 and mean('seat_support_n')>1
        return 120<mean('height_mm')<155 and abs(mean('pitch_deg'))<8 and mean('feet_n')>6 and mean('other_n')<.1

    def step(self,action):
        progress,direction,elapsed,duration=phase(self.steps);sitting=direction>0
        self.plant.advance(action);self.steps+=1
        r=self.plant.measure();velocity=np.zeros(6)
        self.mj.mj_objectVelocity(self.sim.model,self.sim.data,self.mj.mjtObj.mjOBJ_XBODY,self.sim.root_id,velocity,1)
        delta=self.plant.filtered-self.previous_targets;self.previous_targets=self.plant.filtered.copy()
        target_step=float(np.max(np.abs(np.rad2deg(delta))));self.episode_target_step=max(self.episode_target_step,target_step)
        contact=r['contacts'];seat=sum(max(0.,c['force_n']) for c in contact if c['geom'] in SEATED_GEOMS)
        upper=sum(max(0.,c['force_n']) for c in contact if c['geom'] not in SEATED_GEOMS and c['kind'] not in ('feet','butt'))
        upper=max(upper,self.plant.peak_contact_forces_n['head'])
        q=np.rad2deg(self.sim.data.qpos[self.sim.qadr]);goal=SEAT_DEG if sitting else self.plant.start[INDEX]
        pose_error=float(np.max(np.abs(q[LEG_INDICES]-goal[LEG_INDICES])))
        linear=float(np.linalg.norm(velocity[3:]));angular=float(np.linalg.norm(velocity[:3]))
        self.tail.append(dict(height_mm=r['root_height_mm'],pitch_deg=r['pitch_deg'],pose_error_deg=pose_error,
            linear_speed=linear,angular_speed=angular,seat_support_n=seat,feet_n=r['forces_n']['feet'],other_n=sum(r['forces_n'][k] for k in ('butt','head','other'))))
        tilt=abs(r['pitch_deg'])
        if sitting:self.sit_peak_tilt=max(self.sit_peak_tilt,tilt)
        else:self.rise_peak_tilt=max(self.rise_peak_tilt,tilt)
        height=r['root_height_mm']/1000;target_height=(self.standing_height+(SEAT_HEIGHT-self.standing_height)*smooth(progress)) if sitting else (SEAT_HEIGHT+(self.standing_height-SEAT_HEIGHT)*smooth(progress))
        # Optimize a margin INSIDE the fixed 20/25deg evaluation gates. The
        # policy owns every action; these are rewards, never corrective targets.
        tilt_margin=17. if sitting else 22.
        reward=.02*(.5*math.exp(-((height-target_height)/.025)**2)
            -4*(max(0.,tilt-tilt_margin)/3)**2
            -2*(max(0.,target_step-5.)/1.)**2
            -.1*float(np.mean((delta/.1)**2))-20*upper
            -2*(max(0.,60-r['head_clearance_mm'])/10)**2)
        if progress>=1:
            reward+=.02*(4*math.exp(-(pose_error/8)**2)+2*math.exp(-((height-target_height)/.01)**2)
                         -20*linear-2*angular)
        if self.steps==SIT_STEPS:
            self.seat_achieved=self.settled(True) and self.sit_peak_tilt<=20
            reward+=5 if self.seat_achieved else -5
            self.tail.clear()
        failures=[];physical=tilt>55 or abs(r['roll_deg'])>25 or r['head_clearance_mm']<40
        done=physical or self.steps>=CYCLE_STEPS
        if physical:failures.append('sit_cycle_physical_failure');reward-=10
        elif done:
            passed=self.seat_achieved and self.settled(False) and self.rise_peak_tilt<=25 and self.episode_target_step<=6
            reward+=5 if passed else -5
            if not passed:failures.append('sit_cycle_pose_or_margin')
        info=dict(velocity=[float(velocity[3]),float(velocity[4]),float(velocity[2])],tilt=math.radians(tilt),height=height,standing_height=self.standing_height,
            command=[0.,0.,0.],command_age_s=self.steps*.02,zero_release=False,failure_reasons=failures,resets=self.total_resets,steps=self.steps,
            phase=progress,direction=direction,entry_mode=self.entry_mode,entry_duration_s=self.entry_duration,seat_achieved=self.seat_achieved,sit_peak_tilt_deg=self.sit_peak_tilt,rise_peak_tilt_deg=self.rise_peak_tilt,
            target_step_deg=target_step,head_min_mm=r['head_clearance_mm'],seat_support_n=seat,upper_contact_n=upper,
            peak_motor_torque_nm=self.plant.peak_torque_nm,peak_joint_speed_radps=self.plant.peak_joint_speed_radps,contact_sampling_hz=200,physical_termination=physical)
        obs=self.reset() if done and self.auto_reset else self.observe()
        return obs,float(reward),done,False,info

    def close(self):self.adapter.close()
