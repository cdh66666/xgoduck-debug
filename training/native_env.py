"""Private native-CPU training wrapper. No device/network/deployment actions.

The immutable worker owns observation/filter/action/BAM/physics math. Only its
ONNX session.run output is substituted with the externally sampled raw action.
Torch is imported only by NativeVecEnv in the coordinating parent process.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import sys
import traceback
import time
import types
from motion_stall import MotionWindow,STALL_EVENT_PENALTY,event_outcome
from precision_reward import direct_precision
from lateral_reward import lateral_precision
from yaw_reward import straight_yaw_precision
sys.dont_write_bytecode=True

R = Path(__file__).resolve().parent
SOURCE = R / 'engine'
REFERENCE_MANIFEST=R/'ASSET_MANIFEST.json'
MANIFEST = REFERENCE_MANIFEST
COMMANDS = [[0., 0., 0.], [.2, 0., 0.], [.3, 0., 0.], [-.3, 0., 0.], [0., 0., 1.], [0., 0., -1.]]
PROBABILITIES = [.15, .17, .17, .17, .17, .17]
STABILITY_REWARD = dict(profile='standstop-v3',zero_speed_amplitude=5.,linear_sigma_mps=.025,yaw_sigma_radps=.08,
                       tilt_amplitude=5.,tilt_width_deg=.6,speed_path_cost_per_m=20.,yaw_path_cost_per_rad=5.,tilt_path_cost_per_deg=1.)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def manifest():
    result = json.loads(MANIFEST.read_text(encoding='utf-8'))
    if result.get('runtime_interface_version')!='bodyframe-v2':
        raise ValueError('bodyframe-v2_reference_required')
    for key, binding in result['bindings'].items():
        binding['path']=str(R/binding['path'])
        if sha(binding['path']) != binding['sha256']:
            raise ValueError('frozen_binding_changed:' + key)
    xml=Path(os.environ.get('XGODUCK_TRAIN_XML') or R/'model/robot.xml').resolve()
    result['physical_model']=dict(path=str(xml),sha256=sha(xml),
        reference_model_sha256=result['bindings']['model/robot.xml']['sha256'])
    from runtime_profile import selected
    result['runtime_profile']=selected()
    result['randomize_physics']=os.environ.get('XGODUCK_RANDOMIZE_PHYSICS','0')=='1'
    return result


def adapter_module():
    import training_adapter
    return training_adapter


class RawSession:
    """Session.run compatible raw-action source; Policy.infer stays unchanged."""
    def __init__(self, np):
        self.np = np
        self.action = np.zeros((1, 14), dtype=np.float32)
        self.calls = 0
        self.last_obs = None

    def run(self, outputs, feed):
        self.calls += 1
        self.last_obs = next(iter(feed.values())).copy()
        return [self.action.copy()]


class NativeEpisode:
    def __init__(self, frozen, seed, *, curriculum='six', auto_reset=True, max_episode_steps=1200):
        self.adapter = adapter_module().Adapter(frozen, seed, 'native-training', enable_initial_policy=False)
        self.sim = self.adapter.sim
        self.np = self.adapter.worker.np
        self.mj = self.adapter.worker.mujoco
        self.seed = seed
        self.rng = self.np.random.default_rng(seed + 70001)
        self.variation=None
        if frozen.get('randomize_physics'):
            from physics_variation import PhysicsVariation
            self.variation=PhysicsVariation(self.sim,seed)
            self.variation.before_reset();self.sim.reset();self.variation.after_reset()
        self.curriculum = curriculum
        self.auto_reset = auto_reset
        self.max_episode_steps = max_episode_steps
        self.sessions = {}
        for name, actor in self.sim.policies.items():
            session = RawSession(self.np)
            actor.session = session
            self.sessions[name] = session
        self.episode = 0
        self.total_resets = 0
        self.episode_steps = 0
        self.command_ticks = 0
        self.command_index = None
        self.command_age_ticks = 0
        self.zero_release = False
        self.command_counts = self.np.zeros(6, dtype=self.np.int64)
        self.reward_sum = 0.
        self.motion_window=MotionWindow()
        self.target_rate_cost=float(os.environ.get('XGODUCK_TARGET_RATE_COST','0'))
        self.head_motion_cost=float(os.environ.get('XGODUCK_HEAD_MOTION_COST','0'))
        self._enable_without_infer()
        self._resample()

    def _enable_without_infer(self):
        # The exact fields of Simulation.set_policy(True), deliberately without
        # its terminal self.infer(): first observe t0, then apply first PPO action.
        sim = self.sim
        sim.policy_enabled = True
        sim.paused = False
        sim.data.ctrl.fill(0.)
        sim.previous_action.fill(0.)
        sim.recovery.reset()
        sim.policy_started = float(sim.data.time)
        sim.special_active = False
        for actor in sim.policies.values():
            actor.reset_action_filter()

    def _resample(self):
        previous_index = self.command_index
        if self.curriculum in ('stability','stop'):
            index = 0 if previous_index in (None,1,2,3,4,5) else int(self.rng.integers(1,6))
            if self.curriculum=='stop' and previous_index is None and self.rng.random()<.8:
                index=int(self.rng.integers(1,6))
        else:
            index = 0 if self.curriculum == 'stand' else int(self.rng.choice(6, p=PROBABILITIES))
        self.command_index = index
        self.command_age_ticks = 0
        self.zero_release = index == 0 and previous_index not in (None, 0)
        previous_command=getattr(self,'command',self.np.zeros(3,dtype=self.np.float32)).copy()
        self.command = self.np.asarray(COMMANDS[index], dtype=self.np.float32)
        if self.curriculum == 'speed' and index == 2:
            maximum = float(os.environ.get('XGODUCK_MAX_FORWARD', '.3'))
            if not .2 <= maximum <= 1.:
                raise ValueError('speed curriculum maximum must be .2..1 m/s')
            self.command[0] = float(self.rng.uniform(max(.2, maximum * .7), maximum))
        if self.curriculum == 'continuous' and index and self.rng.random()<.5:
            # Train the intermediate commands traversed during keyboard ramps;
            # retain half of the original nonzero endpoints and all zero cases.
            self.command*=float(self.rng.uniform(.05,1.))
        if self.curriculum == 'ramped':
            maximum=float(os.environ.get('XGODUCK_MAX_FORWARD','.3'))
            if not .2<=maximum<=1.:raise ValueError('ramped forward maximum must be .2..1 m/s')
            if index==2:self.command[0]=maximum
            self.command_target=self.command.copy();self.command=previous_command
            self._advance_command_ramp()
        bounds = (3., 5.) if self.curriculum == 'stability' else ((6., 8.) if index == 0 else (4., 6.))
        if self.curriculum=='stop':
            bounds=((4.,5.) if self.rng.random()<.25 else (1.3,1.8)) if index==0 else (3.,5.)
        self.command_ticks = int(math.ceil(float(self.rng.uniform(*bounds)) / .02))
        self.push_tick = None
        if self.curriculum == 'stability' and index == 0 and self.rng.random() < .6:
            self.push_tick = int(self.rng.integers(75,126))
        elif self.curriculum=='stop' and index==0 and bounds==(4.,5.):
            self.push_tick=75
        self.sim.twist[:] = self.command
        self.motion_window.reset(self.command)

    def _advance_command_ramp(self):
        """Sample the next 20 ms command; never buffer or alter motor actions."""
        previous=self.command.copy()
        delta=self.np.clip(self.command_target-self.command,-self.np.array([1.,1.,3.])*.02,self.np.array([1.,1.,3.])*.02)
        self.command=(self.command+delta).astype(self.np.float32)
        return not self.np.array_equal(previous,self.command)

    def observe(self):
        sim, np = self.sim, self.np
        # Sampling this boundary now is equivalent to the first line of the next
        # Simulation.step; the worker's timestamp guard prevents resampling it.
        sim._receive_policy_feedback()
        rotation = sim.data.xmat[sim.root_id].reshape(3, 3)
        gravity = rotation.T @ np.array([0., 0., -1.])
        angles = np.zeros(15, dtype=np.float32)
        vels = np.zeros(15, dtype=np.float32)
        angles[sim.policy_index] = np.rad2deg(sim.data.qpos[sim.qadr])
        vels[sim.policy_index] = np.rad2deg(sim.filtered_velocity)
        return sim.policies[sim.task].build_obs(gravity, sim.filtered_gyro, angles, vels,
                                               self.command, np.zeros(4, dtype=np.float32)).copy()

    def metrics(self):
        sim, np = self.sim, self.np
        velocity = np.zeros(6)
        self.mj.mj_objectVelocity(sim.model, sim.data, self.mj.mjtObj.mjOBJ_XBODY, sim.root_id, velocity, 1)
        rotation = sim.data.xmat[sim.root_id].reshape(3, 3)
        tilt = float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1., 1.))))
        return np.array([velocity[3], velocity[4], velocity[2]]), tilt, float(sim.data.qpos[2])

    def critic_linear_velocity(self):
        # Privileged REAL vx/vy/vz, never actor metrics' vx/vy/yaw shortcut.
        # Read current native data, including after an automatic episode reset.
        velocity=self.np.zeros(6)
        self.mj.mj_objectVelocity(self.sim.model,self.sim.data,self.mj.mjtObj.mjOBJ_XBODY,
                                  self.sim.root_id,velocity,1)
        return velocity[3:6].copy()

    def reset(self):
        sim, np, mj = self.sim, self.np, self.mj
        if self.variation:self.variation.before_reset()
        sim.reset()
        if self.variation:self.variation.after_reset()
        # Same seed distribution as the frozen evaluator, not physics DR.
        yaw = float(self.rng.uniform(-.03, .03))
        sim.data.qpos[3:7] = [np.cos(yaw / 2), 0., 0., np.sin(yaw / 2)]
        sim.data.qpos[sim.qadr] += self.rng.uniform(-.005, .005, len(sim.qadr))
        sim.data.qvel.fill(0.)
        mj.mj_forward(sim.model, sim.data)
        lows = []
        for name in ('left_foot_collision', 'right_foot_collision'):
            gid = sim.model.geom(name).id
            mid = sim.model.geom_dataid[gid]
            start, count = sim.model.mesh_vertadr[mid], sim.model.mesh_vertnum[mid]
            points = sim.model.mesh_vert[start:start + count] @ sim.data.geom_xmat[gid].reshape(3, 3).T + sim.data.geom_xpos[gid]
            lows.append(float(points[:, 2].min()))
        sim.data.qpos[2] += .001 - min(lows)
        sim.standing_height = float(sim.data.qpos[2])
        mj.mj_forward(sim.model, sim.data)
        self._enable_without_infer()
        self.episode_steps = 0
        self.reward_sum = 0.
        self.episode += 1
        self.total_resets += 1
        self.command_index = None
        if self.curriculum=='ramped':self.command=np.zeros(3,dtype=np.float32)
        self._resample()
        return self.observe()

    @staticmethod
    def huber(np, value):
        absolute = np.abs(value)
        return np.where(absolute < 1., .5 * value**2, absolute - .5)

    def _business_failure(self, reason):
        """Record genuine simulator/recovery failure, then reset training only."""
        sim, np = self.sim, self.np
        self.episode_steps += 1
        self.command_age_ticks += 1
        self.command_counts[self.command_index] += 1
        self.reward_sum -= 1.
        def finite_list(array):
            return [float(value) if np.isfinite(value) else None for value in np.asarray(array).reshape(-1)]
        info = dict(terminated=True, timeout=False, steps=self.episode_steps,
                    episode_reward=self.reward_sum, velocity=None, tilt=None, height=None,
                    standing_height=sim.standing_height, command=self.command.tolist(),
                    command_index=self.command_index, command_age_s=self.command_age_ticks*.02,
                    zero_release=self.zero_release, reward_terms={},
                    failure_reasons=[reason], finite_metrics=False, resets=self.total_resets,
                    ncon=int(sim.data.ncon), nefc=int(sim.data.nefc),
                    failure_state=dict(time_s=float(sim.data.time), recovery_phase=sim.recovery.phase,
                                       active_model=sim.active_model, qpos=finite_list(sim.data.qpos),
                                       qvel=finite_list(sim.data.qvel), ctrl=finite_list(sim.data.ctrl)))
        info.update(motion_stall=False,motion_progress=self.motion_window.summary(self.command_age_ticks),
                    terminal_command=self.command.tolist(),motion_stall_penalty=0.)
        info['direct_precision']=dict(active=False,per_second=0.,cost=None,
            command_age_ticks=self.command_age_ticks,reason='business_failure_no_valid_sample',training_only=True)
        info['reward_terms']['direct_precision']=0.
        info['lateral_precision']=dict(active=False,per_second=0.,cost=None,command_age_ticks=self.command_age_ticks,reason='business_failure_no_valid_sample',training_only=True)
        info['reward_terms']['lateral_precision']=0.
        info['straight_yaw_precision']=dict(active=False,per_second=0.,cost=None,command_age_ticks=self.command_age_ticks,reason='business_failure_no_valid_sample',training_only=True)
        info['reward_terms']['straight_yaw_precision']=0.
        if not self.auto_reset:
            raise RuntimeError('business_failure_requires_training_reset:' + reason)
        return self.reset(), -1., True, False, info

    def step(self, action):
        sim, np = self.sim, self.np
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (14,) or not np.isfinite(action).all():
            raise ValueError('invalid_raw_action')
        previous = sim.previous_action.copy()
        previous_targets = sim.last_targets.copy()
        session = self.sessions[sim.task]
        session.action[:] = action
        before_calls = session.calls
        sim.twist[:] = self.command
        if self.curriculum in ('stability','stop') and self.push_tick == self.command_age_ticks:
            direction = self.rng.uniform(-math.pi,math.pi)
            magnitude = self.rng.uniform(.8,2.)
            if self.curriculum=='stop':
                direction=float(self.rng.integers(4))*math.pi/2.;magnitude=2.
            sim.command(dict(op='push',force=[magnitude*math.cos(direction),magnitude*math.sin(direction),0.],duration=.16,body='trunk_base'))
        try:
            sim.step(4)
        except ValueError as error:
            if str(error) in ('physics_nonfinite', 'observation contains NaN or Inf', 'model output contains NaN or Inf'):
                return self._business_failure(str(error))
            raise
        self.mj.mj_forward(sim.model, sim.data)
        if not np.isfinite(sim.data.qpos).all() or not np.isfinite(sim.data.qvel).all():
            return self._business_failure('physics_nonfinite_after_forward')
        if sim.active_model != sim.task or sim.recovery.phase != 'walk':
            return self._business_failure('recovery_or_policy_switch')
        if session.calls - before_calls != 1:
            raise ValueError('policy_call_contract')
        velocity, tilt, height = self.metrics()
        moving = bool(np.linalg.norm(self.command) > .04)
        upright = float(np.clip(np.cos(np.deg2rad(tilt)), 0., 1.) ** 4)
        scale = np.array([.1, .05, .3]) if moving else np.array([.05, .05, .15])
        dense = upright / (1. + float(self.huber(np, (velocity - self.command) / scale).sum()))
        height_score = 1. / (1. + float(self.huber(np, (height - sim.standing_height) / .015)))
        error = velocity - self.command
        gaussian = math.exp(-float((error[:2]**2).sum())/.01 - float(error[2]**2)/.16) if moving else math.exp(-float((velocity[:2]**2).sum())/.03**2 - float(velocity[2]**2)/.08**2)*upright
        progress = 0.
        for index in (0, 2):
            if abs(float(self.command[index])) > .04:
                progress += float(np.clip(velocity[index] * np.sign(self.command[index]) / abs(self.command[index]), -1., 1.)) * upright
        pose = float(np.exp(-np.mean((sim.data.qpos[sim.qadr] - sim.home)**2) / .25))
        action_rate = float(np.sum((action - previous)**2))
        # All per-second terms scaled by the 20ms policy interval, like mjlab.
        reward_terms = dict(dense=6.*dense, height=3.*height_score,
                            gaussian=(6. if moving else 2.)*gaussian,
                            progress=3.*progress, pose=.35*pose, action_rate=-.05*action_rate)
        target_step=sim.last_targets-previous_targets
        reward_terms['filtered_target_rate']=-self.target_rate_cost*float(np.mean((target_step/.1)**2))
        # The observed large jumps are predominantly head_roll. Penalize its
        # real motion and large head offsets during locomotion, without changing
        # targets, filtering, inference rate or which actor weights can learn.
        head_speed=sim.data.qvel[sim.motor.dof_indexes][5:9]
        head_offset=sim.last_targets[5:9]-sim.home[5:9]
        reward_terms['head_motion']=-self.head_motion_cost*float(np.mean((head_speed/2.)**2)+.25*np.mean((head_offset/.35)**2))
        if self.curriculum in ('stability','stop') and not moving:
            target=STABILITY_REWARD
            reward_terms['stop_stability'] = target['zero_speed_amplitude']*math.exp(-float((velocity[:2]**2).sum())/target['linear_sigma_mps']**2-float(velocity[2]**2)/target['yaw_sigma_radps']**2)
            reward_terms['stand_tilt'] = target['tilt_amplitude']*math.exp(-(tilt/target['tilt_width_deg'])**2)
            reward_terms['stop_path'] = -target['speed_path_cost_per_m']*float(np.linalg.norm(velocity[:2]))-target['yaw_path_cost_per_rad']*abs(float(velocity[2]))
            reward_terms['tilt_path'] = -target['tilt_path_cost_per_deg']*tilt
        # self.command_age_ticks is still previous tick here. The current
        # post-forward sample is age+1, so the100th sample gates at exactly2s.
        precision_term,precision_info=direct_precision(self.command,velocity,self.command_age_ticks+1)
        reward_terms['direct_precision']=precision_term
        lateral_term,lateral_info=lateral_precision(self.command,velocity,self.command_age_ticks+1)
        reward_terms['lateral_precision']=lateral_term
        yaw_term,yaw_info=straight_yaw_precision(self.command,velocity,self.command_age_ticks+1)
        reward_terms['straight_yaw_precision']=yaw_term
        reward = .02 * sum(reward_terms.values())
        self.episode_steps += 1
        self.command_age_ticks += 1
        self.command_ticks -= 1
        self.command_counts[self.command_index] += 1
        physical_terminated = bool(max(tilt, sim.max_substep_tilt_deg) >= 25. or
                          max(abs(height - sim.standing_height), sim.max_substep_height_error_m) >= .025)
        # Measure the OLD/current command before any resampling. No previous
        # command's samples enter this segment. Zero command never stalls.
        ramping=self.curriculum=='ramped' and not np.array_equal(self.command,self.command_target)
        # A cross-axis transition may briefly contain both forward and yaw.
        # The single-command stall window starts only at the settled endpoint;
        # physical tilt/height failure detection still applies during the ramp.
        motion_progress=self.motion_window.summary(self.command_age_ticks) if ramping else self.motion_window.observe(self.command,velocity,self.command_age_ticks)
        motion_stall=motion_progress['triggered']
        terminated,timeout,event_penalty=event_outcome(physical_terminated,motion_stall,self.episode_steps>=self.max_episode_steps)
        failure_reasons = []
        if max(tilt, sim.max_substep_tilt_deg) >= 25.:
            failure_reasons.append('tilt25deg')
        if max(abs(height - sim.standing_height), sim.max_substep_height_error_m) >= .025:
            failure_reasons.append('height25mm')
        reward-=event_penalty # original physical -1 plus optional stall -1, once each after dt scaling
        if motion_stall:
            failure_reasons.append('motion_stall')
        self.reward_sum += reward
        info = dict(terminated=terminated, timeout=timeout, steps=self.episode_steps,
                    episode_reward=self.reward_sum, velocity=velocity.tolist(), tilt=tilt,
                    height=height, command=self.command.tolist(), reward_terms=reward_terms,
                    standing_height=sim.standing_height, failure_reasons=failure_reasons,
                    command_index=self.command_index, command_age_s=self.command_age_ticks*.02,
                    zero_release=self.zero_release, finite_metrics=True,
                    resets=self.total_resets, ncon=int(sim.data.ncon), nefc=int(sim.data.nefc))
        info.update(motion_stall=motion_stall,motion_progress=motion_progress,
                    terminal_command=self.command.tolist() if (terminated or timeout) else None,
                    motion_stall_penalty=STALL_EVENT_PENALTY if motion_stall else 0.)
        info['direct_precision']=precision_info
        info['lateral_precision']=lateral_info
        info['straight_yaw_precision']=yaw_info
        done = terminated or timeout
        if done and self.auto_reset:
            obs = self.reset()
        else:
            if self.command_ticks <= 0:
                self._resample()
            elif self.curriculum=='ramped' and self._advance_command_ramp():
                # The stall window and precision grace apply to one settled
                # command, not a mixture of ramp values from earlier ticks.
                self.command_age_ticks=0;self.motion_window.reset(self.command)
            obs = self.observe()
        return obs, float(reward), done, timeout and not terminated, info

    def close(self):
        self.adapter.close()


def _worker(connection, frozen, seeds, curriculum):
    envs = []
    try:
        os.environ['OMP_NUM_THREADS'] = '1'
        os.environ['OPENBLAS_NUM_THREADS'] = '1'
        factory=NativeEpisode
        if curriculum in ('roll','roll_phase'):
            from native_skill_env import NativeSkillEpisode
            factory=NativeSkillEpisode
        elif curriculum=='sit_cycle':
            from native_sit_env import NativeSitEpisode
            factory=NativeSitEpisode
        envs = [factory(frozen, seed, curriculum=curriculum) for seed in seeds]
        np = envs[0].np
        connection.send(('ready', np.stack([env.observe() for env in envs]),
                         np.stack([env.critic_linear_velocity() for env in envs]),
                         dict(pid=os.getpid(), mujoco=envs[0].mj.__version__, envs=len(envs),torch_imported='torch' in sys.modules,
                              physical_model_sha256=envs[0].adapter.physical_model['sha256'],
                              compiled_total_mass_kg=float(envs[0].sim.model.body_mass.sum()))))
        while True:
            op, payload = connection.recv()
            if op == 'close':
                break
            if op == 'step':
                rows = [env.step(action) for env, action in zip(envs, payload, strict=True)]
                # env.step may already have reset. Both obs and this real
                # velocity must belong to the NEW episode in that case.
                privileged=np.stack([env.critic_linear_velocity() for env in envs])
                connection.send(('ok', (np.stack([row[0] for row in rows]),
                                        np.array([row[1] for row in rows], dtype=np.float32),
                                        np.array([row[2] for row in rows], dtype=np.int64),
                                        np.array([row[3] for row in rows], dtype=bool),
                                        privileged,
                                        [row[4] for row in rows])))
            else:
                raise ValueError('unknown_worker_op:' + op)
    except BaseException:
        connection.send(('error', traceback.format_exc()))
    finally:
        for env in envs:
            env.close()
        connection.close()


class NativePool:
    def __init__(self, workers=8, envs_per_worker=2, seed=20261220, curriculum='six'):
        frozen = manifest()
        self.physical_model = frozen['physical_model']
        self.runtime_profile = frozen['runtime_profile']
        self.randomize_physics = frozen['randomize_physics']
        self.np = __import__('numpy')
        self.workers = workers
        self.envs_per_worker = envs_per_worker
        self.num_envs = workers * envs_per_worker
        self.processes, self.connections = [], []
        context = mp.get_context('spawn')
        for index in range(workers):
            parent, child = context.Pipe()
            seeds = [seed + index * envs_per_worker + offset for offset in range(envs_per_worker)]
            process = context.Process(target=_worker, args=(child, frozen, seeds, curriculum), daemon=True)
            process.start()
            child.close()
            self.processes.append(process)
            self.connections.append(parent)
        ready = [self._receive(connection) for connection in self.connections]
        self.obs = self.np.concatenate([row[1] for row in ready])
        self.critic_velocity=self.np.concatenate([row[2] for row in ready])
        self.worker_metadata = [row[3] for row in ready]

    @staticmethod
    def _receive(connection):
        row = connection.recv()
        if row[0] == 'error':
            raise RuntimeError(row[1])
        return row

    def step(self, actions):
        actions = self.np.asarray(actions, dtype=self.np.float32)
        if actions.shape != (self.num_envs, 14):
            raise ValueError('batch_action_shape')
        for index, connection in enumerate(self.connections):
            start = index * self.envs_per_worker
            connection.send(('step', actions[start:start + self.envs_per_worker]))
        results = [self._receive(connection)[1] for connection in self.connections]
        self.obs = self.np.concatenate([row[0] for row in results])
        self.critic_velocity=self.np.concatenate([row[4] for row in results])
        return (self.obs, self.np.concatenate([row[1] for row in results]),
                self.np.concatenate([row[2] for row in results]),
                self.np.concatenate([row[3] for row in results]),
                sum([row[5] for row in results], []))

    def close(self):
        for connection, process in zip(self.connections, self.processes):
            if process.is_alive():
                try:connection.send(('close', None))
                except (BrokenPipeError,EOFError):pass # failed worker already closed its pipe
        for process in self.processes:
            process.join(timeout=15.)
        for connection in self.connections:
            connection.close()


class NativeVecEnv:
    """Structural rsl_rl VecEnv; pure CPU physics with parent-owned tensor I/O."""
    def __init__(self, **kwargs):
        import torch
        from tensordict import TensorDict
        self.torch, self.TensorDict = torch, TensorDict
        self.pool = NativePool(**kwargs)
        self.num_envs, self.num_actions = self.pool.num_envs, 14
        self.device = torch.device('cpu')
        self.max_episode_length = 95 if kwargs.get('curriculum') in ('roll','roll_phase') else 1200
        if kwargs.get('curriculum')=='sit_cycle':
            from sit_reference import CYCLE_STEPS
            self.max_episode_length=CYCLE_STEPS
        self.episode_length_buf = torch.zeros(self.num_envs, dtype=torch.long)
        self.cfg = dict(backend='Windows native frozen MuJoCo/BAM', policy_dt=.02,
                        physics_dt=.005, curriculum=kwargs.get('curriculum', 'six'),
                        runtime_interface_version='bodyframe-v2',gyro_frame='root_body_xmat',linear_velocity_frame='root_body_xmat',
                        hardware_commands_sent=False, deployment_allowed=False)
        self.cfg['physical_model']=self.pool.physical_model
        self.cfg['runtime_profile']=self.pool.runtime_profile
        self.cfg['target_rate_cost']=float(os.environ.get('XGODUCK_TARGET_RATE_COST','0'))
        self.cfg['head_motion_cost']=float(os.environ.get('XGODUCK_HEAD_MOTION_COST','0'))
        if kwargs.get('curriculum')=='sit_cycle':
            from sit_reference import SIT_DURATION,RISE_DURATION,CYCLE_STEPS
            self.cfg['sit_cycle']=dict(sit_duration_s=SIT_DURATION,rise_duration_s=RISE_DURATION,steps=CYCLE_STEPS,
                contact_sampling_hz=200,reward_tilt_margins_deg=[17,22],acceptance_tilt_deg=[20,25],
                source_sha256=sha(R/'native_sit_env.py'),reference_sha256=sha(R/'sit_reference.py'))
        if kwargs.get('curriculum') in ('roll','roll_phase'):
            self.cfg['roll_episode']=dict(duration_s=1.9,contact_sampling_hz=200,head_force_reference_n=20.,reference_is_strength_limit=False,
                source_sha256=sha(R/'native_skill_env.py'),plant_source_sha256=sha(R/'sit_training.py'),contact_source_sha256=sha(R/'butt_sit.py'))
            self.cfg['roll_episode']['phase_conditioned']=kwargs.get('curriculum')=='roll_phase'
        if self.pool.randomize_physics:
            from physics_variation import ENVELOPE
            self.cfg['physics_randomization']=ENVELOPE.copy()
        if kwargs.get('curriculum') == 'speed':
            self.cfg['max_forward_mps']=float(os.environ.get('XGODUCK_MAX_FORWARD', '.3'))
        if kwargs.get('curriculum') == 'continuous':
            self.cfg['intermediate_command_distribution']='50% of nonzero commands scaled by Uniform(.05,1), original endpoints and zero otherwise'
        if kwargs.get('curriculum') == 'ramped':
            self.cfg['command_ramp']=dict(acceleration=[1.,1.,3.],dt_s=.02,hold_s=[4.,6.],zero_hold_s=[6.,8.],source='same command acceleration as evaluate_skill_chain --smooth-commands; no action filtering added')
            self.cfg['max_forward_mps']=float(os.environ.get('XGODUCK_MAX_FORWARD','.3'))
        if kwargs.get('curriculum') in ('stability','stop'):
            self.cfg['stability_reward']=STABILITY_REWARD.copy()
            self.cfg['stability_commands']='alternate zero and random original moving command, each 3-5s'
            self.cfg['stability_push']='60% of zero segments, 0.8-2.0N random world direction for 0.16s at 1.5-2.5s'
            if kwargs.get('curriculum')=='stop':
                self.cfg['stability_commands']='moving 3-5s, zero 1.3-1.8s; 25% zero segments last 4-5s; 80% start moving'
                self.cfg['stability_push']='each long zero segment: 2.0N random one of four world directions for 0.16s at 1.5s'
        self.common_step_counter = 0

    def get_observations(self):
        value = self.torch.from_numpy(self.pool.obs.copy())
        velocity=self.torch.from_numpy(self.pool.critic_velocity.copy()).to(dtype=value.dtype)
        critic=self.torch.cat((value,velocity),dim=-1)
        return self.TensorDict({'actor': value,'critic':critic}, batch_size=[self.num_envs])

    def step(self, actions):
        _, rewards, dones, timeouts, rows = self.pool.step(actions.detach().cpu().numpy())
        self.episode_length_buf += 1
        self.episode_length_buf[self.torch.from_numpy(dones.astype(bool))] = 0
        self.common_step_counter += 1
        return (self.get_observations(), self.torch.from_numpy(rewards), self.torch.from_numpy(dones),
                {'time_outs': self.torch.from_numpy(timeouts),
                 'log': {'/native/tilt': sum(row['tilt'] for row in rows if row['tilt'] is not None)/max(1,sum(row['tilt'] is not None for row in rows))}, 'native_rows': rows})

    def close(self):
        self.pool.close()
