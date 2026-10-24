"""Local CPU physics sandbox. This module has no hardware or network imports.

stdin: JSON commands; stdout: NDJSON acknowledgements and live snapshots.
MuJoCo owns contacts, gravity, actuator forces and all body poses. Official
Policy/Recovery run at 50 Hz, CPU BAM at 200 Hz. No recorded trajectory is read.
"""
from __future__ import annotations

import hashlib
from collections import deque
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'vendor'))
import numpy as np
import mujoco
import onnxruntime as ort

PROJECT = HERE.parent.parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE/'bam_vendor'))
from simulation_catalog import resolve_model
from bam.model import load_model as load_bam
from bam.mujoco import MujocoController
JOINTS = ('left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee',
          'left_ankle', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
          'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle')
OBSERVATIONS = 'base_ang_vel,projected_gravity,joint_pos,joint_vel,actions,command,head_command,body_command'


def vector(value, n, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (n,) or not np.isfinite(result).all():
        raise ValueError(f'invalid_{label}')
    return result


class Simulation:
    def __init__(self):
        self.model = self.data = self.session = None
        self.paused = True
        self.policy_enabled = False
        self.seq = 0
        self.feedback_filter_samples = 0

    def load(self, model_id, expected_sha):
        bound = resolve_model(HERE.parent, model_id)
        if bound['sha256'] != expected_sha:
            raise ValueError('selected_model_sha_changed')
        source = Path(bound['xml'])
        sys.path.insert(0, str(Path(bound['sources']['policy.py']['path']).parent))
        from policy import Policy
        from rl_core import HOME_DEG, MOTOR_IDS, POLICY_INDEX
        sys.path.insert(0, str(Path(bound['recovery']).parent))
        from recovery import Recovery
        self.policies = {}
        for name in {bound['selected'], 'walk', 'getup'}:
            policy = Policy(reference_home=True)
            ok, message = policy.load(bound['models'][name]['path'])
            if not ok:
                raise ValueError(message)
            if name=='roulade':policy.set_action_alphas(leg=.15,head=.15)
            self.policies[name] = policy
        self.binding = bound
        self.motor_ids, self.policy_index = MOTOR_IDS, POLICY_INDEX
        self.recovery = Recovery()
        tree = ET.parse(source)
        root = tree.getroot()
        root.find('compiler').set('meshdir', str(source.parent / 'assets'))
        option = root.find('option')
        if option is None:
            option = ET.SubElement(root, 'option')
        option.set('timestep', '0.005')
        option.set('gravity', '0 0 -9.81')
        option.set('integrator', 'implicitfast')
        # Explicitly requested interactive-drop extension. Source XML is never
        # saved or changed; existing foot geoms remain the trained contact mesh.
        extra_contacts = 0
        for body in root.findall('.//body'):
            for geom in list(body.findall('geom')):
                if geom.get('class') != 'visual' or not geom.get('mesh'):
                    continue
                mesh=geom.get('mesh')
                if mesh in ('uno-q','extension','cover','left_ankle','right_ankle'):
                    continue
                attributes={k:v for k,v in geom.attrib.items() if k not in ('class','name')}
                attributes.update(name='sandbox_contact_'+mesh,contype='1',conaffinity='0',
                                  group='3',friction='0.8 0.02 0.001',condim='3')
                ET.SubElement(body,'geom',**attributes)
                extra_contacts += 1
        ET.SubElement(root.find('worldbody'), 'geom', name='sandbox_ground', type='plane',
                      size='0 0 0.01', contype='8', conaffinity='1',
                      friction='0.8 0.02 0.001', condim='3')
        # The XML position gains are viewer companions. Training replaces them
        # with this same identified BAM motor model and firmware P6.
        bam = load_bam(bound['bam'])
        bam.actuator.kp = 6.
        bam.actuator.vin = 7.4
        limit = 8.*bam.kt.value/bam.R.value
        actuators=root.find('actuator')
        for old in list(actuators):
            name=old.get('name');joint=old.get('joint');actuators.remove(old)
            ET.SubElement(actuators,'motor',name=name,joint=joint,gear='1',
                          ctrllimited='false',forcelimited='true',forcerange=f'{-limit} {limit}')
        for joint in root.findall('.//joint'):
            if joint.get('name') in JOINTS:
                joint.set('armature',str(bam.armature.value));joint.set('damping','0');joint.set('frictionloss','0')
                joint.set('solreffriction','-50000 -200');joint.set('solimpfriction','0.99 0.9999 0.001 0.5 2')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        if model.nq != 21 or model.nu != 14:
            raise ValueError('physics_joint_contract')
        self.model, self.data, self.session = model, mujoco.MjData(model), self.policies[bound['selected']].session
        self.task, self.model_sha = bound['selected'], bound['sha256']
        self.xml_sha = hashlib.sha256(source.read_bytes()).hexdigest()
        self.home, self.action_scale = np.deg2rad(HOME_DEG).astype(float), 1.
        self.qadr = np.asarray([model.joint(name).qposadr[0] for name in JOINTS])
        self.vadr = np.asarray([model.joint(name).dofadr[0] for name in JOINTS])
        self.actuator_ids = np.asarray([model.actuator(name).id for name in JOINTS])
        self.body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(1, model.nbody)]
        self.root_id = model.body('trunk_base').id
        self.bam = bam
        self.motor = MujocoController(bam, JOINTS, model, self.data)
        # Native MuJoCo friction efc_id stores DOF ids (the free root offsets
        # them by six); the CPU BAM selector must use those DOFs.
        self.motor.joint_indexes = self.motor.dof_indexes
        self.extra_contacts = extra_contacts
        self.reset()

    def reset(self):
        if self.model is None:
            raise ValueError('no_model_loaded')
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = self.home
        self.data.qpos[:3] = 0
        self.data.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)
        lows = []
        for name in ('left_foot_collision', 'right_foot_collision'):
            gid = self.model.geom(name).id
            mid = self.model.geom_dataid[gid]
            start, count = self.model.mesh_vertadr[mid], self.model.mesh_vertnum[mid]
            points = self.model.mesh_vert[start:start + count] @ self.data.geom_xmat[gid].reshape(3, 3).T + self.data.geom_xpos[gid]
            lows.append(float(points[:, 2].min()))
        self.data.qpos[2] = -min(lows) + .001
        self.standing_height = float(self.data.qpos[2])
        self.previous_action = np.zeros(14, dtype=np.float32)
        self.filtered_gyro = self.filtered_velocity = None
        self._feedback_filter_time = None
        self.feedback_filter_samples = 0
        default_command=self.binding['models'][self.task].get('default_command', [.2,0,0] if self.task=='fast' else [0,0,0])
        self.twist = np.array(default_command,dtype=np.float32)
        self.delay_queue = deque([self.home.copy() for _ in range(4)],maxlen=4)
        self.motor.reset(self.data.qpos)
        self.motor.last_ts=float(self.data.time)
        self.bam.actuator.q_target_smooth=self.home.copy()
        for policy in self.policies.values():policy.reset_action_filter()
        self.recovery.reset()
        self.active_model = self.task
        self.last_targets = self.home.copy()
        self.inference_counts = {name:0 for name in self.policies}
        self.policy_steps = self.physics_steps = 0
        self.last_infer_time = -1.
        self.force = np.zeros(3)
        self.force_until = 0.
        self.force_body = self.root_id
        self.force_point = None
        self.grab_position = None
        self.drop_seed = None
        self.paused = True
        self.set_policy(False)
        mujoco.mj_forward(self.model, self.data)

    def set_policy(self, enabled):
        self.policy_enabled = bool(enabled)
        self.data.ctrl.fill(0)
        if enabled:
            self.paused = False
            self.previous_action.fill(0)
            self.recovery.reset()
            if self.task=='getup':self.recovery.phase='getup'
            self.policy_started=float(self.data.time)
            self.special_active=self.task in ('pick','roulade')
            for policy in self.policies.values():policy.reset_action_filter()
            self.infer()

    def _receive_policy_feedback(self):
        # Match the 100 Hz receive IIR; a 50 Hz end sample with alpha squared
        # misses the intervening sample during a push or a fast rotation.
        now=float(self.data.time)
        before=self._feedback_filter_time
        if before is not None and now<=before:return
        velocity=np.zeros(6)
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_XBODY,self.root_id,velocity,1)
        raw=self.data.qvel[self.vadr]
        if before is None or now-before>.1:
            self.filtered_gyro=velocity[:3].copy();self.filtered_velocity=raw.copy()
        else:
            ga=.5**((now-before)/.01);va=.4**((now-before)/.01)
            self.filtered_gyro=ga*self.filtered_gyro+(1-ga)*velocity[:3]
            self.filtered_velocity=va*self.filtered_velocity+(1-va)*raw
        self._feedback_filter_time=now
        self.feedback_filter_samples += 1

    def infer(self):
        rotation = self.data.xmat[self.root_id].reshape(3, 3)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, self.root_id, velocity, 1)
        gravity=rotation.T @ np.array([0.,0.,-1.])
        raw_velocity=self.data.qvel[self.vadr]
        if self.filtered_gyro is None:self._receive_policy_feedback()
        elapsed=float(self.data.time)-self.policy_started
        rolling=self.task=='roulade' and self.special_active and elapsed<1.9
        if self.task=='roulade' and self.special_active and not rolling:
            self.special_active=False
            self.recovery.fall_elapsed=0.
        # Match Controller: the one-shot roll owns its complete 1.9 s window;
        # Recovery resumes after it ends, rather than aborting the roll early.
        changed=False if rolling else self.recovery.update(gravity,float(self.data.time),True)
        if changed:
            self.special_active=False
            for policy in self.policies.values():policy.reset_action_filter()
        selected='getup' if self.recovery.phase=='getup' else ('walk' if self.task=='getup' else self.task)
        command=self.twist.copy()
        if selected=='getup':command.fill(0)
        elif self.task=='pick':
            if self.special_active and elapsed<4.:
                phi=elapsed/4.;command=np.array([math.cos(2*math.pi*phi),math.sin(2*math.pi*phi),0.],dtype=np.float32)
            else:selected='walk';command.fill(0)
        elif self.task=='roulade':
            if not self.special_active or elapsed>=1.9:selected='walk'
            command.fill(0)
        policy=self.policies[selected]
        if selected!=self.active_model:policy.reset_action_filter()
        self.active_model=selected
        angles=np.zeros(15,dtype=np.float32);vels=np.zeros(15,dtype=np.float32)
        angles[self.policy_index]=np.rad2deg(self.data.qpos[self.qadr])
        vels[self.policy_index]=np.rad2deg(self.filtered_velocity)
        action=policy.infer(gravity,self.filtered_gyro,angles,vels,command,np.zeros(4,dtype=np.float32))
        target=np.deg2rad(np.array(policy.action_to_deg(action))[self.policy_index])
        ranges = self.model.jnt_range[[self.model.joint(name).id for name in JOINTS]]
        # As in training, keep raw position targets; MuJoCo joint constraints
        # and BAM torque saturation enforce the physical mechanical limits.
        self.outside_ranges = int(np.count_nonzero((target < ranges[:, 0]) | (target > ranges[:, 1])))
        self.last_targets=target
        self.previous_action[:] = policy.prev_action
        self.policy_steps += 1
        self.last_infer_time=float(self.data.time)
        self.inference_counts[selected]+=1

    def step(self, count=1):
        for _ in range(count):
            if self.physics_steps%2==0:self._receive_policy_feedback()
            if self.policy_enabled and self.physics_steps % 4 == 0 and self.data.time>self.last_infer_time+1e-8:
                self.infer()
            if self.policy_enabled:
                delayed=self.delay_queue.popleft();self.delay_queue.append(self.last_targets.copy())
                self.motor.q_target=delayed
                self.motor.update()
            else:
                self.data.ctrl.fill(0)
            self.data.qfrc_applied.fill(0)
            self.data.xfrc_applied.fill(0)
            if self.data.time < self.force_until:
                point = self.force_point if self.force_point is not None else self.data.xipos[self.force_body]
                mujoco.mj_applyFT(self.model, self.data, self.force, np.zeros(3), point, self.force_body, self.data.qfrc_applied)
            if self.grab_position is not None:
                velocity = np.zeros(6)
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, self.root_id, velocity, 0)
                force = 120 * (self.grab_position - self.data.xpos[self.root_id]) - 8 * velocity[3:]
                norm = np.linalg.norm(force)
                if norm > 20:
                    force *= 20 / norm
                mujoco.mj_applyFT(self.model, self.data, force, np.zeros(3), self.data.xpos[self.root_id], self.root_id, self.data.qfrc_applied)
            mujoco.mj_step(self.model, self.data)
            self.physics_steps += 1
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                self.paused = True
                self.set_policy(False)
                raise ValueError('physics_nonfinite')

    def command(self, cmd):
        op = cmd.get('op')
        if op == 'load':
            self.load(cmd['model_id'], cmd['expected_sha'])
            return
        if self.model is None:
            raise ValueError('no_model_loaded')
        if op == 'reset':
            self.reset()
        elif op == 'policy':
            self.set_policy(cmd.get('enabled', False))
        elif op == 'command':
            twist=vector(cmd['twist'],3,'command')
            if np.any(np.abs(twist)>np.array([.4,.3,1.])):raise ValueError('command_outside_official_range')
            self.twist=twist.astype(np.float32)
        elif op == 'pause':
            self.paused = bool(cmd.get('paused', True))
            if self.paused:
                self.twist.fill(0)
                self.force_until = 0
                self.grab_position = None
        elif op == 'push':
            force = vector(cmd['force'], 3, 'force')
            duration = float(cmd.get('duration', .3))
            if np.linalg.norm(force) > 30 or not .01 <= duration <= 3:
                raise ValueError('force_outside_sandbox_range')
            body = self.model.body(cmd.get('body', 'trunk_base')).id
            if body == 0:
                raise ValueError('cannot_push_world')
            self.force = force
            self.force_body = body
            self.force_point = vector(cmd['point'], 3, 'point') if 'point' in cmd else None
            self.force_until = self.data.time + duration
            self.paused = False
        elif op == 'grab':
            position = vector(cmd['position'], 3, 'grab')
            if not np.all(np.abs(position[:2]) <= 2) or not .08 <= position[2] <= 1:
                raise ValueError('grab_outside_sandbox_range')
            self.grab_position = position
            self.paused = False
        elif op == 'release':
            self.grab_position = None
            self.paused = False
        elif op == 'drop':
            seed = int(cmd.get('seed', np.random.SeedSequence().entropy)) % (2 ** 32)
            rng = np.random.default_rng(seed)
            quat = vector(cmd['orientation_quat'], 4, 'orientation') if 'orientation_quat' in cmd else rng.normal(size=4)
            if np.linalg.norm(quat) < 1e-8:
                raise ValueError('zero_orientation')
            enabled = self.policy_enabled
            self.reset()
            self.data.qpos[:3] = [0, 0, .45]
            self.data.qpos[3:7] = quat / np.linalg.norm(quat)
            self.data.qpos[self.qadr] = self.home
            self.data.qvel.fill(0)
            self.data.qacc_warmstart.fill(0)
            self.previous_action.fill(0)
            self.force_until = 0
            self.grab_position = None
            self.drop_seed = seed
            self.paused = False
            mujoco.mj_forward(self.model, self.data)
            if enabled:
                self.set_policy(True)
        else:
            raise ValueError('unknown_simulation_operation')

    def state(self):
        self.seq += 1
        if self.model is None:
            return {'type': 'state', 'seq': self.seq, 'loaded': False, 'paused': True}
        mujoco.mj_forward(self.model, self.data)
        tilt = math.degrees(math.acos(float(np.clip(self.data.xmat[self.root_id].reshape(3, 3)[2, 2], -1, 1))))
        poses = [{'name': name, 'position': self.data.xpos[i].tolist(), 'quaternion': self.data.xquat[i].tolist()}
                 for i, name in enumerate(self.body_names, 1)]
        return dict(type='state', seq=self.seq, loaded=True, simTime=float(self.data.time),
                    paused=self.paused, policyEnabled=self.policy_enabled, task=self.task,
                    modelSha=self.model_sha, xmlSha=self.xml_sha, physicsDt=.005, policyDt=.02,
                    activeModel=self.active_model,activeModelSha=self.binding['models'][self.active_model]['sha256'],
                    recovery=self.recovery.status(),command=self.twist.tolist(),inferenceCounts=self.inference_counts,
                    actionAlpha=.45,rawPreviousAction=self.previous_action.tolist(),actuator='HLS1910 BAM m6 CPU, firmware P6, 7.4V, fixed20ms lag',
                    policySourceSha=self.binding['sources']['policy.py']['sha256'],bamSha=self.binding['bam_sha256'],
                    extraGroundContacts=self.extra_contacts,
                    engine=f'MuJoCo {mujoco.__version__}', bodyNames=self.body_names,
                    body_poses=poses, jointNames=list(JOINTS),
                    joint_positions_rad=self.data.qpos[self.qadr].tolist(),
                    tiltDeg=tilt, heightM=float(self.data.qpos[2]),
                    force=dict(worldN=self.force.tolist() if self.data.time < self.force_until else [0, 0, 0],
                               remainingS=max(0., self.force_until - float(self.data.time)),
                               body=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.force_body)),
                    grabbed=self.grab_position is not None, dropSeed=self.drop_seed,
                    policySteps=self.policy_steps, physicsSteps=self.physics_steps,
                    feedbackFilterSteps=self.feedback_filter_samples, receiverDt=.01,
                    massKg=float(self.model.body_mass.sum()), contacts=int(self.data.ncon),
                    clippedActions=0,targetsBeyondJointLimits=getattr(self,'outside_ranges',0),
                    scope='交互仿真含头腿落地碰撞扩展，训练环境有差异；固定参数单环境CPU，D20未经BAM辨识；无硬件控制')


def main():
    incoming = queue.Queue(maxsize=256)

    def read_commands():
        for line in sys.stdin:
            try:
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise ValueError('command_must_be_object')
                incoming.put(value, timeout=1)
            except Exception as exc:
                incoming.put({'op': '_input_error', 'error': str(exc)})
        incoming.put(None)

    threading.Thread(target=read_commands, daemon=True).start()
    sim = Simulation()
    next_step = time.perf_counter()
    next_frame = next_step

    def emit(value):
        print(json.dumps(value, separators=(',', ':'), allow_nan=False), flush=True)

    while True:
        try:
            while True:
                cmd = incoming.get_nowait()
                if cmd is None:
                    return
                try:
                    sim.command(cmd)
                    emit({'type': 'ack', 'id': cmd.get('id'), 'op': cmd.get('op'), 'ok': True})
                    emit(sim.state())
                except Exception as exc:
                    emit({'type': 'ack', 'id': cmd.get('id'), 'op': cmd.get('op'), 'ok': False,
                          'error': f'{type(exc).__name__}: {exc}'})
        except queue.Empty:
            pass
        now = time.perf_counter()
        if sim.model is not None and not sim.paused:
            # Never skip simulation steps. Slow hosts run below wall-clock rate.
            count = min(4, max(0, int((now - next_step) / .005) + 1))
            if count:
                try:
                    sim.step(count)
                except Exception as exc:
                    sim.paused = True
                    sim.set_policy(False)
                    emit({'type': 'error', 'error': str(exc)})
                next_step += count * .005
                if now - next_step > .1:
                    next_step = now
        else:
            next_step = now
        if now >= next_frame:
            emit(sim.state())
            next_frame = now + 1 / 30
        time.sleep(.001)


if __name__ == '__main__':
    main()
