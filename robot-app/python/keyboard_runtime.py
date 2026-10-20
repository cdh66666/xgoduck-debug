"""Explicit keyboard extensions around the unchanged official policy math.

No IO at import; no ARM or servo commands. The existing Controller still owns
feedback admission, Recovery, PD/MCU protocol, STOP, and every final output.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
import time

from policy import Policy

FAST_FILE = 'xgoduck_fast_forward.onnx'
FAST_SHA256 = '9391ee2255977546a4b308302c409d64a408fe4383d88abadc554f3452552247'
FAST_SPEED = .3
FORWARD_WALK_SPEED = .2
FAST_BRAKE_S = 3.0
MAX_PICK_PHASE = .18
PICK_PHASE_RATE = .08
MIX_END_PHASE = .06
CONTROL_PERIOD = .02


class KeyboardRuntimeMixin:
    def _keyboard_init(self):
        self.fast_policy = Policy()
        path = Path(__file__).resolve().parent/FAST_FILE
        if path.is_file():
            if hashlib.sha256(path.read_bytes()).hexdigest() != FAST_SHA256:
                raise RuntimeError('The limited forward specialist does not match its admitted artifact')
            ok,message = self.fast_policy.load(path)
            if not ok:raise RuntimeError(message)
            self.model_message += '; limited-forward '+message
        self._keyboard_cancel_locked()

    def _keyboard_cancel_locked(self):
        self.manual_pick_phase = 0.0
        self.manual_pick_target = 0.0
        self.manual_pick_returning = False
        self.fast_requested = False
        self.fast_speed = FORWARD_WALK_SPEED
        self.fast_shift_requested = False
        self.fast_active = False
        self.fast_brake_until = 0.0
        self._keyboard_policy = None
        self._keyboard_manual_blending = False
        self._keyboard_recovery_latched = False

    def _keyboard_recovery_command_locked(self,twist,head,depth,forward,fast,navigation=False):
        if self.mode!='policy':return False
        if self.recovery.phase!='walk':
            self._keyboard_cancel_locked()
            self._keyboard_recovery_latched=True
            self.twist,self.head=[0.]*3,[0.]*4
            return True
        if self._keyboard_recovery_latched:
            neutral=(twist is not None and head is not None and all(abs(v)<1e-9 for v in (*twist,*head))
                     and type(depth) in (int,float) and depth==0 and forward is False and fast is False and navigation is False)
            self.twist,self.head=[0.]*3,[0.]*4
            if neutral:self._keyboard_recovery_latched=False
            return True
        return False

    def _keyboard_command_locked(self, twist, depth, fast, forward=None):
        if depth is not None:
            if isinstance(depth,bool):raise ValueError('manual_pick_depth must be a finite number within 0..1')
            depth=float(depth)
            if not math.isfinite(depth) or not 0<=depth<=1:
                raise ValueError('manual_pick_depth must be a finite number within 0..1')
        if type(fast) is not bool:raise ValueError('fast_forward must be boolean')
        if forward is None:forward=fast  # Compatibility with the first .3-only packet.
        if type(forward) is not bool:raise ValueError('forward_specialist must be boolean')
        if fast and not forward:raise ValueError('fast_forward requires forward_specialist')
        if depth is not None and depth>0 and twist is None and any(abs(v)>1e-9 for v in self.twist):
            raise ValueError('Provide an explicit zero motion command before manual crouch')
        if (depth is not None and depth>0) or forward:
            if self.mode!='policy' or self.recovery.phase!='walk':
                raise ValueError('Manual crouch/forward specialist requires explicitly enabled upright policy mode')
            if self.pick_active or self._roulade_on(time.monotonic()):
                raise ValueError('Finish the current automatic action before manual crouch or fast forward')
        speed=FAST_SPEED if fast else FORWARD_WALK_SPEED
        if forward:
            if not self.fast_policy.loaded:raise ValueError('The admitted 0.2/0.3 m/s forward specialist is unavailable')
            if twist is None or any(abs(a-b)>1e-7 for a,b in zip(twist,[speed,0.,0.])):
                raise ValueError('forward_specialist admits only explicit straight 0.2 or Shift 0.3 commands')
        if depth is not None:self.manual_pick_target=depth*MAX_PICK_PHASE
        self.fast_requested=forward
        if forward:self.fast_speed=speed
        self.fast_shift_requested=fast
        if twist is not None and any(abs(v)>1e-9 for v in twist):
            # The client also clears its persistent depth intent when a motion
            # key is pressed. Never resume an old crouch on walking-key release.
            self.manual_pick_target=0.0
            self.manual_pick_returning=self.manual_pick_phase>0

    def _keyboard_handoff_locked(self, selected):
        previous=self._keyboard_policy
        if previous is not selected and previous is not None:
            # HOME/order/action scale are the same runtime contract. Carry raw
            # history AND filtered action exactly; never reset at a handoff.
            with self.policy_lock:
                selected.prev_action[:]=previous.prev_action
                selected._act_filt[:]=previous._act_filt
            self.policy_needs_reset=False
        self._keyboard_policy=selected

    def _keyboard_select_locked(self, selected, twist, head, mouth, *, mode, phase, valid, now, automatic):
        self._keyboard_manual_blending=False
        if mode!='policy' or phase!='walk':
            self._keyboard_cancel_locked()
            if mode=='policy' and phase!='walk':self._keyboard_recovery_latched=True
            return selected,twist,head,mouth
        if automatic:
            self._keyboard_policy=None
            return selected,twist,head,mouth
        manual=self.manual_pick_target>0 or self.manual_pick_phase>0
        if manual and self.fast_active:
            # K never preempts the admitted fast->zero->official switch path.
            # Retain the manual target, brake at zero for the same three seconds,
            # then run one original-walk zero tick before entering Pick.
            self.fast_requested=False
            if not self.fast_brake_until:self.fast_brake_until=now+FAST_BRAKE_S
            if now<self.fast_brake_until:
                selected=self.fast_policy
            else:
                self.fast_active=False
                self.fast_brake_until=0.0
                selected=self.policy
            self._keyboard_handoff_locked(selected)
            return selected,[0.]*3,[0.]*4,mouth
        if manual:
            self.fast_requested=False
            self.fast_active=False
            self.fast_brake_until=0.0
            self.manual_pick_returning=self.manual_pick_target < self.manual_pick_phase
            if valid:
                change=max(-PICK_PHASE_RATE*CONTROL_PERIOD,min(PICK_PHASE_RATE*CONTROL_PERIOD,self.manual_pick_target-self.manual_pick_phase))
                self.manual_pick_phase=max(0.,min(MAX_PICK_PHASE,self.manual_pick_phase+change))
            if self.manual_pick_phase>1e-12 or self.manual_pick_target>0:
                selected=self.pick_policy
                angle=2*math.pi*self.manual_pick_phase
                twist,head=[math.cos(angle),math.sin(angle),0.],[0.]*4
                self._keyboard_handoff_locked(selected)
                self._keyboard_manual_blending=True
                return selected,twist,head,mouth
            self.manual_pick_phase=0.0
            self.manual_pick_returning=False
        if self.fast_requested:
            self.fast_active=True
            self.fast_brake_until=0.0
            selected,twist,head=self.fast_policy,[self.fast_speed,0.,0.],[0.]*4
        elif self.fast_active:
            if not self.fast_brake_until:self.fast_brake_until=now+FAST_BRAKE_S
            if now<self.fast_brake_until:
                selected,twist,head=self.fast_policy,[0.]*3,[0.]*4
            else:
                self.fast_active=False
                self.fast_brake_until=0.0
                selected=self.policy
        self._keyboard_handoff_locked(selected)
        return selected,twist,head,mouth

    @staticmethod
    def _keyboard_mix_weight(phase):
        x=max(0.,min(1.,phase/MIX_END_PHASE))
        return x*x*(3.-2.*x)

    def _keyboard_infer_manual_locked(self,filt):
        """Both unchanged actors see identical feedback and previous mixed state.

        Mixing the two original .45-IIR outputs equals applying that same IIR
        once to the mixed raw outputs. Store both mixed states on both sessions.
        The caller already owns the original policy_lock.
        """
        previous_raw=self.pick_policy.prev_action.copy()
        previous_filtered=self.pick_policy._act_filt.copy()
        angle=2*math.pi*self.manual_pick_phase
        outputs=[];raw=[]
        for policy,twist in ((self.policy,[0.,0.,0.]),(self.pick_policy,[math.cos(angle),math.sin(angle),0.])):
            policy.prev_action[:]=previous_raw
            policy._act_filt[:]=previous_filtered
            outputs.append(policy.infer(filt['grav'],filt['gyro_rad'],filt['angles_deg'],filt['vels_deg'],twist,[0.]*4))
            raw.append(policy.prev_action.copy())
        weight=self._keyboard_mix_weight(self.manual_pick_phase)
        mixed_raw=(1-weight)*raw[0]+weight*raw[1]
        mixed_filtered=(1-weight)*outputs[0]+weight*outputs[1]
        for policy in (self.policy,self.pick_policy):
            policy.prev_action[:]=mixed_raw
            policy._act_filt[:]=mixed_filtered
        return mixed_filtered

    def _keyboard_state_locked(self, now):
        active=self.manual_pick_phase>1e-12 or self.manual_pick_target>0
        forward={'available':self.fast_policy.loaded,'requested':self.fast_requested,
                 'active':self.fast_active,'accelerated':self.fast_shift_requested and self.fast_requested,
                 'braking':self.fast_active and not self.fast_requested,
                 'brake_remaining_s':max(0.,self.fast_brake_until-now),
                 'speed':self.fast_speed if self.fast_requested else 0.,'allowed_forward_points':[.2,.3],
                 'model':FAST_FILE,'head_deferred':self.fast_active and any(abs(v)>1e-8 for v in self.head),
                 'scope':'straight forward 0.2/0.3 only; original S/A/D and combination commands retained'}
        return {
            'motion_recovery_latched':self._keyboard_recovery_latched,
            'manual_pick':{'active':active,'depth':self.manual_pick_phase/MAX_PICK_PHASE,
                           'target_depth':self.manual_pick_target/MAX_PICK_PHASE,
                           'phase':self.manual_pick_phase,'max_phase':MAX_PICK_PHASE,
                           'mix_weight':self._keyboard_mix_weight(self.manual_pick_phase),
                           'mix_end_phase':MIX_END_PHASE,'mix_is_measured_pose':False,
                           'returning':self.manual_pick_returning,
                           'waiting_for_fast_stop':active and self.fast_active,
                           'yaw_deferred':active and any(abs(v)>1e-8 for v in self.head[2:]),
                           'mouth_owner':'user','phase_is_measured_pose':False},
            'forward_specialist':forward,'fast_forward':dict(forward)
        }


"""Append to the frozen v1 helper; no import/network/device side effects."""

NAVIGATION_FILE = 'xgoduck_walk.onnx'
NAVIGATION_SHA256 = '5727b909d5ea1a0f9ff63bf4cea9707339dea6a522374c7a120da633605b923f'
NAVIGATION_POINTS = ((-.3,0.,0.),(0.,0.,1.),(0.,0.,-1.),(0.,0.,0.))


class NavigationKeyboardRuntimeMixin(KeyboardRuntimeMixin):
    def _keyboard_init(self):
        super()._keyboard_init()
        self.navigation_policy=self.policy
        path=Path(__file__).resolve().parent/NAVIGATION_FILE
        if path.is_file():
            if hashlib.sha256(path.read_bytes()).hexdigest()!=NAVIGATION_SHA256:
                raise RuntimeError('The retained official navigation artifact changed')
        self.model_message+='; retained official navigation: reverse0.3 / yaw+/-1 only'

    def _keyboard_cancel_locked(self):
        super()._keyboard_cancel_locked()
        self.navigation_requested=False
        self.navigation_active=False
        self.navigation_command=[0.,0.,0.]
        self._motion_active=None
        self._motion_brake_until=0.

    def _keyboard_command_locked(self,twist,depth,fast,forward=None,navigation=False):
        if type(navigation) is not bool:raise ValueError('navigation_specialist must be boolean')
        requested_forward=fast if forward is None else forward
        if navigation and requested_forward:raise ValueError('Forward and navigation specialists are mutually exclusive')
        if navigation:
            if self.mode!='policy' or self.recovery.phase!='walk' or self.pick_active or self._roulade_on(time.monotonic()):
                raise ValueError('Navigation requires explicitly enabled upright policy with no automatic action')
            if not self.navigation_policy.loaded:raise ValueError('The separately admitted navigation policy is unavailable')
            if twist is None or not any(all(abs(a-b)<1e-7 for a,b in zip(twist,point)) for point in NAVIGATION_POINTS):
                raise ValueError('Navigation admits only pure reverse 0.3, pure yaw +/-1, or zero')
        super()._keyboard_command_locked(twist,depth,fast,forward)
        self.navigation_requested=navigation
        if navigation:self.navigation_command=list(twist)

    def _motion_policy(self,kind):
        return self.fast_policy if kind=='forward' else self.navigation_policy

    def _motion_aliases_locked(self):
        self.fast_active=self._motion_active=='forward'
        self.navigation_active=self._motion_active=='navigation'
        self.fast_brake_until=self._motion_brake_until if self.fast_active else 0.

    def _keyboard_select_locked(self,selected,twist,head,mouth,*,mode,phase,valid,now,automatic):
        self._keyboard_manual_blending=False
        if mode!='policy' or phase!='walk':
            self._keyboard_cancel_locked()
            if mode=='policy' and phase!='walk':self._keyboard_recovery_latched=True
            return selected,twist,head,mouth
        if automatic:
            self._keyboard_policy=None;return selected,twist,head,mouth
        desired='forward' if self.fast_requested else 'navigation' if self.navigation_requested else None
        manual=self.manual_pick_target>0 or self.manual_pick_phase>0
        if self._motion_active:
            if manual or desired!=self._motion_active:
                if not self._motion_brake_until:self._motion_brake_until=now+FAST_BRAKE_S
                if now<self._motion_brake_until:
                    selected=self._motion_policy(self._motion_active)
                else:
                    self._motion_active=None;self._motion_brake_until=0.;selected=self.policy
                self._motion_aliases_locked();self._keyboard_handoff_locked(selected)
                # The expiry tick is exactly one original-walk zero frame. The
                # next tick considers only the latest still-held request.
                return selected,[0.]*3,[0.]*4,mouth
            self._motion_brake_until=0.
            selected=self._motion_policy(self._motion_active)
            twist=[self.fast_speed,0.,0.] if self.fast_active else list(self.navigation_command)
            self._motion_aliases_locked();self._keyboard_handoff_locked(selected)
            return selected,twist,[0.]*4,mouth
        if manual:
            self.manual_pick_returning=self.manual_pick_target<self.manual_pick_phase
            if valid:
                delta=max(-PICK_PHASE_RATE*CONTROL_PERIOD,min(PICK_PHASE_RATE*CONTROL_PERIOD,self.manual_pick_target-self.manual_pick_phase))
                self.manual_pick_phase=max(0.,min(MAX_PICK_PHASE,self.manual_pick_phase+delta))
            if self.manual_pick_phase>1e-12 or self.manual_pick_target>0:
                self._keyboard_manual_blending=True;selected=self.pick_policy
                angle=2*math.pi*self.manual_pick_phase
                self._keyboard_handoff_locked(selected)
                return selected,[math.cos(angle),math.sin(angle),0.],[0.]*4,mouth
            self.manual_pick_phase=0.;self.manual_pick_returning=False
            self._keyboard_handoff_locked(self.policy)
            return self.policy,[0.]*3,head if desired is None else [0.]*4,mouth
        if desired:
            self._motion_active=desired;self._motion_aliases_locked()
            selected=self._motion_policy(desired)
            twist=[self.fast_speed,0.,0.] if desired=='forward' else list(self.navigation_command)
            self._keyboard_handoff_locked(selected)
            return selected,twist,[0.]*4,mouth
        self._motion_aliases_locked();self._keyboard_handoff_locked(selected)
        return selected,twist,head,mouth

    def _keyboard_state_locked(self,now):
        value=super()._keyboard_state_locked(now)
        braking=self.navigation_active and (self.manual_pick_target>0 or self.manual_pick_phase>0 or not self.navigation_requested or self.fast_requested)
        value['manual_pick']['waiting_for_fast_stop']=value['manual_pick']['active'] and self._motion_active is not None
        value['navigation_specialist']={
            'available':self.navigation_policy.loaded,'requested':self.navigation_requested,
            'active':self.navigation_active,'braking':braking,
            'brake_remaining_s':max(0.,self._motion_brake_until-now) if self.navigation_active else 0.,
            'command':list(self.navigation_command) if self.navigation_requested and not braking else [0.]*3,
            'model':NAVIGATION_FILE,'head_deferred':self.navigation_active and any(abs(v)>1e-8 for v in self.head),
            'source':'retained_official','artifact_sha256':NAVIGATION_SHA256,
            'allowed_points':[list(point) for point in NAVIGATION_POINTS],
            'pending_profile':'forward' if self.fast_requested else 'navigation' if self.navigation_requested else None,
            'same_navigation_model_target_changes':'direct, no reset; requires its separate continuous admission'
        }
        return value


KeyboardRuntimeMixin=NavigationKeyboardRuntimeMixin
