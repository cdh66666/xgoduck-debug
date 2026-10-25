"""Board-local ownership of the trained sit policy; no externally timed setpoints."""
import math,time
from tray_guard import require_motion_clear
from sit_policy import SitPolicy,SIT_SECONDS

class SitRuntimeMixin:
    def _sit_init(self):
        self.sit_policy=None;self.sit_active=False;self.sit_direction=1.
        self.sit_started=0.;self.sit_paused_phase=None;self.sit_error='';self.sit_limit=1.
        self.sit_load_error=''

    def prepare_sit(self,path=None):
        with self.lock:
            if self.mode in ('hold','policy') or self.feedback and self.feedback.enabled:
                raise ValueError('Prepare the sit model only while unloaded')
            policy=SitPolicy();ok,message=policy.load(path)
            if not ok:
                self.sit_load_error=message
                return dict(ok=False,error=message,available=self.sit_policy is not None)
            self.sit_policy=policy
            self.sit_load_error=''
        return dict(ok=True,model=policy.file,model_sha256=policy.artifact_sha256,control_hz=50)

    def _sit_phase_locked(self,now):
        return self.sit_paused_phase if self.sit_paused_phase is not None else max(0.,min(self.sit_limit,(now-self.sit_started)/SIT_SECONDS))

    def _sit_mode_locked(self,mode):
        if mode!='policy' or self.mode!='policy':
            self.sit_active=False;return
        if self.sit_active:
            if self.sit_direction<0 and self._sit_status_locked(time.monotonic())['pose_ready']:
                self.sit_active=False;self.generation+=1;self.policy_needs_reset=True;self.recovery.reset()
            else:raise ValueError('Use slow stand before resuming walking; OFF remains emergency stop')

    def _sit_command_locked(self,twist,head,mouth):
        if self.sit_active and self.mode=='policy' and (
            twist is not None and any(abs(v)>.001 for v in twist) or
            head is not None and any(abs(v)>.001 for v in head) or
            mouth is not None and abs(mouth)>.001):
            raise ValueError('Sit model owns the pose; use slow stand before travel/head commands')

    def start_sit(self,phase_limit=1.):
        phase_limit=float(phase_limit)
        if not math.isfinite(phase_limit) or not .1<=phase_limit<=1:raise ValueError('Invalid seated phase limit')
        with self.lock:
            fb=self._require_feedback((1<<15)-1);require_motion_clear(fb)
            if self.sit_active:
                now=time.monotonic();phase=self._sit_phase_locked(now)
                if self.sit_direction<0 or self.sit_error or phase_limit<phase:raise ValueError('Cannot rewind or resume an interrupted sit')
                self.sit_started=now-phase*SIT_SECONDS;self.sit_limit=phase_limit
                return self._sit_status_locked(now)
            if not self.sit_policy or self.mode!='policy' or not fb.enabled:
                raise ValueError('Sitting requires a prepared model and enabled standing policy')
            g=self.filtered['grav'];tilt=math.degrees(math.atan2(math.hypot(g[0],g[1]),-g[2]))
            if tilt>15 or self.recovery.phase!='walk' or any(abs(v)>.001 for v in self.twist) or self.pick_active or self.roulade_until>time.monotonic():
                raise ValueError('Zero travel and settle upright before sitting')
            self.sit_policy.seed_targets(self.last_action)
            self.sit_active=True;self.sit_direction=1.;self.sit_started=time.monotonic()
            self.sit_paused_phase=None;self.sit_error='';self.sit_limit=phase_limit;self.generation+=1;self.policy_needs_reset=False
            self.head=[0.]*4;self.mouth=0.;self.web_time=time.monotonic()
            return self._sit_status_locked(self.sit_started)

    def start_sit_stand(self):
        with self.lock:
            fb=self._require_feedback((1<<15)-1);require_motion_clear(fb)
            if not self.sit_active or self.mode!='policy' or not fb.enabled or self.sit_error or self.sit_direction<0 or not self._sit_status_locked(time.monotonic())['pose_ready']:
                raise ValueError('Slow stand requires the completed supported seated pose')
            self.sit_policy.seed_targets(self.last_action);self.sit_direction=-1.;self.sit_started=time.monotonic();self.sit_limit=1.
            self.generation+=1;self.policy_needs_reset=False;self.web_time=time.monotonic()
            return self._sit_status_locked(self.sit_started)

    def _sit_select_locked(self,now,gravity,feedback):
        phase=self._sit_phase_locked(now)
        g=gravity;tilt=math.degrees(math.atan2(math.hypot(g[0],g[1]),-g[2]))
        tracking=max(abs(a-b) for a,b in zip(self.last_action,feedback.angles))
        if not self.sit_error and (tilt>20 or tracking>20):
            self.sit_paused_phase=phase;self.sit_error='Sit progression paused: excessive tilt or joint tracking error'
        self.sit_policy.phase=self._sit_phase_locked(now);self.sit_policy.direction=self.sit_direction
        return self.sit_policy

    def _sit_status_locked(self,now):
        active=self.sit_active and self.mode=='policy';phase=self._sit_phase_locked(now) if active else 0.
        ready=False;fb=self.feedback
        if active and phase>=1 and not self.sit_error and fb and fb.enabled and fb.imu_ok and fb.mask==(1<<15)-1 and now-self.rx_time<.15:
            g=self.filtered['grav'];tilt=math.degrees(math.atan2(math.hypot(g[0],g[1]),-g[2]));q=fb.angles
            ready=tilt<15 and (q[2]>30 and q[3]>60 and q[7]<-30 and q[8]<-60 if self.sit_direction>0 else abs(q[3])<12 and abs(q[8])<12)
        stage=('seated' if self.sit_direction>0 else 'standing') if phase>=1 else ('lowering' if self.sit_direction>0 else 'raising')
        if active and phase>=1 and not ready:stage='settling'
        return dict(available=self.sit_policy is not None,active=active,phase=stage if active else 'inactive',pose_ready=ready,
                    progress=phase,phase_limit=self.sit_limit,error=self.sit_error if active else self.sit_load_error,duration_s=SIT_SECONDS,control_hz=50,
                    model=self.sit_policy.file if self.sit_policy else None,
                    model_sha256=self.sit_policy.artifact_sha256 if self.sit_policy else None,
                    last_step=self.sit_policy.last_inference if active else None)
