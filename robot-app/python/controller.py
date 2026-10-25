"""100Hz receive/filter callback and independent 50Hz ONNX loop."""
import collections
import json
import math
import threading
import time
from pathlib import Path
from policy import Policy
from gyro_bias_control import FrozenGyroBias
from recovery import Recovery
from proc import SignalFilter
from rl_core import MOTOR_IDS, HOME_DEG, DEFAULT_DEG, GRAVITY_TAU, IMU_BIAS_DEG, MOUTH_INDEX, MOUTH_MAX_DEG, HEAD_YAW_INDEX, HEAD_YAW_LIMIT_DEG
from wire import *
from display_telemetry import DisplayTelemetryCache
from current_telemetry import CurrentTelemetry
from factory_calibration import FactoryCalibrationMixin, atomic_json
from tray_guard import is_tray_pose, require_motion_clear, TRAY_REASON

FACTORY_ZERO = [
    2236, 2005, 1912, 1866, 1945,
    2127, 2135, 1914, 2228, 2206,
    2550, 2060, 2128, 2034, 2190,
]
ZERO_PATH = Path(__file__).resolve().parents[1] / 'data' / 'zero_pos.json'
EXCLUSIVE_MODES = ('calibrate', 'servo_debug')
PICK_PERIOD_S = 4.0
PICK_MOUTH_CLOSE_PHI = 0.4
ROULADE_S = 1.9
ROULADE_ALPHA = 0.15
CONTROL_PERIOD = 0.02

from keyboard_runtime import KeyboardRuntimeMixin
from sit_runtime import SitRuntimeMixin

class Controller(SitRuntimeMixin, KeyboardRuntimeMixin, FactoryCalibrationMixin):
    def __init__(self, bridge):
        self.bridge = bridge
        self.lock = threading.RLock()
        self.servo_op_lock = threading.Lock()
        self.policy_lock = threading.Lock()
        # The walk export declares its own HOME (hip_pitch/ankle 24.01 deg). Forcing
        # the runtime 22 deg here tells the policy it is standing 2 deg away from
        # where it was trained, on every frame, and it answers by moving the legs.
        # Its own metadata is the only correct reference for this model.
        self.policy = Policy()
        ok, message = self.policy.load()
        if not ok:
            raise RuntimeError(message)
        self.model_message = message
        self.getup_policy = Policy(reference_home=True)
        ok, message = self.getup_policy.load('xgoduck_getup.onnx')
        if not ok:
            raise RuntimeError(message)
        self.model_message += '; ' + message
        self.pick_policy = Policy(reference_home=True)
        ok, message = self.pick_policy.load('xgoduck_pick.onnx')
        if not ok:
            raise RuntimeError(message)
        self.model_message += '; ' + message
        self.roulade_policy = Policy(reference_home=True)
        ok, message = self.roulade_policy.load('xgoduck_roulade.onnx')
        if not ok:
            raise RuntimeError(message)
        self.model_message += '; ' + message
        self.recovery = Recovery()
        self.policy_needs_reset = True
        self.pick_active = False
        self.pick_phi = 0.0
        self.roulade_until = 0.0
        self._roulade_alpha = None
        self._warm_index = 0
        self.filter = SignalFilter()
        self.gyro_bias_control = FrozenGyroBias()
        self._gyro_control_epoch = None
        self.feedback = self.filtered = None
        self._retired_feedback_boots = collections.deque(maxlen=16)
        self._feedback_restart_candidate = None
        self._feedback_receive_epoch = 0
        self.out_of_order_frames = 0
        self._reset_filter_clock()
        self.rx_time = 0.0
        self._joint_rx_times = [None]*15
        self.mode = 'shadow'
        self.generation = 0
        self.twist, self.head, self.mouth = [0.0]*3, [0.0]*4, 0.0
        self.web_time = 0.0
        self.remote_command_timed_out = False
        self.arm_pending = False
        self._sit_init()
        self.seq = self.received = self.inferred = self.sent = self.bad_frames = self.gaps = 0
        self.last_error = ''
        self.stop_event = threading.Event()
        self.rx_times = collections.deque(maxlen=1000)
        self.infer_times = collections.deque(maxlen=500)
        self.infer_start_times = collections.deque(maxlen=500)
        self.infer_ms = collections.deque(maxlen=500)
        self.rx_intervals = collections.deque(maxlen=1000)
        self.loop_missed = 0
        self.intent_lock = threading.Lock()
        self.pending_intent = None
        self.intent_sequence = self.intent_overwritten = 0
        self.intent_result = None
        self.active_intent_seq = 0
        self.last_action = [0.0]*15
        self.last_policy_step = None
        self.head_yaw_output = None
        self.started = time.monotonic()
        self.boot_pd = {}
        self.raw_pos = list(FACTORY_ZERO)
        self.zero_pos = list(FACTORY_ZERO)
        self.cal_done = [False]*15
        self.cal_targets = [2047]*15
        self.raw_mask = 0
        self.servo_reply = None
        self.servo_reply_event = threading.Event()
        self.servo_expected_reply = None
        self._reset_zero_sync()
        self._load_zeros()
        self._keyboard_init()
        self._factory_init(ZERO_PATH)
        self.display_telemetry=DisplayTelemetryCache()
        self.current_telemetry=CurrentTelemetry()

    def _load_zeros(self):
        try:
            data = json.loads(ZERO_PATH.read_text(encoding='utf-8'))
            values = [int(v) for v in data['zero_pos']]
            if len(values) != 15 or any(not 0 <= v <= 4095 for v in values):
                return
            self.zero_pos = values
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass

    def _save_zeros(self):
        self._reset_zero_sync()
        ZERO_PATH.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'version': 1,
            'ids': list(MOTOR_IDS),
            'zero_pos': list(self.zero_pos),
            'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        }
        atomic_json(ZERO_PATH,payload)

    def _reset_zero_sync(self):
        self._zeros_pushed = False  # A later RAW match is the confirmation.
        self._zero_push_at = None
        self._mcu_zero_pos = None

    def _zero_sync_confirmed(self):
        return bool(self._zeros_pushed and not getattr(self, 'factory_pending', None)
                    and self._mcu_zero_pos == self.zero_pos)

    def _push_zeros(self):
        # A lost factory reply can leave EEPROM changed. Never push the old file
        # across an unresolved transaction after a host/MCU restart.
        if getattr(self, 'factory_pending', None):
            return
        if self.feedback and self.feedback.enabled:
            return  # Disarm before changing the MCU position reference.
        now = time.monotonic()
        if self._zero_push_at is not None and now-self._zero_push_at < .25:
            return
        self._zero_push_at = now
        # CAL_SET_ZEROS changes MCU RAM only; retries never recalibrate EEPROM.
        self.bridge.notify('duck_cal', encode_cal(CAL_SET_ZEROS, mask=ALL_MASK, raw=self.zero_pos))

    def receive_display(self,raw):
        with self.lock:
            self.display_telemetry.receive(raw, feedback=self.feedback)

    def receive_boot_pd(self, raw):
        if not isinstance(raw, (bytes, bytearray)) or len(raw)!=32: return
        mask=int.from_bytes(raw[:2],'little')
        with self.lock:
            self.boot_pd={str(id):{'p':raw[2+i],'d':raw[17+i]} for i,id in enumerate(MOTOR_IDS) if mask&(1<<i)}

    def receive_raw(self, raw):
        try:
            pkt = decode_raw(raw)
        except ValueError:
            return
        with self.lock:
            self.raw_pos = pkt['pos']
            self.raw_mask = pkt['mask']
            self._mcu_zero_pos = list(pkt['zero'])
            # The file / completed factory result is authoritative. This RAW may
            # predate the request sent below, so it cannot acknowledge that push.
            self._zeros_pushed = bool(self._zero_push_at is not None
                and not getattr(self, 'factory_pending', None)
                and self._mcu_zero_pos == self.zero_pos)
            if not self._zeros_pushed:
                self._push_zeros()

    def receive_servo_reply(self, raw):
        try:
            pkt = decode_servo_reply(raw)
        except ValueError:
            return
        with self.lock:
            if self.servo_expected_reply != (pkt['op'], pkt['target_id']):
                return
            self.servo_reply = pkt
            self.servo_reply_event.set()

    def _reset_filter_clock(self):
        self._feedback_clock_us = None
        self._feedback_clock_s = 0.0

    def _filter_sample_time(self, sample_us):
        if self._feedback_clock_us is not None:
            delta = elapsed_us(sample_us, self._feedback_clock_us)
            if delta == 0 or delta >= 0x80000000:
                return None  # Do not refresh receive age from a duplicate/backward frame.
            self._feedback_clock_s += delta / 1e6
        self._feedback_clock_us = sample_us
        return self._feedback_clock_s

    def receive(self, raw):
        try:
            fb = decode_state(raw)
        except ValueError:
            with self.lock: self.bad_frames += 1
            return
        now = time.monotonic()
        with self.lock:
            old = self.feedback
            restarted = False
            if old:
                new_boot = fb.boot is not None and old.boot is not None and fb.boot != old.boot
                step = elapsed_us(fb.seq, old.seq)
                if step == 0 and not new_boot: return
                ambiguous = (new_boot and fb.boot in self._retired_feedback_boots
                             or not new_boot and step >= 0x80000000 and fb.boot is not None)
                if ambiguous and not self._restart_progress(fb, now):
                    self.out_of_order_frames += 1
                    return
                # A reused clock marker may restart and catch up past the old
                # sequence while disconnected. Forward progress after losing
                # feedback cannot prove continuity either; withdraw old fit/zeros.
                recovered_after_gap = now-self.rx_time > .15
                if new_boot or step >= 0x80000000 or recovered_after_gap:
                    restarted = True
                    self._feedback_receive_epoch += 1
                    self.display_telemetry.reset()
                    # OFF is an explicit no-inference/no-torque choice, including
                    # after a restart or ambiguous sequence resync. Still reset
                    # every sensor/zero qualification below before any new start.
                    keep_off = self.mode == 'off' and not fb.enabled
                    if new_boot:
                        if fb.boot in self._retired_feedback_boots:
                            self._retired_feedback_boots.remove(fb.boot)
                        self._retired_feedback_boots.append(old.boot)
                    self._factory_reboot()
                    self.mode = 'off' if keep_off else 'shadow'; self.generation += 1
                    self.arm_pending = False; self.filter.reset()
                    self._joint_rx_times = [None]*15
                    self._reset_filter_clock()
                    self.gyro_bias_control.reset(reason='mcu_sequence_restart')
                    self._gyro_control_epoch = None
                    self.recovery.reset(); self.policy_needs_reset = True
                    self._stop_pick_locked(); self._stop_roulade_locked(); self._keyboard_cancel_locked()
                    self._reset_zero_sync()
            self._feedback_restart_candidate = None
            sample_time = self._filter_sample_time(fb.sample_us)
            if sample_time is None:
                return
            for index in range(15):
                if fb.mask & (1 << index):
                    self._joint_rx_times[index] = now
            if old:
                if not restarted:
                    self.gaps += step-1
                self.rx_intervals.append((now-self.rx_time)*1000)
            epoch = self._gyro_bias_epoch(fb, now)
            bias_event = self.gyro_bias_control.observe(acc=fb.acc, gyro=fb.gyro,
                angles=fb.angles, velocities=fb.velocities, seq=fb.seq,
                sample_us=fb.sample_us, mask=fb.mask, imu_ok=fb.imu_ok,
                eligible=self.mode == 'off' and not fb.enabled and not fb.calibrating
                    and not fb.servo_debugging and not self.factory_pending,
                epoch=epoch, now=now)
            if bias_event in ('epoch_changed','imu_invalidated'):
                if self.mode in ('hold','policy'):
                    self.mode = 'shadow'; self.generation += 1
                    self.arm_pending = False; self.last_error = 'IMU/epoch changed; stationary gyro fit required'
                    self.recovery.reset(); self.policy_needs_reset = True
                    self._stop_pick_locked(); self._stop_roulade_locked(); self._keyboard_cancel_locked()
                    self._disarm_now()
                self.gyro_bias_control.reset(epoch, bias_event)
                self.filter.reset()
            self.filtered = self.filter.update(fb.acc, self.gyro_bias_control.correct(fb.gyro), fb.angles, fb.velocities,
                timestamp=sample_time, imu_ok=fb.imu_ok)
            self.feedback, self.rx_time = fb, now
            self._stop_for_tray_locked()
            self.received += 1; self.rx_times.append(now)
            receive_epoch = self._feedback_receive_epoch
        self.current_telemetry.submit(raw, fb, now, receiver_epoch=receive_epoch)

    def _restart_progress(self, fb, now):
        # The firmware marker uses startup clocks and can repeat. An old batch
        # cannot take over immediately; recover a reused marker only after the
        # accepted stream stops and a candidate advances in real and MCU time.
        c = self._feedback_restart_candidate
        if (c is None or c['boot'] != fb.boot or elapsed_us(fb.seq, c['seq']) != 1
                or not 0 < elapsed_us(fb.sample_us, c['us']) <= 20000):
            c = dict(boot=fb.boot, first_at=now, first_us=fb.sample_us)
        c.update(seq=fb.seq, us=fb.sample_us)
        self._feedback_restart_candidate = c
        return (now-self.rx_time >= .15 and now-c['first_at'] >= .15
                and 150000 <= elapsed_us(fb.sample_us, c['first_us']) < 0x80000000)

    def _gyro_bias_epoch(self, fb, now):
        if fb.boot is not None:
            self._gyro_control_epoch = '%016x' % fb.boot
            return self._gyro_control_epoch
        # Only packet identity is borrowed from DQD1. Heading availability,
        # singularities, continuity and display age never gate control.
        identity = self.display_telemetry.snapshot(fb, now, now).get('display_heading', {})
        source_seq = identity.get('source_seq')
        epoch = identity.get('boot_epoch')
        if epoch and source_seq is not None and ((fb.seq-source_seq) & 0xffffffff) < 32:
            self._gyro_control_epoch = epoch
        # A display gap cannot withdraw a known identity. Raw MCU restart
        # invalidates this cache in receive() before a new fit may qualify.
        return self._gyro_control_epoch

    def _require_feedback(self, mask):
        fb = self.feedback
        if not fb or time.monotonic()-self.rx_time > .15:
            raise ValueError('MCU feedback is stale')
        if not fb.imu_ok or (fb.mask & mask) != mask:
            raise ValueError('required servo feedback / IMU missing')
        if not self._zero_sync_confirmed():
            raise ValueError('MCU zero synchronization pending')
        require_motion_clear(fb)
        return fb

    def _stop_for_tray_locked(self):
        if not is_tray_pose(self.feedback):
            return False
        if self.mode in ('hold', 'policy') or self.feedback.enabled:
            self.mode = 'off'; self.generation += 1
            self.last_error = TRAY_REASON
            self.recovery.reset(); self.policy_needs_reset = True
            self._stop_pick_locked(); self._stop_roulade_locked(); self._keyboard_cancel_locked()
            self._disarm_now()
            return True
        return False

    def _stale_joint_ids(self, now):
        # A live IMU/packet does not refresh joints absent from its servo mask.
        return [joint for joint, received in zip(MOTOR_IDS, self._joint_rx_times)
                if received is None or now-received > .15]

    def start_sit(self,phase_limit=1.):
        with self.lock:
            result=super().start_sit(phase_limit)
            self._keyboard_cancel_locked()
            with self.intent_lock:self.pending_intent=None
            return result

    def _disarm_now(self):
        self.arm_pending = False
        self.seq = (self.seq + 1) & 0xffffffff
        sample = self.feedback.sample_us if self.feedback else 0
        self.bridge.notify('duck_command', encode_command(self.seq, sample, DISARM, DEFAULT_DEG))

    def set_mode(self, mode):
        if mode == 'calibrate':
            return self.factory_action('enter')
        if mode not in ('shadow', 'off', 'hold', 'policy', 'calibrate', 'servo_debug'):
            raise ValueError('unknown mode')
        with self.lock:
            self._sit_mode_locked(mode)
            if mode in ('hold', 'policy'):
                require_motion_clear(self.feedback)
            if mode in ('hold','policy') and self.factory_pending:
                raise ValueError('calibration outcome unresolved; inspect pending transaction')
            if mode == self.mode:
                self._expire_remote_intent_locked(time.monotonic())
                self.web_time = time.monotonic()
                return {'mode': self.mode}
            if mode in ('hold', 'policy') and self.mode in EXCLUSIVE_MODES:
                raise ValueError('exit calibrate/servo_debug first')
            if mode in EXCLUSIVE_MODES and self.mode in ('hold', 'policy'):
                raise ValueError('disarm before calibrate/servo_debug')
            fb = None
            if mode in ('hold', 'policy'):
                fb = self._require_feedback(ALL_MASK)
            if mode == 'policy':
                newly_frozen = self.gyro_bias_control.frozen is None
                self.gyro_bias_control.freeze(epoch=self._gyro_bias_epoch(fb, time.monotonic()),
                    now=time.monotonic())
                # Explicit start is the only place a newly frozen correction
                # changes the gravity seed; no live-policy offset adaptation.
                if newly_frozen:
                    before_gravity = list(self.filtered['grav'])
                    self.filter.reset()
                    self.filtered = self.filter.update(fb.acc, self.gyro_bias_control.correct(fb.gyro),
                        fb.angles, fb.velocities, timestamp=self._feedback_clock_s, imu_ok=fb.imu_ok)
                    self.gyro_bias_control.mark_applied(seq=fb.seq, sample_us=fb.sample_us,
                        sample_clock_s=self._feedback_clock_s, now=time.monotonic(), raw_acc=fb.acc,
                        corrected_gyro_dps=self.gyro_bias_control.correct(fb.gyro),
                        before_gravity=before_gravity, after_gravity=self.filtered['grav'])
            previous = self.mode
            self.generation += 1; self.mode = mode
            with self.intent_lock:
                if self.pending_intent is not None:
                    self.intent_result = dict(sequence=self.pending_intent[0], applied=False, error='mode_changed')
                self.pending_intent = None
            self.active_intent_seq = 0
            self.last_policy_step = None
            self.remote_command_timed_out = False
            self.recovery.reset(); self.policy_needs_reset = True
            if mode == 'policy':
                filt = self.filtered
                valid = bool(fb and filt and filt['imu_fusion_ready'] and filt['imu_fusion_updated'])
                self.recovery.start(filt['grav'] if filt else [0,0,0], time.monotonic(), valid)
            self._stop_pick_locked(); self._stop_roulade_locked(); self._keyboard_cancel_locked()
            self.last_error = ''
            self.web_time = time.monotonic()
            self.twist, self.head, self.mouth = [0.0]*3, [0.0]*4, 0.0
            self.arm_pending = fb is not None and not fb.enabled
            if previous == 'calibrate' and mode != 'calibrate':
                self.cal_done = [False]*15
                self.bridge.notify('duck_cal', encode_cal(CAL_EXIT))
            if previous == 'servo_debug' and mode != 'servo_debug':
                self.factory_active = False
                self.factory_held = 0
                self.factory_reads = {}
                self.bridge.notify('duck_servo', encode_servo(SERVO_EXIT, 10))
            if mode == 'off':
                self._disarm_now()
            elif mode == 'calibrate':
                self._disarm_now()
                self.cal_targets = [2047]*15
                self.cal_done = [False]*15
                self.bridge.notify('duck_cal', encode_cal(CAL_ENTER))
            elif mode == 'servo_debug':
                self._disarm_now()
                self.bridge.notify('duck_servo', encode_servo(SERVO_ENTER, 10))
            elif previous in EXCLUSIVE_MODES:
                self._disarm_now()
        with self.policy_lock:
            self.policy.reset_action_filter()
            self.getup_policy.reset_action_filter()
            self.pick_policy.reset_action_filter()
            self.roulade_policy.reset_action_filter()
        return {'mode': mode}

    def _models(self):
        return (self.policy, self.getup_policy, self.pick_policy, self.roulade_policy)

    def _apply_alphas_locked(self, leg, head):
        for model in self._models():
            model.set_action_alphas(leg=leg, head=head)

    def set_action_alphas(self, **values):
        with self.lock:
            with self.policy_lock:
                leg, head = self.policy.set_action_alphas(**values)
                if self._roulade_alpha is not None:
                    self._roulade_alpha = (leg, head)
                    self._apply_alphas_locked(ROULADE_ALPHA, ROULADE_ALPHA)
                else:
                    self._apply_alphas_locked(leg, head)
                return leg, head

    def _stop_pick_locked(self):
        self.pick_active = False
        self.pick_phi = 0.0

    def _stop_roulade_locked(self):
        self.roulade_until = 0.0
        saved = self._roulade_alpha
        self._roulade_alpha = None
        if saved is not None:
            with self.policy_lock:
                self._apply_alphas_locked(*saved)

    def _roulade_on(self, now):
        return now < self.roulade_until

    def start_pick(self):
        with self.lock:
            if self.mode not in ('shadow', 'policy'):
                raise ValueError('pick requires shadow/policy mode')
            if self.recovery.phase != 'walk' or self._roulade_on(time.monotonic()) or self.manual_pick_phase>0 or self.manual_pick_target>0 or self.fast_active or self.navigation_active:
                raise ValueError('pick only while walking upright')
            self.pick_active = True
            self.pick_phi = 0.0
            self.mouth = MOUTH_MAX_DEG
            self.policy_needs_reset = True
            self.web_time = time.monotonic()
            self.last_error = ''
        with self.policy_lock:
            self.pick_policy.reset_action_filter()
            self.pick_policy.warm(2)
        return {'ok': True, 'phi': 0.0, 'model': self.pick_policy.file}

    def start_roulade(self):
        with self.lock:
            if self.mode not in ('shadow', 'policy'):
                raise ValueError('roulade requires shadow/policy mode')
            if self.recovery.phase != 'walk' or self.pick_active or self.manual_pick_phase>0 or self.manual_pick_target>0 or self.fast_active or self.navigation_active:
                raise ValueError('roulade only while walking')
            if self._roulade_alpha is None:
                self._roulade_alpha = (self.policy.action_alpha, self.policy.action_alpha_head)
            self.roulade_until = time.monotonic() + ROULADE_S
            self.policy_needs_reset = True
            self.web_time = time.monotonic()
            self.last_error = ''
            with self.policy_lock:
                self._apply_alphas_locked(ROULADE_ALPHA, ROULADE_ALPHA)
                self.roulade_policy.reset_action_filter()
                self.roulade_policy.warm(2)
        return {'ok': True, 'model': self.roulade_policy.file, 'seconds': ROULADE_S}

    def _expire_remote_intent_locked(self, now):
        # Only remote intent expires. Local sensor guards and actor history are
        # independent of the browser/PC connection. A neutral acknowledgement
        # must precede any subsequent movement after an expired command.
        if self.mode == 'policy' and not self.remote_command_timed_out and now-self.web_time > 1.0:
            self._keyboard_cancel_locked()
            self._keyboard_recovery_latched = True
            self.remote_command_timed_out = True

    def command(self, twist=None, head=None, mouth=None, manual_pick_depth=None, forward_specialist=None, fast_forward=False, navigation_specialist=False):
        def values(seq, limits):
            if len(seq)!=len(limits): raise ValueError('wrong command dimension')
            seq = [float(v) for v in seq]
            if any(not math.isfinite(v) or abs(v)>limit for v,limit in zip(seq,limits)):
                raise ValueError('command exceeds bounds')
            return seq
        t = values(twist, [.4,.3,1.0]) if twist is not None else None
        h = values(head, [1.0]*4) if head is not None else None
        m = None
        if mouth is not None:
            m = float(mouth)
            if not math.isfinite(m) or not 0 <= m <= MOUTH_MAX_DEG:
                raise ValueError('command exceeds bounds')
        with self.lock:
            if self.mode in EXCLUSIVE_MODES:
                raise ValueError('motion commands disabled in calibrate/servo_debug')
            self._sit_command_locked(t,h,m)
            self._expire_remote_intent_locked(time.monotonic())
            if self._keyboard_recovery_command_locked(t, h, manual_pick_depth, forward_specialist, fast_forward, navigation_specialist):
                if m is not None and not self.pick_active: self.mouth = m
                if not self._keyboard_recovery_latched:
                    self.remote_command_timed_out = False
                self.web_time = time.monotonic()
                return {'ok': True, 'motion_ignored': True, 'recovering': self.recovery.phase != 'walk'}
            self._keyboard_command_locked(t, manual_pick_depth, fast_forward, forward_specialist, navigation_specialist)
            if t is not None and not self.pick_active: self.twist = t
            if h is not None and not self.pick_active: self.head = h
            if m is not None and not self.pick_active: self.mouth = m
            self.web_time = time.monotonic()
        return {'ok': True}

    def submit_command(self, twist=None, head=None, mouth=None, manual_pick_depth=None,
                       forward_specialist=None, fast_forward=False, navigation_specialist=False):
        """Admit one latest desired state; the HTTP thread never executes it."""
        from release_profile import constrain_command
        values = dict(twist=twist, head=head, mouth=mouth, manual_pick_depth=manual_pick_depth,
            forward_specialist=forward_specialist, fast_forward=fast_forward,
            navigation_specialist=navigation_specialist)
        values = constrain_command({k:v for k,v in values.items() if v is not None})
        received = time.monotonic()
        generation = self.generation
        with self.intent_lock:
            self.intent_sequence += 1
            if self.pending_intent is not None:
                self.intent_overwritten += 1
            self.pending_intent = (self.intent_sequence, generation, received, values)
            sequence = self.intent_sequence
        return dict(ok=True, accepted_seq=sequence, applied=False, apply_on='next_local_50hz_tick')

    def _consume_command(self):
        with self.intent_lock:
            pending, self.pending_intent = self.pending_intent, None
        if pending is None:
            return
        sequence, generation, received, values = pending
        with self.lock:
            now = time.monotonic()
            result = dict(sequence=sequence, received_uptime_s=received-self.started,
                consumed_uptime_s=now-self.started, wait_ms=(now-received)*1000, applied=False)
            if generation != self.generation:
                result['error'] = 'mode_changed'
            elif now-received > 1.0:
                result['error'] = 'intent_expired'
            else:
                try:
                    reply = self.command(**values)
                    # Freshness belongs to receipt, not delayed consumption.
                    self.web_time = received
                    result.update(reply, applied=not reply.get('motion_ignored',False))
                    self.active_intent_seq = sequence
                except (TypeError,ValueError) as exc:
                    result['error'] = str(exc)
            self.intent_result = result

    def cal_targets_set(self, targets):
        values = [int(v) for v in targets]
        if len(values) != 15 or any(not 0 <= v <= 4095 for v in values):
            raise ValueError('15 raw targets in 0..4095 required')
        with self.lock:
            if self.mode != 'calibrate':
                raise ValueError('not in calibrate mode')
            self.cal_targets = values
            self.web_time = time.monotonic()
            self.bridge.notify('duck_cal', encode_cal(CAL_SET_RAW, mask=ALL_MASK, raw=values))
        return {'ok': True, 'targets': values}

    def cal_finish(self, index):
        index = int(index)
        if not 0 <= index < 15:
            raise ValueError('joint index out of range')
        with self.lock:
            if self.mode != 'calibrate':
                raise ValueError('not in calibrate mode')
            self.bridge.notify('duck_cal', encode_cal(CAL_FINISH_ONE, index=index))
            # Optimistically store the last reported encoder reading.
            self.zero_pos[index] = int(self.raw_pos[index])
            self.cal_done[index] = True
            self._save_zeros()
            self.web_time = time.monotonic()
            return {'ok': True, 'index': index, 'zero_pos': self.zero_pos[index],
                    'cal_done': list(self.cal_done)}

    def _require_servo_debug(self):
        if self.mode != 'servo_debug':
            raise ValueError('not in servo_debug mode')
        self.web_time = time.monotonic()

    def servo_op(self, action, target_id, new_id=None, kp=5, kd=20, raw_pos=2047):
        with self.servo_op_lock:
            if action != 'set_id':
                return self._servo_op(action, target_id, new_id, kp, kd, raw_pos)
            target_id, new_id = int(target_id), int(new_id)
            if not 0 <= target_id <= 253 or not 0 <= new_id <= 253:
                raise ValueError('ID must be between 0 and 253')
            if not self._servo_op('ping', target_id)['ok']:
                raise ValueError(f'No servo replied at ID {target_id}; ID was not changed')
            if target_id == new_id:
                return dict(ok=True, op=SERVO_SET_ID, target_id=target_id, new_id=new_id)
            if self._servo_op('ping', new_id)['ok']:
                raise ValueError(f'ID {new_id} is already in use; choose another ID')
            # Keep the checks, write and verification in one serialized operation.
            reply = self._servo_op(action, target_id, new_id, kp, kd, raw_pos)
            time.sleep(0.05)
            if not self._servo_op('ping', new_id)['ok']:
                raise ValueError(f'ID write sent, but ID {new_id} did not reply. Scan again')
            if self._servo_op('ping', target_id)['ok']:
                raise ValueError('Both old and new IDs replied. Connect only one servo and scan again')
            return dict(reply, new_id=new_id)

    def _servo_op(self, action, target_id, new_id=None, kp=5, kd=20, raw_pos=2047):
        target_id = int(target_id)
        with self.lock:
            self._require_servo_debug()
            if action == 'unlock':
                op, payload = SERVO_UNLOCK, encode_servo(SERVO_UNLOCK, target_id)
            elif action == 'set_id':
                new_id = int(new_id)
                if not 0 <= new_id <= 253:
                    raise ValueError('invalid new id')
                op, payload = SERVO_SET_ID, encode_servo(SERVO_SET_ID, target_id, new_id=new_id)
            elif action == 'goto':
                raw_pos = int(raw_pos)
                op, payload = SERVO_GOTO, encode_servo(SERVO_GOTO, target_id, raw_pos=raw_pos)
            elif action == 'set_gains':
                kp, kd = int(kp), int(kd)
                if not 0 <= kp <= 255 or not 0 <= kd <= 255:
                    raise ValueError('kp/kd out of range')
                op = SERVO_SET_PERM_KP_KD
                payload = encode_servo(SERVO_SET_PERM_KP_KD, target_id, kp=kp, kd=kd)
            elif action == 'read':
                op, payload = SERVO_READ, encode_servo(SERVO_READ, target_id)
            elif action == 'ping':
                op, payload = SERVO_PING, encode_servo(SERVO_PING, target_id)
            elif action == 'torque_off':
                op, payload = SERVO_ENTER, encode_servo(SERVO_ENTER, target_id)
            else:
                raise ValueError('unknown servo action')
            self.servo_reply_event.clear()
            self.servo_reply = None
            self.servo_expected_reply = (op, target_id)
        try:
            self.bridge.notify('duck_servo', payload)
            return self._wait_servo_reply(op, target_id=target_id)
        finally:
            with self.lock:
                self.servo_expected_reply = None

    def _wait_servo_reply(self, op, timeout=0.5, target_id=None):
        if not self.servo_reply_event.wait(timeout):
            raise ValueError('servo reply timeout')
        with self.lock:
            reply = self.servo_reply
        if not reply or reply['op'] != op:
            raise ValueError('unexpected servo reply')
        if target_id is not None and reply['target_id'] != target_id:
            raise ValueError('unexpected servo reply ID')
        if not reply['ok'] and op != SERVO_PING:
            raise ValueError('servo operation failed')
        return reply

    def step(self):
        cycle_begin = time.monotonic()
        self._consume_command()
        with self.lock:
            lock_acquired = time.monotonic()
            fb, filt, generation, mode = self.feedback, self.filtered, self.generation, self.mode
            receiver_epoch = self._feedback_receive_epoch
            age = time.monotonic()-self.rx_time
            stale_joints = self._stale_joint_ids(time.monotonic())
            if mode in ('hold','policy'):
                lost = None
                if not fb or age>.15 or not fb.imu_ok:
                    lost = 'sensor feedback lost; disarmed'
                elif not self._zero_sync_confirmed():
                    lost = 'MCU zero synchronization lost; disarmed'
                elif stale_joints:
                    lost = 'servo feedback lost: '+','.join(map(str, stale_joints))+'; disarmed'
                elif mode == 'hold' and time.monotonic()-self.web_time > 1.0:
                    lost = 'web heartbeat lost; disarmed'
                if lost:
                    self.mode = mode = 'shadow'; self.generation += 1
                    self.arm_pending = False; self.last_error = lost
                    generation = self.generation
                    self.recovery.reset(); self.policy_needs_reset = True
                    self._stop_pick_locked(); self._stop_roulade_locked(); self._keyboard_cancel_locked()
                else:
                    self._expire_remote_intent_locked(time.monotonic())
            elif mode in EXCLUSIVE_MODES:
                if time.monotonic()-self.web_time > 2.0 and not (mode == 'servo_debug' and self.factory_active):
                    previous = mode
                    self.mode = mode = 'shadow'; self.generation += 1
                    self.last_error = 'calibrate/servo_debug heartbeat lost; exited'
                    generation = self.generation
                    if previous == 'calibrate':
                        self.cal_done = [False]*15
                        self.bridge.notify('duck_cal', encode_cal(CAL_EXIT))
                    else:
                        self.bridge.notify('duck_servo', encode_servo(SERVO_EXIT, 10))
                    self._disarm_now()
            twist, head, mouth = list(self.twist), list(self.head), self.mouth
            valid = bool(fb and age < .15 and fb.imu_ok and not stale_joints and filt
                         and filt['imu_fusion_ready'] and filt['imu_fusion_updated'])
            now = time.monotonic()
            if self.roulade_until and now >= self.roulade_until:
                self._stop_roulade_locked()
                self.recovery.fall_elapsed = 0.0
                self.policy_needs_reset = True
            rolling = self._roulade_on(now)
            if mode in ('shadow', 'policy') and not rolling and not (self.sit_active and mode=='policy'):
                changed = self.recovery.update(filt['grav'] if filt else [0,0,0],
                                               now, valid)
                self.policy_needs_reset |= changed
                if changed and self.recovery.phase != 'walk':
                    self._stop_pick_locked()
                    self._keyboard_cancel_locked()
                    self.twist, self.head = [0.]*3, [0.]*4
                    self._keyboard_recovery_latched = mode == 'policy'
            phase = self.recovery.phase
            if rolling and phase != 'walk':
                self._stop_roulade_locked()
                rolling = False
            picking = self.pick_active and phase == 'walk' and not rolling
            if rolling:
                twist, head = [0.0]*3, [0.0]*4
                selected = self.roulade_policy
            elif picking:
                phi = self.pick_phi
                angle = 2.0 * math.pi * phi
                twist = [math.cos(angle), math.sin(angle), 0.0]
                head = [0.0]*4
                mouth = MOUTH_MAX_DEG if phi < PICK_MOUTH_CLOSE_PHI else 0.0
                self.mouth = mouth
                selected = self.pick_policy
            else:
                selected = self.policy if phase == 'walk' else self.getup_policy
            selected, twist, head, mouth = self._keyboard_select_locked(selected, twist, head, mouth, mode=mode, phase=phase, valid=valid, now=now, automatic=rolling or picking)
            if self.sit_active and mode=='policy' and valid:
                selected=self._sit_select_locked(now,filt['grav'],fb)
                phase='walk';twist,head,mouth=[0.]*3,[0.]*4,0.
            reset_policy = self.policy_needs_reset
            # ARM sends measured angles only; do not advance an actor for an
            # output that is deliberately replaced and will never be applied.
            inferring = mode in ('shadow','policy') and valid and not self.arm_pending
            if inferring:
                self.policy_needs_reset = False
            exclusive = mode in EXCLUSIVE_MODES
        if exclusive:
            return
        target = list(self.last_action) if self.sit_active and mode=='policy' else ([0.0]*15 if mode == 'hold' else list(DEFAULT_DEG))
        policy_step = None
        if inferring and phase == 'default_pose':
            # Switch to getup immediately, but force action/history to zero for 1 s.
            with self.policy_lock:
                selected.reset_action_filter()
                target = selected.action_to_deg([0.0]*14)
        elif inferring:
            if phase == 'getup':
                twist, head = [0.0]*3, [0.0]*4
            begin = time.monotonic()
            with self.policy_lock:
                if reset_policy:
                    selected.reset_action_filter()
                    # One warm pass keeps the newly selected session's arenas resident.
                    selected.warm(1)
                action = selected.infer(filt['grav'], filt['gyro_rad'], filt['angles_deg'], filt['vels_deg'], twist, head)
                target = selected.action_to_deg(action)
                if selected.last_inference is not None:
                    policy_step = dict(selected.last_inference, phase=phase,
                        source_mcu_seq=fb.seq, source_sample_us=fb.sample_us,
                        source_receiver_epoch=receiver_epoch, requested_targets=list(target))
            finish = time.monotonic()
            with self.lock:
                self.inferred += 1; self.infer_times.append(finish)
                self.infer_start_times.append(begin)
                self.infer_ms.append((finish-begin)*1000)
                if picking:
                    self.pick_phi += CONTROL_PERIOD / PICK_PERIOD_S
                    if self.pick_phi >= 1.0:
                        self._stop_pick_locked()
                        self.policy_needs_reset = True
        with self.lock:
            # A stop arriving during ONNX always wins; never send its obsolete action.
            if self.generation != generation:
                return
            mode = self.mode
            if mode in EXCLUSIVE_MODES:
                return
            if self._stop_for_tray_locked():
                return
            # A RAW mismatch arriving during inference also wins before ARM/TX.
            if mode in ('hold','policy') and not self._zero_sync_confirmed():
                self._disarm_now()
                return
            if mode in ('hold','policy') and self._stale_joint_ids(time.monotonic()):
                stale = self._stale_joint_ids(time.monotonic())
                self.mode = 'shadow'; self.generation += 1
                self.arm_pending = False
                self.last_error = 'servo feedback lost: '+','.join(map(str, stale))+'; disarmed'
                self.recovery.reset(); self.policy_needs_reset = True
                self._stop_pick_locked(); self._stop_roulade_locked(); self._keyboard_cancel_locked()
                self._disarm_now()
                return
            command_mode = {'shadow':DISARM,'off':DISARM,'hold':HOLD,'policy':POLICY}[mode]
            if self.arm_pending:
                command_mode = ARM
                # Initial enable always uses measured angles.
                target = list(fb.angles)
                self.arm_pending = False
            else:
                target = list(target)
                target[MOUTH_INDEX] = 0.0 if mode == 'hold' else mouth
            requested_yaw = target[HEAD_YAW_INDEX]
            target[HEAD_YAW_INDEX] = max(-HEAD_YAW_LIMIT_DEG, min(HEAD_YAW_LIMIT_DEG, requested_yaw))
            self.head_yaw_output = dict(requested_deg=requested_yaw, sent_deg=target[HEAD_YAW_INDEX],
                limited=requested_yaw != target[HEAD_YAW_INDEX], command_mode=command_mode)
            if fb is None: return
            self.last_action = list(target)
            self.seq = (self.seq+1)&0xffffffff
            payload = encode_command(self.seq, fb.sample_us, command_mode, target)
            send_begin = time.monotonic()
            self.bridge.notify('duck_command', payload)
            if policy_step is not None:
                self.last_policy_step = dict(policy_step, command_seq=self.seq, command_mode=command_mode,
                    control_intent_seq=self.active_intent_seq,
                    submitted_uptime_s=time.monotonic()-self.started,
                    timing_ms=dict(lock_wait=(lock_acquired-cycle_begin)*1000,
                        prepare=(begin-lock_acquired)*1000, inference=(finish-begin)*1000,
                        submit_wait=(send_begin-finish)*1000, send=(time.monotonic()-send_begin)*1000),
                    submitted_targets=list(target), submission='Bridge notify returned; match MCU command_seq to confirm acceptance')
            self.sent += 1

    def _keep_warm_idle(self):
        """Run one dummy pass on an idle session so walk/getup/pick/roulade stay hot."""
        with self.lock:
            phase = self.recovery.phase
            rolling = self._roulade_on(time.monotonic()) and phase == 'walk'
            picking = self.pick_active and phase == 'walk' and not rolling
            if rolling:
                active = self.roulade_policy
            elif picking:
                active = self.pick_policy
            elif phase == 'walk':
                active = self.policy
            else:
                active = self.getup_policy
            candidates = [p for p in (self.policy, self.getup_policy, self.pick_policy, self.roulade_policy) if p is not active]
        if not candidates:
            return
        self._warm_index = (self._warm_index + 1) % len(candidates)
        idle = candidates[self._warm_index]
        if self.policy_lock.acquire(blocking=False):
            try:
                idle.warm(1)
            finally:
                self.policy_lock.release()

    def run(self):
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            try:
                self.step()
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc); self.mode = 'off'; self.generation += 1; self.arm_pending = False
                    self._stop_pick_locked(); self._stop_roulade_locked(); self._keyboard_cancel_locked()
            deadline += CONTROL_PERIOD
            now = time.monotonic()
            if now-deadline >= CONTROL_PERIOD:
                # Keep the fixed 20 ms clock through a small overrun: the next
                # step reads fresh feedback once, then returns to that clock.
                # Rebase only after a whole missed period; never run a backlog.
                self.loop_missed += int((now-deadline)/CONTROL_PERIOD)
                deadline = now
            self.stop_event.wait(max(0.0,deadline-time.monotonic()))

    def stop(self):
        self.stop_event.set()
        with self.lock:
            previous = self.mode
            self.mode = 'off'; self.generation += 1; self.arm_pending = False
            self._keyboard_cancel_locked()
            self.factory_active = False
            self.factory_held = 0
            self.factory_reads = {}
            if previous == 'calibrate':
                self.cal_done = [False]*15
                self.bridge.notify('duck_cal', encode_cal(CAL_EXIT))
            elif previous == 'servo_debug':
                self.bridge.notify('duck_servo', encode_servo(SERVO_EXIT, 10))
            self.seq += 1
            self.bridge.notify('duck_command', encode_command(self.seq, 0, DISARM, DEFAULT_DEG))
        self.current_telemetry.close()

    def status(self):
        snapshot_begin = time.monotonic()
        def hz(times):
            recent = [t for t in times if now-t < 5]
            if len(recent) < 2:
                return 0.0
            span = recent[-1] - recent[0]
            return (len(recent)-1)/span if span > 1e-9 else 0.0
        def summary(samples):
            vals=sorted(samples)
            return {'mean':sum(vals)/len(vals), 'p99':vals[min(len(vals)-1,int(len(vals)*.99))], 'max':vals[-1]} if vals else {}
        with self.lock:
            now = time.monotonic()
            fb = self.feedback
            zero_confirmed = self._zero_sync_confirmed()
            # Copy bounded histories while locked; sorting and rate scans are
            # display work and must not hold up receive or policy50.
            rx_times, infer_times = list(self.rx_times), list(self.infer_times)
            infer_starts = list(self.infer_start_times)
            infer_ms, rx_intervals = list(self.infer_ms), list(self.rx_intervals)
            result = dict(mode=self.mode, model=self.model_message, bridge_baud=2000000,
                received=self.received, inferred=self.inferred, sent=self.sent,
                bad_frames=self.bad_frames, sequence_gaps=self.gaps, out_of_order_frames=self.out_of_order_frames,
                loop_missed=self.loop_missed,
                feedback_age_ms=(now-self.rx_time)*1000 if fb else None,
                uptime_s=now-self.started, last_error=self.last_error, command=self.twist,
                head=self.head, mouth=self.mouth, policy_targets=self.last_action, motors=MOTOR_IDS, boot_pd=self.boot_pd,
                last_policy_step=self.last_policy_step,
                remote_command=dict(age_ms=max(0.0, now-self.web_time)*1000,
                    timed_out=self.remote_command_timed_out,
                    policy_disconnect_action='zero_intent_keep_local_policy'),
                full_robot_ready=bool(zero_confirmed and fb and fb.imu_ok and fb.mask==ALL_MASK and now-self.rx_time<.15),
                raw_pos=list(self.raw_pos), zero_pos=list(self.zero_pos),
                factory_zero_pos=list(FACTORY_ZERO), cal_done=list(self.cal_done),
                cal_targets=list(self.cal_targets), raw_mask=self.raw_mask)
            result['gyro_bias_control'] = self.gyro_bias_control.status()
            result['joint_feedback_age_ms']=[None if t is None else (now-t)*1000 for t in self._joint_rx_times]
            result['stale_joint_ids']=self._stale_joint_ids(now)
            result['control_ready']=bool(zero_confirmed and fb and fb.imu_ok and not result['stale_joint_ids'] and now-self.rx_time<.15 and self.mode not in EXCLUSIVE_MODES and not is_tray_pose(fb))
            result['zero_sync']=dict(confirmed=zero_confirmed, expected_zero_pos=list(self.zero_pos),
                mcu_zero_pos=list(self._mcu_zero_pos) if self._mcu_zero_pos is not None else None,
                pending_transaction=bool(getattr(self, 'factory_pending', None)),
                last_push_age_ms=(now-self._zero_push_at)*1000 if self._zero_push_at is not None else None)
            result['servo_scan_version'] = 1
            result['joint_limits_deg'] = {'32': [-HEAD_YAW_LIMIT_DEG, HEAD_YAW_LIMIT_DEG]}
            result['head_yaw_output'] = self.head_yaw_output
            result['tray_motion_guard'] = dict(detected=is_tray_pose(fb), reason=TRAY_REASON if is_tray_pose(fb) else '')
            result['feedback_timing']=dict(filter_clock='mcu_sample_us',
                freshness_clock='host_monotonic', sample_clock_s=self._feedback_clock_s,
                last_sample_us=self._feedback_clock_us, receiver_epoch=self._feedback_receive_epoch,
                boot_marker='%016x'%fb.boot if fb and fb.boot is not None else None,
                last_imu_us=fb.imu_us if fb else None)
            result['deployment'] = dict(reference='rl/raspi_deploy', control_hz=50,
                models_preloaded=True, command_dispatch='latest_intent_at_control_tick',
                command_receipt='accepted_seq; control_intent.result; last_policy_step.control_intent_seq; mcu_command_seq',
                home_deg=HOME_DEG.tolist(), default_deg=DEFAULT_DEG,
                action_scale=self.policy.action_scale, action_alpha=self.policy.action_alpha,
                action_alpha_head=self.policy.action_alpha_head,
                gyro_alpha=self.filter.gyro_alpha, velocity_alpha=self.filter.vel_alpha,
                gravity_tau_s=GRAVITY_TAU, imu_bias_deg=list(IMU_BIAS_DEG),
                filter_clock='mcu_sample_us', snapshot='latest')
            result['recovery'] = self.recovery.status()
            result['recovery']['active'] = self.mode in ('shadow','policy')
            result.update(self._keyboard_state_locked(now))
            result['pick'] = dict(active=self.pick_active, phi=self.pick_phi,
                                 period_s=PICK_PERIOD_S, mouth_close_phi=PICK_MOUTH_CLOSE_PHI,
                                 model=self.pick_policy.file)
            remain = max(0.0, self.roulade_until - now)
            result['roulade'] = dict(active=remain > 0, remaining_s=remain,
                                    seconds=ROULADE_S, model=self.roulade_policy.file)
            if self.mode in ('shadow','policy'):
                if remain > 0 and self.recovery.phase == 'walk':
                    result['active_model'] = self.roulade_policy.file
                elif (self.pick_active or self.manual_pick_phase>0 or self.manual_pick_target>0) and self.recovery.phase == 'walk':
                    result['active_model'] = self.pick_policy.file
                elif self.recovery.phase == 'walk':
                    result['active_model'] = self.policy.file
                else:
                    result['active_model'] = self.getup_policy.file
            else:
                result['active_model'] = None
            result['sit_down']=self._sit_status_locked(now)
            if result['sit_down']['active']:result['active_model']=self.sit_policy.file
            if fb:
                result.update(servo_ids=[i for j,i in enumerate(MOTOR_IDS) if fb.mask & (1<<j)],
                    imu_ok=fb.imu_ok, enabled=fb.enabled, mcu_watchdog=bool(fb.flags&4),
                    mcu_calibrating=fb.calibrating, mcu_servo_debug=fb.servo_debugging,
                    mcu_seq=fb.seq, mcu_period_us=fb.period_us, mcu_work_us=fb.work_us,
                    mcu_overruns=fb.overruns, mcu_report_drops=fb.report_drops,
                    mcu_invalid_commands=fb.invalid_commands, mcu_command_seq=fb.command_seq,
                    mcu_command_age_us=fb.command_age_us, angles=fb.angles,
                    velocities=fb.velocities, targets=fb.targets, acc=fb.acc, gyro=fb.gyro,
                    gravity=self.filtered['grav'] if self.filtered else None,
                    missing_joint_ids=[i for j,i in enumerate(MOTOR_IDS) if not fb.mask&(1<<j)])
            result.update(self.display_telemetry.snapshot(fb,self.rx_time,now))
            result['factory_calibration']=self.factory_status()
            with self.intent_lock:
                result['control_intent'] = dict(accepted_seq=self.intent_sequence,
                    pending_seq=self.pending_intent[0] if self.pending_intent else None,
                    superseded=self.intent_overwritten, result=dict(self.intent_result) if self.intent_result else None)
        unlocked = time.monotonic()
        starts = [t for t in infer_starts if now-t < 5]
        intervals = [(b-a)*1000 for a,b in zip(starts, starts[1:])]
        result.update(feedback_hz=hz(rx_times), inference_hz=hz(infer_times),
            inference_ms=summary(infer_ms), receive_interval_ms=summary(rx_intervals),
            inference_interval_ms=dict(summary(intervals), count=len(intervals),
                over_30ms=sum(v>30 for v in intervals)))
        result['servo_current']=self.current_telemetry.snapshot(now)
        result['status_compute_ms'] = dict(lock_wait=(now-snapshot_begin)*1000,
            locked=(unlocked-now)*1000, outside_lock=(time.monotonic()-unlocked)*1000)
        return result
