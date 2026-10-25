"""Passive, robot-local motion episodes. Never sends commands to the controller.

25 Hz snapshots use the robot clock, including one second before/after actuation.
Sampling and compressed disk writes run outside the policy/Bridge threads.
The PC archives completed episodes; the robot keeps a 16 MiB rolling cache.
"""
from collections import deque
import copy
import gzip
import hashlib
import json
import math
import marshal
from pathlib import Path
import queue
import re
import threading
import time
import uuid

SCHEMA = 'xgoduck-motion-v1'
SESSION_ID = re.compile(r'^[a-zA-Z0-9_-]{1,100}$')
FIELDS = ('uptime_s', 'mode', 'enabled', 'command', 'head', 'mouth', 'active_model',
          'recovery', 'motion_recovery_latched', 'angles', 'velocities', 'targets',
          'policy_targets', 'last_policy_step', 'remote_command', 'gravity', 'gyro', 'acc', 'imu_ok', 'servo_ids',
          'missing_joint_ids', 'stale_joint_ids', 'joint_feedback_age_ms',
          'full_robot_ready', 'feedback_age_ms', 'feedback_timing', 'mcu_seq',
          'mcu_command_seq', 'mcu_command_age_us', 'mcu_watchdog',
          'mcu_invalid_commands', 'mcu_overruns', 'mcu_report_drops',
          'sent', 'inferred', 'feedback_hz', 'inference_hz', 'inference_interval_ms', 'last_error',
          'gyro_bias_control', 'zero_sync', 'head_yaw_output', 'display_heading',
          'servo_power', 'servo_current', 'factory_calibration', 'raw_pos', 'zero_pos',
          'deployment', 'mcu_period_us', 'mcu_work_us', 'control_event', 'control_intent','sit_down')


def actuating(state):
    return (state.get('enabled') is True or
            state.get('mode') in ('policy', 'hold', 'calibrate', 'servo_debug') or
            bool(state.get('factory_calibration', {}).get('held_mask')) or
            (state.get('control_event', {}).get('name') == 'mode' and
             state['control_event'].get('body', {}).get('mode') in ('policy', 'hold', 'calibrate', 'servo_debug')))


def compact(state):
    return copy.deepcopy({key: state[key] for key in FIELDS if key in state})


def tilt(state):
    g = state.get('gravity')
    if isinstance(g, (list, tuple)) and len(g) == 3 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in g):
        return math.degrees(math.atan2(math.hypot(g[0], g[1]), -g[2]))
    return None


class MotionRecorder:
    def __init__(self, status, directory=None, *, rate=25, cache_bytes=16*1024*1024,
                 start=True, provenance=None):
        self.status = status
        self.directory = Path(directory) if directory is not None else Path(__file__).resolve().parents[1]/'data/motions'
        self.rate, self.cache_bytes = rate, cache_bytes
        self.provenance = provenance if provenance is not None else self._provenance()
        self.queue = queue.Queue(maxsize=512)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.capture_lock = threading.Lock()
        self.latest_lock = threading.Condition()
        self.latest_generation = 0
        self._latest = None
        self._latest_at = None
        self.pre = deque(maxlen=rate)
        self.active = None
        self.stream = None
        self.last_active = self.last_meta = self.previous_uptime = None
        self.previous_event = None
        self.drops = self.errors = 0
        self.last_error = ''
        self.capture_count = self.capture_total_ms = self.capture_max_ms = 0
        self.directory.mkdir(parents=True, exist_ok=True)
        self._recover()
        self.sampler = threading.Thread(target=self._sample, name='motion-sampler', daemon=True)
        self.writer = threading.Thread(target=self._write_loop, name='motion-writer', daemon=True)
        if start:
            self.writer.start()
            self.sampler.start()

    @staticmethod
    def _provenance():
        root = Path(__file__).resolve().parent
        names = ('main.py', 'controller.py', 'policy.py', 'rl_core.py', 'proc.py',
                 'motion_recording.py', 'release_profile.py', 'xgoduck_walk.onnx', 'xgoduck_getup.onnx')
        return {'sha256': {name: hashlib.sha256((root/name).read_bytes()).hexdigest()
                           for name in names if (root/name).is_file()}}

    def _meta(self):
        value = copy.deepcopy(self.active)
        target = self.directory/(value['id']+'.json')
        temporary = target.with_suffix('.json.part')
        temporary.write_text(json.dumps(value, ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
        temporary.replace(target)

    def _recover(self):
        for path in self.directory.glob('*.json'):
            try:
                meta = json.loads(path.read_text('utf-8'))
                if meta.get('state') != 'recording' or not SESSION_ID.fullmatch(meta.get('id', '')):
                    continue
                source = self.directory/(meta['id']+'.jsonl.gz')
                temporary = source.with_suffix('.recover')
                frames, last = 0, None
                with gzip.open(temporary, 'wt', encoding='utf-8') as out:
                    try:
                        with gzip.open(source, 'rt', encoding='utf-8') as data:
                            for line in data:
                                row = json.loads(line)
                                if row.get('type') == 'frame':
                                    frames += 1
                                    last = row
                                if row.get('type') != 'end':
                                    out.write(json.dumps(row, ensure_ascii=False, separators=(',', ':'))+'\n')
                    except (EOFError, OSError, ValueError):
                        pass
                    meta.update(state='interrupted', end_reason='process_interrupted', frames=frames,
                                duration_s=max(0, last['t']) if last else 0)
                    out.write(json.dumps({'type': 'end', 'summary': meta}, ensure_ascii=False)+'\n')
                temporary.replace(source)
                path.write_text(json.dumps(meta, ensure_ascii=False), encoding='utf-8')
            except (OSError, ValueError, KeyError) as error:
                self.last_error = '恢复记录: '+str(error)
                self.errors += 1

    def submit(self, state, wall=None, *, frozen=False):
        try:
            value = {key: state[key] for key in FIELDS if key in state} if frozen else compact(state)
            self.queue.put_nowait((value, time.time() if wall is None else wall))
        except queue.Full:
            self.drops += 1

    def _sample(self):
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            try:
                self.capture()
            except Exception as error:
                self.last_error = '采样: '+str(error)
                self.errors += 1
            deadline += 1/self.rate
            if deadline < time.monotonic():
                deadline = time.monotonic()
            self.stop_event.wait(max(0, deadline-time.monotonic()))

    def capture(self, event=None):
        # Serialize observation + submission so API event snapshots cannot be
        # queued behind a later periodic snapshot and reverse the source clock.
        with self.capture_lock:
            begin = time.perf_counter()
            observed_at = time.monotonic()
            state = self.status()
            if event is not None:
                state['control_event'] = dict(event, id=uuid.uuid4().hex, accepted=True)
            # This snapshot is a tree of Python builtins. Clone it in C rather
            # than spending policy-thread GIL time recursively copying in Python.
            # The temporary bytes stay in-process; on-disk recordings remain JSON.
            state = marshal.loads(marshal.dumps(state))
            with self.latest_lock:
                self._latest = state
                self._latest_at = observed_at
                self.latest_generation += 1
                self.latest_lock.notify_all()
            self.submit(state, frozen=True)
            elapsed = (time.perf_counter()-begin)*1000
            self.capture_count += 1
            self.capture_total_ms += elapsed
            self.capture_max_ms = max(self.capture_max_ms, elapsed)

    def wait_latest(self, after, timeout=.5):
        """Wake on capture, coalescing slow consumers to the newest snapshot."""
        with self.latest_lock:
            self.latest_lock.wait_for(lambda: self.latest_generation > after, timeout)
            return self.latest_generation, self.latest()

    def latest(self):
        """Reuse the one sampler for HTTP streaming instead of polling twice.

        The source uptime/sequence stay unchanged; all freshness ages include
        the time spent in this cache. This cache never grants control permission.
        """
        with self.latest_lock:
            if self._latest is None:
                return None
            # The sampler owns this frozen snapshot. Streaming only copies
            # the containers whose age fields it changes, not all 15 axes.
            value = dict(self._latest)
            elapsed = max(0, time.monotonic()-self._latest_at)*1000
        value['telemetry_cache_age_ms'] = elapsed
        if value.get('feedback_age_ms') is not None:
            value['feedback_age_ms'] += elapsed
            if value['feedback_age_ms'] >= 150:
                value['full_robot_ready'] = value['control_ready'] = False
        if isinstance(value.get('joint_feedback_age_ms'), list):
            value['joint_feedback_age_ms'] = [None if v is None else v+elapsed for v in value['joint_feedback_age_ms']]
            ids = value.get('motors') or [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]
            value['stale_joint_ids'] = [sid for sid, age in zip(ids, value['joint_feedback_age_ms']) if age is None or age >= 150]
            if value['stale_joint_ids']:
                value['full_robot_ready'] = value['control_ready'] = False
        for name, limit in [('display_heading', 150), ('servo_current', 150), ('servo_power', 6000)]:
            item = value.get(name)
            if isinstance(item, dict) and item.get('age_ms') is not None:
                item = value[name] = dict(item)
                item['age_ms'] += elapsed
                if item['age_ms'] >= limit:
                    item['available'] = False
                    if name == 'servo_current':
                        item.update(stale=True, sum_abs_a=None, sum_signed_a=None, per_servo_signed_a=[None]*15)
        return value

    def mark_event(self, name, body):
        try:
            self.capture(dict(name=name, body=copy.deepcopy(body)))
        except Exception as error:
            self.last_error = '操作记录: '+str(error)
            self.errors += 1

    def _row(self, state, wall):
        t = state['uptime_s']-self.active['start_uptime_s']
        row = dict(type='frame', t=t, recorded_unix_s=wall, status=state)
        self.stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n')
        meta = self.active
        meta['frames'] += 1
        meta['duration_s'] = max(0, t)
        meta['last_recorded_unix_s'] = wall
        if self.previous_uptime is not None:
            gap = state['uptime_s']-self.previous_uptime
            meta['max_sample_gap_s'] = max(meta['max_sample_gap_s'], gap)
            if gap > 2/self.rate:
                meta['sample_gaps'] += 1
        self.previous_uptime = state['uptime_s']
        angle = tilt(state)
        event = (state.get('mode'), state.get('enabled'), state.get('recovery', {}).get('phase'),
                 state.get('command'), state.get('last_error'))
        if event != self.previous_event:
            meta['events'].append(dict(t=t, mode=event[0], enabled=event[1], phase=event[2],
                                       command=event[3], error=event[4]))
            self.previous_event = copy.deepcopy(event)
        if state.get('control_event'):
            meta['events'].append(dict(t=t, control_event=state['control_event']))
        if angle is not None and state.get('enabled'):
            meta['max_enabled_tilt_deg'] = max(meta['max_enabled_tilt_deg'] or 0, angle)
            if angle < 20 and meta['upright_at_s'] is None:
                meta['upright_at_s'] = t
            if angle > 45 and meta['upright_at_s'] is not None and meta['tilt_after_upright_at_s'] is None:
                meta['tilt_after_upright_at_s'] = t

    def consume(self, state, wall):
        now = state.get('uptime_s')
        if not isinstance(now, (int, float)) or not math.isfinite(now):
            return
        moving = actuating(state)
        if self.active is None and moving:
            sid = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(wall))+'-'+uuid.uuid4().hex[:8]
            self.active = dict(schema=SCHEMA, id=sid, state='recording', source='robot-status',
                label='校准 / 维护' if state.get('mode') in ('calibrate', 'servo_debug', 'hold') else '站立 / 遥控',
                start_unix_s=wall, start_uptime_s=now, frames=0, duration_s=0, sample_hz=self.rate,
                clock='robot_uptime_s', pre_roll_s=1, post_roll_s=1, events=[], sample_gaps=0,
                max_sample_gap_s=0, max_enabled_tilt_deg=None, upright_at_s=None,
                tilt_after_upright_at_s=None, provenance=self.provenance,
                zero_pos=state.get('zero_sync', {}).get('expected_zero_pos'),
                queue_drops_at_start=self.drops)
            self.stream = gzip.open(self.directory/(sid+'.jsonl.gz'), 'wt', encoding='utf-8', compresslevel=1)
            self.stream.write(json.dumps(dict(type='header', **self.active), ensure_ascii=False)+'\n')
            self.previous_uptime = self.previous_event = None
            for before, before_wall in self.pre:
                self._row(before, before_wall)
            self.pre.clear()
            self.last_active = now
            self.last_meta = now
            self._meta()
        if self.active is not None:
            self._row(state, wall)
            if moving:
                self.last_active = now
            if not moving and now-self.last_active >= 1:
                self._finish('disabled')
            elif now-self.active['start_uptime_s'] >= 300:
                self._finish('five_minute_segment')
            elif now-self.last_meta >= 1:
                self.stream.flush()
                self._meta()
                self.last_meta = now
        else:
            self.pre.append((state, wall))

    def _finish(self, reason):
        if self.active is None:
            return
        self.active.update(state='complete' if reason in ('disabled', 'five_minute_segment') else 'interrupted',
                           end_reason=reason, queue_drops=self.drops-self.active['queue_drops_at_start'])
        self.stream.write(json.dumps(dict(type='end', summary=self.active), ensure_ascii=False)+'\n')
        self.stream.close()
        self.stream = None
        self._meta()
        self.active = None
        self._prune()

    def _prune(self):
        paths = sorted(self.directory.glob('*.jsonl.gz'), key=lambda p: p.stat().st_mtime, reverse=True)
        size = 0
        for path in paths:
            size += path.stat().st_size
            if size > self.cache_bytes:
                path.unlink()
                path.with_suffix('').with_suffix('.json').unlink(missing_ok=True)

    def _write_loop(self):
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                state, wall = self.queue.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                with self.lock:
                    self.consume(state, wall)
            except Exception as error:
                self.last_error = '保存: '+str(error)
                self.errors += 1
                if self.stream is not None:
                    try:
                        self.stream.close()
                    except OSError:
                        pass
                    self.stream = None
                self.active = None
        with self.lock:
            self._finish('service_stopped')

    def snapshot(self):
        with self.lock:
            return dict(automatic=True, sample_hz=self.rate, queue_drops=self.drops,
                        errors=self.errors, error=self.last_error, active=copy.deepcopy(self.active),
                        sampler_alive=self.sampler.is_alive(), writer_alive=self.writer.is_alive(),
                        capture_count=self.capture_count,
                        mean_capture_ms=self.capture_total_ms/self.capture_count if self.capture_count else None,
                        max_capture_ms=self.capture_max_ms,
                        robot_cache_bytes=self.cache_bytes)

    def sessions(self):
        with self.lock:
            items = []
            for path in self.directory.glob('*.json'):
                try:
                    items.append(json.loads(path.read_text('utf-8')))
                except (OSError, ValueError):
                    continue
            if self.active:
                items = [row for row in items if row['id'] != self.active['id']]+[copy.deepcopy(self.active)]
            return sorted(items, key=lambda row: row.get('start_unix_s', 0), reverse=True)

    def session_path(self, sid):
        if not SESSION_ID.fullmatch(sid):
            raise ValueError('Invalid session id')
        with self.lock:
            if self.active and self.active['id'] == sid:
                raise ValueError('动作仍在记录，结束后可回放')
            path = self.directory/(sid+'.jsonl.gz')
            if not path.is_file():
                raise FileNotFoundError(sid)
            return path

    def close(self):
        self.stop_event.set()
        if self.sampler.is_alive():
            self.sampler.join(2)
        if self.writer.is_alive():
            self.writer.join(3)
