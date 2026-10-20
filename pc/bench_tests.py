"""PC-only observation of the official controller. Never sends robot commands.

Starting/stopping an observation does not change mode, keep a heartbeat alive,
or remove torque. A visible control page remains responsible for control.
"""
import copy
import json
import math
from pathlib import Path
import threading
import time
import uuid

PROFILES = ('default', 'policy', 'getup', 'pick', 'roulade', 'walk', 'disturbance')
SAMPLE_PERIOD_S = .1
MAX_OBSERVATION_GAP_S = .3


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _zero(values, length):
    return (isinstance(values, (list, tuple)) and len(values) == length
            and all(_number(v) and abs(v) <= .001 for v in values))


def _vector(values, length):
    return (isinstance(values, (list, tuple)) and len(values) == length
            and all(_number(v) for v in values))


class _Run:
    """Deterministic state machine; monotonic timestamps are supplied by caller."""
    def __init__(self, profile, duration_s, now):
        self.run_id = time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:10]
        self.profile, self.duration_s, self.started = profile, duration_s, now
        self.active = True
        self.elapsed_s = self.longest_stable_s = 0.0
        self.digital_status, self.reason = 'waiting', '等待用户通过控制台启动对应模式；观察器不会启动动作'
        self.physical_acceptance, self.physical_note = 'pending', ''
        self.rows = []
        self.stable_since = self.finish_since = self.previous_time = None
        self.previous_identity = self.previous_uptime = self.zero_pos = None
        self.entered_mode = self.trigger_seen = False
        self.metrics = dict(samples=0, fresh_samples=0, integrity_breaks=0,
                            observation_gaps=0, duplicate_samples=0,
                            max_observation_gap_s=0.0, tilt_max_deg=None,
                            joint_error_max_deg=None, trigger_seen=False,
                            completion_seen=False, failures={},
                            sample_period_s=SAMPLE_PERIOD_S,
                            max_allowed_observation_gap_s=MAX_OBSERVATION_GAP_S,
                            physical_scope='站立、翻滚、行走、抗扰和物体叼取效果需独立实物确认')

    def finish(self, status, reason):
        self.active, self.digital_status, self.reason = False, status, reason

    def snapshot(self):
        return copy.deepcopy({k: getattr(self, k) for k in (
            'run_id', 'profile', 'active', 'elapsed_s', 'longest_stable_s',
            'duration_s', 'digital_status', 'physical_acceptance',
            'physical_note', 'reason', 'metrics')})

    def _break(self, reason):
        self.stable_since = self.finish_since = None
        self.metrics['integrity_breaks'] += 1
        failures = self.metrics['failures']
        failures[reason] = failures.get(reason, 0) + 1
        self.reason = reason
        self.digital_status = 'observing'

    def consume(self, state, now):
        if not self.active:
            return
        self.elapsed_s = max(0.0, now - self.started)
        self.metrics['samples'] += 1
        self.rows.append({'elapsed_s': self.elapsed_s, 'status': copy.deepcopy(state)})
        gap_bad = False
        if self.previous_time is not None:
            gap = now - self.previous_time
            self.metrics['max_observation_gap_s'] = max(self.metrics['max_observation_gap_s'], gap)
            if gap > MAX_OBSERVATION_GAP_S + 1e-9:
                self.metrics['observation_gaps'] += 1
                gap_bad = True
        self.previous_time = now
        if not isinstance(state, dict):
            self._break('状态读取失败')
            return self._timeout()
        required = ('mode', 'enabled', 'full_robot_ready', 'imu_ok', 'feedback_age_ms',
                    'uptime_s', 'zero_pos', 'command', 'head', 'recovery',
                    'mcu_seq', 'mcu_command_age_us', 'last_error',
                    'mcu_invalid_commands', 'mcu_watchdog')
        if any(k not in state for k in required):
            self._break('状态字段缺失')
            return self._timeout()
        uptime, zeros = state['uptime_s'], state['zero_pos']
        if (not _number(uptime) or not isinstance(zeros, list) or len(zeros) != 15
                or not all(_number(x) for x in zeros)):
            self._break('启动时间或零位无效')
            return self._timeout()
        if self.previous_uptime is not None and uptime < self.previous_uptime:
            return self.finish('invalid', '主控启动时间回退，可能重启；本轮不可合并')
        self.previous_uptime = uptime
        if self.zero_pos is None:
            self.zero_pos = list(zeros)
        elif zeros != self.zero_pos:
            return self.finish('invalid', '机械零位变化；本轮不可合并')
        identity = state['mcu_seq']
        if not isinstance(identity, int) or isinstance(identity, bool) or not 0 <= identity <= 0xffffffff:
            self._break('反馈样本编号无效')
            return self._timeout()
        duplicate = identity == self.previous_identity
        if (self.previous_identity is not None and identity < self.previous_identity
                and not (self.previous_identity > 0xffff0000 and identity < 0x10000)):
            return self.finish('invalid', 'MCU反馈序号回退，可能重启；本轮不可合并')
        self.previous_identity = identity
        if duplicate:
            self.metrics['duplicate_samples'] += 1
            self._break('重复反馈样本')
            return self._timeout()
        age, command_age = state['feedback_age_ms'], state['mcu_command_age_us']
        if (not _number(age) or not 0 <= age < 150 or not _number(command_age)
                or not 0 <= command_age < 150000 or state['full_robot_ready'] is not True
                or state['imu_ok'] is not True or state['mcu_watchdog'] is not False):
            self._break('反馈或MCU回执不新鲜/不完整')
            return self._timeout()
        if state['last_error']:
            return self.finish('failed', '官方控制错误: ' + str(state['last_error']))
        if not _number(state['mcu_invalid_commands']) or state['mcu_invalid_commands'] < 0:
            self._break('MCU拒绝计数无效')
            return self._timeout()
        old_invalid = self.metrics.setdefault('initial_invalid_commands', state['mcu_invalid_commands'])
        if state['mcu_invalid_commands'] > old_invalid:
            return self.finish('failed', '本轮出现MCU指令拒绝')
        self.metrics['fresh_samples'] += 1
        recovery = state['recovery']
        if not isinstance(recovery, dict):
            self._break('恢复状态缺失')
            return self._timeout()
        # Recovery angle is not updated in hold. Use current filtered gravity.
        gravity = state.get('gravity')
        tilt = None
        if isinstance(gravity, list) and len(gravity) == 3 and all(_number(x) for x in gravity):
            norm = math.sqrt(sum(x*x for x in gravity))
            if norm > 1e-6:
                tilt = math.degrees(math.acos(max(-1, min(1, -gravity[2]/norm))))
        if tilt is None:
            self._break('当前重力姿态缺失或无效')
            return self._timeout()
        self.metrics['tilt_max_deg'] = max(self.metrics['tilt_max_deg'] or 0, tilt)
        angles, targets = state.get('angles'), state.get('targets')
        if (isinstance(angles, list) and isinstance(targets, list) and len(angles) == len(targets) == 15
                and all(_number(v) for v in angles + targets)):
            self.metrics['joint_error_max_deg'] = max(self.metrics['joint_error_max_deg'] or 0,
                                                       max(abs(a-b) for a,b in zip(angles, targets)))
        expected = 'hold' if self.profile == 'default' else 'policy'
        correct_mode = state['mode'] == expected and state['enabled'] is True
        if not correct_mode:
            self.stable_since = self.finish_since = None
            if self.entered_mode:
                return self.finish('incomplete', '退出测试模式或已卸力；未完成窗口不计成功')
            self.reason = '等待用户启动 ' + expected + ' 模式'
            return self._timeout()
        self.entered_mode = True
        self.digital_status = 'observing'
        if gap_bad:
            self._break('观测间隔超过0.3秒，连续窗口重计')
            return self._timeout()
        phase = recovery.get('phase')
        if not _vector(state['command'], 3) or not _vector(state['head'], 4):
            self._break('运动或头部命令字段无效')
            return self._timeout()
        quiet = _zero(state['command'], 3) and _zero(state['head'], 4)
        if 'phase' not in recovery or not isinstance(state.get('pick'), dict) or not isinstance(state.get('roulade'), dict):
            self._break('动作阶段字段缺失')
            return self._timeout()
        if any('active' not in state[k] for k in ('pick', 'roulade')):
            self._break('动作活动字段缺失')
            return self._timeout()
        inactive = state['pick']['active'] is False and state['roulade']['active'] is False
        stable = tilt < 15 and quiet and inactive and (self.profile == 'default' or phase == 'walk')
        if stable:
            if self.stable_since is None:
                self.stable_since = now
            self.longest_stable_s = max(self.longest_stable_s, now - self.stable_since)
        else:
            self.stable_since = None
        if self.profile in ('default', 'policy'):
            self.reason = '记录连续零命令站立；完成后保持当前控制状态'
            if self.stable_since is not None and now - self.stable_since >= self.duration_s:
                return self.finish('passed', '完整连续站立观察窗口达标；实物验收待确认，当前模式保持')
        else:
            triggered = (phase == 'getup' if self.profile == 'getup' else
                         state[self.profile]['active'] is True if self.profile in ('pick', 'roulade') else
                         not _zero(state['command'], 3) if self.profile == 'walk' else False)
            self.trigger_seen |= triggered
            self.metrics['trigger_seen'] = self.trigger_seen
            self.reason = '只读记录动作触发、结束及回稳；实物效果独立确认'
            if self.profile == 'disturbance':
                self.reason = '记录抗扰观察；外力、滑移、持续后退无法仅凭状态确认'
                if self.elapsed_s >= self.duration_s:
                    return self.finish('observed', '抗扰观察时间结束；未自动判断施力或抗扰通过')
            elif self.trigger_seen and stable and not triggered:
                if self.finish_since is None:
                    self.finish_since = now
                if now - self.finish_since >= self.duration_s:
                    self.metrics['completion_seen'] = True
                    return self.finish('observed', f'观察到动作触发并回到零命令站立{self.duration_s:g}秒；实物动作效果待确认')
            else:
                self.finish_since = None
        self._timeout()

    def _timeout(self):
        limit = max(30.0, self.duration_s * 3) if self.profile in ('default', 'policy') else max(30.0, self.duration_s)
        if self.elapsed_s >= limit:
            self.finish('incomplete', '观察超时，未取得完整要求窗口或动作周期')


class BenchTests:
    def __init__(self, root, get_status):
        self.root = Path(root).resolve() / 'bench-results'
        self.get_status = get_status
        self.lock = threading.RLock()
        self.run = self.thread = None
        self.cancel = threading.Event()

    def start(self, profile, duration_s=15):
        if profile not in PROFILES:
            raise ValueError('Unknown profile')
        if not _number(duration_s) or not 1 <= duration_s <= 300:
            raise ValueError('duration_s must be finite and between 1 and 300')
        # Standing acceptance always requires at least the complete 15 second window.
        if profile in ('default', 'policy'):
            duration_s = max(15.0, duration_s)
        with self.lock:
            if self.run is not None and self.run.active:
                raise ValueError('An observation is already active')
            self.cancel = threading.Event()
            self.run = _Run(profile, float(duration_s), time.monotonic())
            current = self.run
            (self.root / current.run_id).mkdir(parents=True, exist_ok=False)
            self._save(current)
            self.thread = threading.Thread(target=self._observe, args=(current, self.cancel),
                                           name='duck-bench-observer', daemon=True)
            self.thread.start()
            return current.snapshot()

    def _observe(self, run, cancel):
        try:
            while not cancel.is_set():
                begin = time.monotonic()
                try:
                    status = self.get_status()
                except Exception:
                    status = None
                with self.lock:
                    if not run.active:
                        break
                    run.consume(status, time.monotonic())
                    # Persist every received snapshot; retain evidence if service exits.
                    with (self.root / run.run_id / 'trace.jsonl').open('a', encoding='utf-8') as stream:
                        stream.write(json.dumps(run.rows[-1], ensure_ascii=False, allow_nan=False) + '\n')
                    if not run.active:
                        break
                cancel.wait(max(0, SAMPLE_PERIOD_S - (time.monotonic() - begin)))
        except Exception as exc:
            with self.lock:
                run.finish('invalid', '观察记录异常: ' + type(exc).__name__)
        finally:
            with self.lock:
                self._save(run)

    def _save(self, run):
        folder = self.root / run.run_id
        result = run.snapshot()
        result['hardware_writes'] = 0
        result['automatic_stop_or_heartbeat'] = False
        result['baseline_zero_pos'] = run.zero_pos
        intervals=[(b['elapsed_s']-a['elapsed_s'])*1000 for a,b in zip(run.rows,run.rows[1:])]
        def distribution(values):
            values=sorted(v for v in values if _number(v))
            return {'min':values[0], 'p95':values[min(len(values)-1,int(.95*len(values)))], 'max':values[-1]} if values else None
        tilts=[]
        for row in run.rows:
            gravity=(row.get('status') or {}).get('gravity')
            if _vector(gravity,3):
                norm=math.sqrt(sum(v*v for v in gravity))
                if norm>1e-6:tilts.append(math.degrees(math.acos(max(-1,min(1,-gravity[2]/norm)))))
        result['metrics']['observed_intervals_ms']=distribution(intervals)
        result['metrics']['tilt_deg']=distribution(tilts)
        result['metrics']['observed_hz']=(len(run.rows)-1)/(run.rows[-1]['elapsed_s']-run.rows[0]['elapsed_s']) if len(run.rows)>1 and run.rows[-1]['elapsed_s']>run.rows[0]['elapsed_s'] else None
        for name, data in (('result.json', result), ('trace.json', {'result': result, 'rows': run.rows})):
            tmp = folder / (name + '.tmp')
            tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
            tmp.replace(folder / name)

    def snapshot(self):
        with self.lock:
            if self.run is None:
                return {'active': False, 'digital_status': 'idle', 'physical_acceptance': 'pending',
                        'run_id': None, 'profile': None, 'elapsed_s': 0, 'longest_stable_s': 0,
                        'duration_s': 15, 'reason': '尚未开始只读观察', 'metrics': {}}
            return self.run.snapshot()

    def stop(self):
        with self.lock:
            self.cancel.set()
            if self.run and self.run.active:
                self.run.finish('incomplete', '用户停止观察；未向机器人发送停止或卸力命令')
                self._save(self.run)
            return self.snapshot()

    def annotate(self, physical, note=''):
        if physical not in ('pending', 'pass', 'fail'):
            raise ValueError('physical must be pending, pass or fail')
        if not isinstance(note, str) or len(note) > 4000:
            raise ValueError('note must be text up to 4000 characters')
        with self.lock:
            if self.run is None:
                raise ValueError('No observation to annotate')
            self.run.physical_acceptance, self.run.physical_note = physical, note
            self._save(self.run)
            return self.run.snapshot()
