"""USB/B commissioning: no automatic connection, numbering, centring or motion.

This service talks to the USB servo adapter, not the Uno Q runtime. All servo
motion requires a short, explicit authorization for exactly one measured joint.
"""
import json
import secrets
import sys
import threading
import time
import mimetypes
from collections import deque
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import unquote, urlsplit

import serial
from serial.tools import list_ports
from provisioning import Provisioner
from release_policy import program_release_info, legacy_update_blocked

ROOT = Path(__file__).parent
TOKEN = secrets.token_hex(24)
ORDER = [*range(10, 15), *range(20, 25), *range(30, 35)]
MAX_STEP_TICKS = 22  # 1.93 degrees for 4096 counts/revolution.
MAX_OFFSET_TICKS = 114  # About 10 degrees from this session's measured anchor.
ARM_SECONDS = 30.0
WATCHDOG_SECONDS = 1.5
MOVE_SECONDS = .8
MAX_TEMPERATURE_C = 55
NUMBERING_HEARTBEAT_SECONDS = 2.0
NUMBERING_MAX_SECONDS = 45.0
NUMBERING_CENTER_SECONDS = 8.0
DISABLED_OPS = {'auto', 'auto-start', 'prepare', 'prepare-servo', 'center', 'center-all', 'id'}
ACTION_DIAGNOSTICS = deque(maxlen=80)
ACTION_DIAGNOSTICS_LOCK = threading.Lock()


def record_wireless_outcome(action, started, result=None, error=None, status=None):
    """Retain bounded outcomes only, never session tokens or action payloads."""
    import re
    payload = result if isinstance(result, dict) else {}
    state = payload.get('status') if isinstance(payload.get('status'), dict) else status
    state = state if isinstance(state, dict) else {}
    home = state.get('home') if isinstance(state.get('home'), dict) else {}
    progress = home.get('progress') if isinstance(home.get('progress'), dict) else {}
    sequential = state.get('sequence_calibration') if isinstance(state.get('sequence_calibration'), dict) else {}
    match = re.search(r'错误\s*(\d+)', str(error)) if error else None
    event = {'at_unix_s': time.time(), 'duration_ms': round((time.monotonic()-started)*1000, 1),
             'op': str(action.get('op', 'unknown'))[:40], 'http_status': 400 if error else 200,
             'ok': error is None and payload.get('ok') is True,
             'reason': getattr(error, 'reason', None) if error else None,
             'reported_fault': int(match.group(1)) if match else None,
             'boot_id': state.get('boot_id'), 'last_received_req_seq': state.get('last_received_req_seq'),
             'home_phase': home.get('phase'), 'home_error': home.get('error'),
             'home_active_id': progress.get('active_id'), 'home_stage': progress.get('stage_name'),
             'home_arrived_mask': progress.get('arrived_mask'), 'home_enabled_mask': progress.get('enabled_mask'),
             'home_arrival_verified': home.get('arrival_verified'), 'home_lease_stage': home.get('lease_stage'),
             'release_verified': home.get('release_verified'), 'all_torque_off_verified': state.get('all_torque_off_verified'),
             'sequence_phase': sequential.get('phase'), 'sequence_error': sequential.get('error'),
             'sequence_selected_id': sequential.get('selected_id'), 'sequence_held_ids': sequential.get('held_ids'),
             'sequence_completed_ids': sequential.get('completed_ids')}
    with ACTION_DIAGNOSTICS_LOCK:
        ACTION_DIAGNOSTICS.append(event)


def perform_wireless_action(action):
    from wireless_calibration_proxy import proxy
    started = time.monotonic()
    try:
        with bus.lock:
            emergency = action.get('op') in ('cancel', 'heartbeat', 'home_heartbeat', 'home_stop', 'sequence_heartbeat', 'sequence_stop')
            if not emergency and (bus.ser or bus.session or bus.enabled or (bus.numbering and bus.numbering['active'])):
                raise ValueError('先关闭备用 USB 校准会话')
            if not emergency and provisioner.status()['active']:
                raise ValueError('无线更新期间暂不操作舵机')
            proxy.reserve(action)
        try:
            result = proxy.action(action)
        finally:
            proxy.unreserve()
    except Exception as error:
        record_wireless_outcome(action, started, error=error, status=proxy.last_status)
        raise
    record_wireless_outcome(action, started, result=result)
    return result


class NoServoReply(ValueError):
    """A timed-out address, distinct from a servo that replied with an error."""


def packet(sid, instruction, data=b''):
    body = bytes([sid, len(data) + 2, instruction]) + data
    return b'\xff\xff' + body + bytes([(~sum(body)) & 255])


class Bus:
    def __init__(self, root=ROOT, clock=time.monotonic, sleeper=time.sleep):
        self.root = Path(root)
        self.clock, self.sleep = clock, sleeper
        self.ser = None
        self.lock = threading.RLock()
        self.found, self.session = [], None
        self.enabled = set()
        self.write_deadline, self.last_error = 0.0, ''
        self.captures, self.zeros = {}, {}
        self.numbering = None
        self._load_zeros()

    def _load_zeros(self):
        try:
            records = json.loads((self.root / 'commissioning-zero-captures.json').read_text('utf-8'))['records']
            if not isinstance(records, dict):
                return
            for key, record in records.items():
                sid = int(key)
                if sid in ORDER and isinstance(record, dict) and type(record.get('position')) is int and 0 <= record['position'] <= 4095:
                    self.zeros[sid] = record
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass

    @staticmethod
    def _atomic_json(path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(path)

    def close(self):
        errors = []
        for sid in list(self.enabled):
            try:
                self._disable(sid)
            except Exception as error:
                errors.append(f'ID {sid}: {error}')
        if self.ser:
            self.ser.close()
        self.ser, self.session, self.found, self.captures = None, None, [], {}
        self.enabled.clear()
        if errors:
            self.last_error = '断开前卸力未确认，请切断舵机电源。' + '; '.join(errors)

    def request(self, sid, instruction, data=b'', expected=None):
        if not self.ser:
            raise ValueError('请先手动连接 USB/B 转接板串口')
        self.ser.reset_input_buffer()
        self.ser.write(packet(sid, instruction, data))
        self.ser.flush()
        buf, end = bytearray(), self.clock() + .035
        while self.clock() < end:
            buf.extend(self.ser.read(self.ser.in_waiting or 1))
            while len(buf) >= 6:
                start = buf.find(b'\xff\xff')
                if start < 0:
                    buf.clear()
                    break
                if start:
                    del buf[:start]
                if len(buf) < 4:
                    break
                length = buf[3] + 4
                if length < 6 or length > 100:
                    del buf[0]
                    continue
                if len(buf) < length:
                    break
                frame = bytes(buf[:length])
                del buf[:length]
                if frame[2] != sid or sum(frame[2:]) & 255 != 255:
                    continue
                # A TX echo is not a position/ID read reply.
                if expected is not None and len(frame[5:-1]) != expected:
                    continue
                if frame[4]:
                    raise ValueError(f'舵机错误码 {frame[4]}')
                return frame[5:-1]
        raise NoServoReply(f'ID {sid} 没有有效回复（检查供电、B 跳线、线序和波特率）')

    def read(self, sid, address, count):
        return self.request(sid, 2, bytes([address, count]), count)

    def write(self, sid, address, data):
        if not self.ser:
            raise ValueError('串口已断开')
        factory_authorized = sid == 1 and self.numbering and 1 in self.numbering['owned_ids']
        if sid not in ORDER and not factory_authorized:
            raise ValueError('只能写入选定鸭子舵机 ID，禁止广播')
        self.ser.reset_input_buffer()
        self.ser.write(packet(sid, 3, bytes([address]) + bytes(data)))
        self.ser.flush()
        self.sleep(.025)

    def single(self, sid):
        if self.read(sid, 5, 1) != bytes([sid]):
            raise ValueError('目标 ID 未读回，停止操作')

    def status(self, sid):
        self.single(sid)
        data = self.read(sid, 56, 8)
        position = int.from_bytes(data[:2], 'little')
        if not 0 <= position <= 4095:
            raise ValueError('位置超出单圈编码范围，先检查舵机模式')
        return {'id': sid, 'position': position, 'voltage': data[6] / 10,
                'temperature': data[7], 'torque_enabled': bool(self.read(sid, 40, 1)[0]),
                'sampled_at': time.strftime('%Y-%m-%dT%H:%M:%S%z')}

    def scan(self, full=False):
        if self.session:
            raise ValueError('先取消单关节授权，再搜索总线')
        self.found, feedback, errors = [], [], {}
        for sid in (range(1, 254) if full else ORDER):
            try:
                state = self.status(sid)
                self.found.append(sid)
                feedback.append(state)
            except ValueError as error:
                errors[str(sid)] = str(error)
        return {'ids': self.found, 'feedback': feedback, 'errors': errors,
                'missing': [sid for sid in ORDER if sid not in self.found],
                'unexpected': [sid for sid in self.found if sid not in ORDER], 'read_only': True,
                'message': '只读搜索完成，没有改 ID、归中或使能出力；同 ID 冲突不能仅靠搜索排除'}

    def auto_state(self):
        return {'active': False, 'bootstrapped': True, 'used': self.found,
                'prepared': [], 'assigned': [], 'next': None,
                'message': '整机调试模式：自动编号和 2047 归中已禁用'}

    def safety_state(self):
        active = self.session is not None and self.clock() < self.session['expires']
        return {'connected': bool(self.ser), 'port': self.ser.port if self.ser else None,
                'armed': active, 'armed_id': self.session['id'] if active else None,
                'arm_remaining_s': max(0, round(self.session['expires'] - self.clock(), 1)) if active else 0,
                'enabled_ids': sorted(self.enabled), 'max_step_ticks': MAX_STEP_TICKS,
                'max_offset_ticks': MAX_OFFSET_TICKS, 'last_error': self.last_error,
                'mode': 'assembled-single-joint', 'startup_read_only': True}

    def _check_session(self, action, sid):
        if not self.session or not secrets.compare_digest(str(action.get('session', '')), self.session['token']):
            raise ValueError('先明确授权本次单关节会话')
        if sid != self.session['id']:
            raise ValueError('授权只对所选 ID 有效，换关节后重新授权')
        if self.clock() >= self.session['expires']:
            self.session = None
            raise ValueError('30 秒单关节授权已过期，重新读取并授权')
        return self.session

    @staticmethod
    def _check_feedback(state):
        if not 4 <= state['voltage'] <= 8.4:
            raise ValueError(f'电压 {state["voltage"]} V 超出已记录舵机范围，停止写入')
        if state['temperature'] >= MAX_TEMPERATURE_C:
            raise ValueError(f'温度 {state["temperature"]} ℃ 达到保守调试停止值，先卸力冷却')

    def _disable(self, sid):
        # Emergency release must remain possible if position feedback is lost.
        self.write(sid, 40, [0])
        if self.read(sid, 40, 1) != b'\x00':
            raise ValueError(f'ID {sid} 卸力未读回，请立即切断舵机电源')
        self.enabled.discard(sid)

    def _motion_write(self, authorization, sid, address, data):
        if self.session is not authorization or sid != authorization['id'] or self.clock() >= authorization['expires']:
            raise ValueError('写入前授权已失效，停止并请求卸力')
        self.write(sid, address, data)

    def watchdog_tick(self):
        with self.lock:
            if self.numbering and self.numbering['active']:
                job = self.numbering
                if self.clock() >= job['heartbeat_deadline'] or self.clock() >= job['run_deadline']:
                    job['cancel_reason'] = '编号页心跳超过 2 秒或单次任务超过 45 秒，已停止并请求卸力'
                    job['cancel'].set()
                    job['message'] = '编号页心跳或总时限已超时，正在停止并卸力'
                    self._numbering_release(job)
                return
            if self.session and self.clock() >= self.session['expires']:
                self.session = None
            if self.enabled and (not self.session or self.clock() >= self.write_deadline):
                for sid in list(self.enabled):
                    try:
                        self._disable(sid)
                    except Exception as error:
                        self.last_error = '看门狗卸力未确认，请切断舵机电源。' + str(error)
                self.session = None

    def watch(self):
        while True:
            self.sleep(.1)
            self.watchdog_tick()

    def _jog(self, action, sid):
        authorization = self._check_session(action, sid)
        state = self.status(sid)
        self._check_feedback(state)
        if state['torque_enabled']:
            raise ValueError('当前舵机仍在出力，先支撑身体并点击此关节卸力')
        if action['op'] == 'jog':
            delta = action.get('delta_ticks')
            if type(delta) is not int or not 1 <= abs(delta) <= MAX_STEP_TICKS:
                raise ValueError(f'每步必须为 ±1–{MAX_STEP_TICKS} 编码刻度')
            target = state['position'] + delta
        else:
            target = action.get('position')
            if type(target) is not int or not 1 <= abs(target - state['position']) <= MAX_STEP_TICKS:
                raise ValueError(f'目标必须距当前实测位置 1–{MAX_STEP_TICKS} 刻度')
        if not 0 <= target <= 4095:
            raise ValueError('目标越过编码边界，禁止跨圈移动')
        anchor = authorization['anchor']
        if abs(state['position'] - anchor) > MAX_OFFSET_TICKS or abs(target - anchor) > MAX_OFFSET_TICKS:
            raise ValueError(f'当前或目标超出本次实测起点 ±{MAX_OFFSET_TICKS} 刻度，停止并检查装配')
        self.enabled.add(sid)
        self.write_deadline = min(self.clock() + WATCHDOG_SECONDS, authorization['expires'])
        try:
            # A freshly measured goal must precede torque enable, never an old goal.
            self.write(sid, 40, [0])
            self._motion_write(authorization, sid, 42, list(target.to_bytes(2, 'little')) + [0, 0, 100, 0])
            self._motion_write(authorization, sid, 40, [1])
            deadline, reached = min(self.clock() + MOVE_SECONDS, self.write_deadline), False
            while self.clock() < deadline:
                if self.clock() >= authorization['expires']:
                    raise ValueError('运动期间授权过期，已请求卸力')
                self.sleep(.04)
                measured = self.status(sid)
                self._check_feedback(measured)
                if abs(measured['position'] - state['position']) > MAX_STEP_TICKS + 8:
                    raise ValueError('实测移动超过单步容差，已请求卸力，请检查舵机及机械连接')
                if abs(measured['position'] - target) <= 4:
                    reached = True
                    break
            if not reached:
                raise ValueError('0.8 秒内目标未达到，已请求卸力，请检查阻挡、供电或负载')
        except Exception:
            self.session = None
            raise
        finally:
            try:
                self._disable(sid)
            except Exception:
                self.session = None
                raise
        return {**self.status(sid), 'target': target, 'start_position': state['position'],
                'torque_off': True, 'session_anchor': anchor,
                'message': f'仅 ID {sid} 小步移动后已读回卸力，请目视核对关节和方向'}

    def calibration_state(self):
        return {'ids': ORDER, 'records': {str(sid): self.zeros[sid] for sid in ORDER if sid in self.zeros},
                'captured_ids': sorted(self.zeros), 'missing_ids': [sid for sid in ORDER if sid not in self.zeros],
                'complete': all(sid in self.zeros for sid in ORDER),
                'message': '人工对准模型机械零位后保存实测编码值，不驱动到 2047，不写舵机 EEPROM'}

    def export_zeros(self):
        if not all(sid in self.zeros for sid in ORDER):
            raise ValueError('15 颗机械零位尚未全部实测保存，禁止用默认值补齐导出')
        return {'version': 1, 'ids': list(ORDER), 'zero_pos': [self.zeros[sid]['position'] for sid in ORDER],
                'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                'source': 'manually-aligned-measured-mechanical-zero',
                'physical_pose_verification': 'user confirmation required; numeric capture alone is not acceptance'}

    def numbering_state(self):
        job = self.numbering
        if not job:
            return {'active': False, 'phase': 'idle', 'completed': False, 'next_id': ORDER[0],
                    'message': '每次新增一颗出厂 ID 1 裸舵机；已编号舵机可保留连接'}
        occupied = job.get('occupied_ids', [])
        next_id = next((sid for sid in ORDER if sid not in occupied), None) if job.get('retain_numbered') else (
            ORDER[ORDER.index(job['target_id']) + 1] if job['target_id'] != ORDER[-1] else None)
        return {'active': job['active'], 'phase': job['phase'], 'completed': job['phase'] == 'complete',
                'id': job['target_id'], 'target_id': job['target_id'], 'source_id': 1,
                'next_id': next_id, 'retain_numbered': job.get('retain_numbered', False),
                'occupied_ids': list(occupied), 'scan_complete': job.get('scan_complete', False),
                'target_selected': job.get('target_selected', False),
                'message': job['message'], 'scanned_ids': job.get('scanned_ids', 0),
                'scan_total': 16 if job.get('retain_numbered') else 253,
                'heartbeat_remaining_s': max(0, round(job['heartbeat_deadline'] - self.clock(), 1)) if job['active'] else 0,
                'torque_off': job.get('torque_off', False), 'feedback': job.get('feedback'),
                'serial_closed': job.get('serial_closed', False),
                'error': job.get('error'), 'permanent_pd': job['permanent_pd']}

    def _numbering_check(self, job):
        if self.numbering is not job or not job['active'] or job['cancel'].is_set():
            raise ValueError(job.get('cancel_reason', '用户已停止裸舵机编号'))
        if self.ser is not job.get('serial_context') or self.session is not None:
            raise ValueError('编号串口或会话已变化，停止本任务，不操作其它连接')
        if self.clock() >= job['heartbeat_deadline']:
            raise ValueError('编号页超过 2 秒没有心跳，已停止并请求卸力')
        if self.clock() >= job['run_deadline']:
            raise ValueError('单次编号超过 45 秒总时限，已停止并请求卸力')

    def _numbering_read(self, job, sid, address, count):
        with self.lock:
            self._numbering_check(job)
            return self.read(sid, address, count)

    def _numbering_write(self, job, sid, address, data):
        with self.lock:
            self._numbering_check(job)
            if sid not in job['owned_ids']:
                raise ValueError('编号写入只允许本次新增舵机的原 ID 和新 ID')
            self.write(sid, address, data)

    def _numbering_status(self, job, sid):
        with self.lock:
            self._numbering_check(job)
            return self.status(sid)

    def _numbering_release(self, job):
        # Existing numbered servos are never owned or written by this job.
        errors, confirmed = [], set()
        for sid in sorted(job['owned_ids']):
            try:
                self._disable(sid)
                confirmed.add(sid)
            except Exception as error:
                errors.append(f'ID {sid}: {error}')
        relevant = job.get('actual_id')
        if relevant is not None:
            job['torque_off'] = relevant in confirmed
        if errors:
            job['release_errors'] = errors
        return errors

    def _start_numbering(self, action):
        with self.lock:
            if self.session or self.enabled or (self.numbering and self.numbering['active']):
                raise ValueError('先取消整机单关节授权并卸力，或停止当前编号任务')
            if any(action.get(key) is not True for key in ('single', 'uninstalled', 'clear')):
                raise ValueError('编号会转动：必须确认本次只有一颗出厂 ID 1、新舵机尚未装入关节、舵盘无阻挡')
            if type(action.get('retain_numbered', False)) is not bool:
                raise ValueError('保留已编号选项必须为布尔值')
            target = action.get('next_id')
            if type(target) is not int or target not in ORDER:
                raise ValueError('选择目标编号：10–14、20–24、30–34')
            port = action.get('port')
            if port not in [item.device for item in list_ports.comports()]:
                raise ValueError('明确选择当前 USB/B 转接板串口')
            if type(action.get('permanent_pd', False)) is not bool:
                raise ValueError('永久 PD 选项必须为布尔值，默认关闭')
            created_serial = None
            if not self.ser or self.ser.port != port:
                self.close()
                self.ser = serial.Serial(port, 1000000, timeout=.003, write_timeout=.2)
                created_serial = self.ser
            self.session = None
            job = {'active': True, 'phase': 'preflight', 'message': '只读检查总线，尚未改号或运动',
                   'target_id': target, 'token': secrets.token_hex(16), 'cancel': threading.Event(),
                   'owned_ids': set(), 'heartbeat_deadline': self.clock() + NUMBERING_HEARTBEAT_SECONDS,
                   'run_deadline': self.clock() + NUMBERING_MAX_SECONDS,
                   'permanent_pd': action.get('permanent_pd', False), 'torque_off': False,
                   'retain_numbered': action.get('retain_numbered', False),
                   'occupied_ids': [], 'scan_complete': False, 'target_selected': False,
                   'serial_context': self.ser, 'created_serial': created_serial}
            self.numbering = job
            worker = threading.Thread(target=self._numbering_worker, args=(job,), daemon=True)
            worker.start()
            return {**self.numbering_state(), 'session': job['token']}

    def _numbering_worker(self, job):
        target = job['target_id']
        original_pd = None
        try:
            responding = []
            scan_ids = [1, *ORDER] if job.get('retain_numbered') else range(1, 254)
            for scanned, sid in enumerate(scan_ids, 1):
                try:
                    reply = self._numbering_read(job, sid, 5, 1)
                    if reply == bytes([sid]):
                        responding.append(sid)
                        if not job.get('retain_numbered') and (sid != 1 or len(responding) > 1):
                            raise RuntimeError(f'总线存在额外 ID {sid}，仅允许一颗出厂 ID 1 裸舵机')
                    elif job.get('retain_numbered'):
                        raise RuntimeError(f'ID {sid} 返回的编号不一致；未写入，请检查总线。')
                except ValueError as error:
                    self._numbering_check(job)  # Missing servo is allowed; cancellation is not.
                    if job.get('retain_numbered') and not isinstance(error, NoServoReply):
                        raise
                job['scanned_ids'] = scanned
                job['occupied_ids'] = [value for value in ORDER if value in responding]
            job['scan_complete'] = True
            if job.get('retain_numbered'):
                target = next((sid for sid in ORDER if sid not in responding), None)
                if target is None:
                    job['target_id'] = None
                    job['phase'], job['message'] = 'full', '15 个目标 ID 均已占用；没有改号或运动。'
                    return
                job['target_id'] = target
            if 1 not in responding:
                raise ValueError('未收到出厂 ID 1 回复；保留已编号连接，只新增一颗 ID 1 后再按。')
            # Multiple servos sharing ID 1 cannot be distinguished by this scan.
            # The explicit physical single-servo assertion remains mandatory.
            with self.lock:
                self._numbering_check(job)
                # Recheck the candidate immediately before acquiring write ownership.
                # A responding address is never overwritten, even after the scan.
                if job.get('retain_numbered'):
                    try:
                        self.read(target, 5, 1)
                        occupied = True
                    except NoServoReply:
                        occupied = False
                    self._numbering_check(job)
                    if occupied:
                        raise ValueError(f'目标 ID {target} 刚被占用；本次未写入，请重新扫描。')
                initial = self.status(1)
                self._check_feedback(initial)
                job['owned_ids'] = {1}
                job['actual_id'] = 1
                self._disable(1)
            job['target_selected'] = True
            job['phase'], job['message'] = 'numbering', f'新增 ID 1 改为 {target}；已编号舵机保持不变'
            self._numbering_write(job, 1, 55, [0])
            if self._numbering_read(job, 1, 55, 1) != b'\x00':
                raise ValueError('EEPROM 解锁未读回，停止编号')
            # An interrupted ID write can leave either address responding. Own
            # only these two possible aliases of the one newly added servo.
            job['owned_ids'].add(target)
            self._numbering_write(job, 1, 5, [target])
            if self._numbering_read(job, target, 5, 1) != bytes([target]):
                raise ValueError('新 ID 未读回，停止，不重复改号或运动')
            job['actual_id'], job['owned_ids'] = target, {target}
            if job['permanent_pd']:
                self._numbering_write(job, target, 21, [5, 20])
                if self._numbering_read(job, target, 21, 2) != bytes([5, 20]):
                    raise ValueError('可选永久 KP/KD 未读回，停止，不归中')
            self._numbering_write(job, target, 55, [1])
            if self._numbering_read(job, target, 55, 1) != b'\x01':
                raise ValueError('EEPROM 重新锁定未读回，停止，不归中')
            original_pd = self._numbering_read(job, target, 50, 2)
            self._numbering_write(job, target, 50, [3, 20])
            if self._numbering_read(job, target, 50, 2) != bytes([3, 20]):
                raise ValueError('临时 KP/KD 未读回，停止，不归中')
            state = self._numbering_status(job, target)
            self._check_feedback(state)
            job['phase'], job['message'] = 'centering', f'裸舵机 {target} 正在归中 2047，请保持无阻挡'
            self._numbering_write(job, target, 42, list((2047).to_bytes(2, 'little')) + [0, 0, 44, 1])
            with self.lock:
                self._numbering_check(job)
                self.enabled.add(target)
                self._numbering_write(job, target, 40, [1])
            deadline, stable = self.clock() + NUMBERING_CENTER_SECONDS, 0
            while self.clock() < deadline:
                self.sleep(.08)  # No bus lock while waiting; stop/heartbeat can preempt.
                state = self._numbering_status(job, target)
                self._check_feedback(state)
                job['feedback'] = state
                stable = stable + 1 if abs(state['position'] - 2047) <= 20 else 0
                if stable >= 3:
                    break
            else:
                raise ValueError('8 秒内裸舵机未到中点，停止并卸力，检查阻挡和供电')
            with self.lock:
                self._numbering_check(job)
                self._disable(target)
                final = self.status(target)
                self._check_feedback(final)
                if abs(final['position'] - 2047) > 20:
                    raise ValueError('卸力后位置偏移，未认定归中完成')
                job['feedback'], job['torque_off'] = final, True
            job['phase'] = 'finishing'
        except Exception as error:
            job['phase'], job['error'] = 'stopped' if job['cancel'].is_set() else 'error', str(error)
            job['message'] = str(error)
        finally:
            with self.lock:
                # A replacement connection/session belongs to its own owner.
                # Never issue cleanup writes or close that new serial object.
                same_context = (self.numbering is job and self.ser is job.get('serial_context')
                                and self.session is None and not (self.enabled - job['owned_ids']))
                cleanup_errors = self._numbering_release(job) if same_context else ['编号串口或会话已变化，未操作其它连接；请核对裸舵机供电。']
                # Relock either possible ID after an interrupted ID write. A
                # missing old alias is not evidence that the new alias is safe.
                for sid in sorted(job['owned_ids'] if same_context else ()):
                    try:
                        self.write(sid, 55, [1])
                        if self.read(sid, 55, 1) != b'\x01':
                            raise ValueError('锁定未读回')
                    except Exception as error:
                        cleanup_errors.append(f'ID {sid} 锁定未确认: {error}')
                if same_context and original_pd is not None and job.get('actual_id') == target:
                    try:
                        self.write(target, 50, original_pd)
                        if self.read(target, 50, 2) != original_pd:
                            raise ValueError('原临时 PD 未读回')
                    except Exception as error:
                        cleanup_errors.append(f'临时 PD 恢复未确认: {error}')
                if job['phase'] == 'finishing' and not cleanup_errors and not job['cancel'].is_set():
                    try:
                        event = {'id': target, 'source_id': 1, 'centered': True, 'position': job['feedback']['position'],
                                 'torque_off': True, 'permanent_pd': job['permanent_pd'],
                                 'temporary_pd_restored': True, 'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                                 'scope': 'one-new-factory-id-servo-user-confirmed' if job.get('retain_numbered') else 'physically-isolated-uninstalled-servo-user-confirmed',
                                 'preserved_ids': list(job.get('occupied_ids', []))}
                        with (self.root / 'numbering-history.jsonl').open('a', encoding='utf-8') as history:
                            history.write(json.dumps(event, ensure_ascii=False) + '\n')
                        job['occupied_ids'] = [sid for sid in ORDER if sid in job.get('occupied_ids', []) or sid == target]
                        job['phase'], job['message'] = 'complete', f'ID {target} 已改号、归中 2047 并卸力；已编号的保留，断电新增一颗 ID 1 后再次点击开始'
                        self.found = list(job['occupied_ids'])
                    except OSError as error:
                        job['phase'], job['error'], job['message'] = 'error', str(error), '硬件操作完成但审计保存失败，请人工记录编号'
                elif cleanup_errors:
                    job['phase'], job['message'] = 'error', '停止后状态未全部确认，请断电检查。' + '; '.join(cleanup_errors)
                    job['error'] = job['message']
                elif job['phase'] == 'finishing':
                    job['phase'], job['message'] = 'stopped', '用户停止，已卸力；没有认定编号流程完成'
                owned_serial = job.get('created_serial')
                if same_context and owned_serial is not None and self.ser is owned_serial:
                    try:
                        owned_serial.close()
                    except Exception:
                        job['phase'], job['error'], job['message'] = 'error', '编号串口关闭未确认', '编号已收尾，但串口关闭未确认；保持无遮挡并手动断电。'
                        self.last_error = job['message']
                    else:
                        self.ser, self.found, self.captures = None, [], {}
                        self.enabled.difference_update(job['owned_ids'])
                        job['serial_closed'] = True
                        job['message'] += ' 转接板串口已关闭；供电需手动断开。'
                job['active'] = False

    def action(self, action):
        if not isinstance(action, dict):
            raise ValueError('请求应为 JSON 对象')
        op = action.get('op')
        if op in DISABLED_OPS:
            raise ValueError('整机未按中位装配，自动编号、参数准备和 2047 归中已禁止；请实测单关节调试')
        if op == 'numbering-start':
            return self._start_numbering(action)
        if op == 'numbering-stop':
            job = self.numbering
            if job and job['active']:
                job['cancel_reason'] = '用户已停止裸舵机编号'
                job['cancel'].set()  # Cancellation does not wait for the bus lock.
                with self.lock:
                    self._numbering_release(job)
                    job['message'] = '停止已请求，已授权裸舵机正在卸力和收尾'
            return self.numbering_state()
        if op == 'numbering-heartbeat':
            with self.lock:
                job = self.numbering
                if not job or not secrets.compare_digest(str(action.get('session', '')), job['token']):
                    raise ValueError('编号心跳会话无效')
                self._numbering_check(job)
                job['heartbeat_deadline'] = self.clock() + NUMBERING_HEARTBEAT_SECONDS
                return self.numbering_state()
        if op == 'numbering-state':
            with self.lock:
                return self.numbering_state()
        with self.lock:
            self.watchdog_tick()
            if self.numbering and self.numbering['active']:
                raise ValueError('裸舵机编号模式正在占用总线，先从编号页停止；不能与整机调试并发')
            if op in ('bootstrap', 'auto-stop'):
                if op == 'auto-stop':
                    self.session = None
                    self.watchdog_tick()
                return self.auto_state()
            if op == 'connect':
                port = action.get('port')
                if port not in [item.device for item in list_ports.comports()]:
                    raise ValueError('串口不在当前设备列表')
                self.close()
                self.ser = serial.Serial(port, 1000000, timeout=.003, write_timeout=.2)
                return {**self.safety_state(), 'message': 'USB/B 串口已连接，尚未写参数或恢复运动'}
            if op == 'disconnect':
                self.close()
                return {'message': self.last_error or '已断开，下次连接须重新明确授权'}
            if op == 'scan':
                return self.scan(action.get('full') is True)
            if op == 'disarm':
                self.session = None
                self.watchdog_tick()
                return {**self.safety_state(), 'message': self.last_error or '单关节授权已取消'}
            if op == 'off-all':
                if action.get('supported') is not True:
                    raise ValueError('整机卸力前必须确认身体已由支架牢固承重')
                results = []
                self.session = None
                for target in ORDER:
                    try:
                        self._disable(target)
                        results.append({'id': target, 'torque_off': True})
                    except Exception as error:
                        results.append({'id': target, 'torque_off': False, 'error': str(error)})
                failed = [item['id'] for item in results if not item['torque_off']]
                return {'results': results, 'failed_ids': failed, 'ok': not failed,
                        'message': f'逐颗卸力已确认 {len(ORDER) - len(failed)}/15；未确认 ID：{failed}，必要时切断舵机电源'}
            if op == 'calibration':
                return self.calibration_state()
            if op == 'export-zeros':
                payload = self.export_zeros()
                self._atomic_json(self.root / 'data' / 'zero_pos.json', payload)
                return {**payload, 'message': '独立 data/zero_pos.json 已导出，尚未安装到 Uno Q 运行程序'}
            sid = action.get('id')
            if type(sid) is not int or sid not in ORDER:
                raise ValueError('明确选择一个鸭子关节 ID：10–14、20–24、30–34')
            if op in ('read', 'capture'):
                state = self.status(sid)
                if op == 'capture':
                    self.captures[sid] = state
                return {**state, 'read_only': True, 'message': f'ID {sid} 当前位置已实测，没有运动指令'}
            if op == 'off':
                try:
                    previous = self.status(sid)
                except Exception:
                    previous = {'id': sid, 'position': None, 'feedback_unavailable': True}
                self._disable(sid)
                if self.session and self.session['id'] == sid:
                    self.session = None
                return {**previous, 'torque_enabled': False, 'torque_off': True,
                        'message': f'仅 ID {sid} 卸力已读回；请用支架承受身体重量'}
            if op == 'arm':
                if self.session:
                    raise ValueError('已有单关节授权，先取消再重新读取和授权')
                if action.get('supported') is not True or action.get('clear') is not True:
                    raise ValueError('先确认身体牢固支撑、关节无阻挡并留有断电入口')
                state = self.status(sid)
                self._check_feedback(state)
                if state['torque_enabled']:
                    raise ValueError('舵机仍在出力，先支撑身体并卸力，再授权')
                self.session = {'id': sid, 'token': secrets.token_hex(16), 'anchor': state['position'],
                                'expires': self.clock() + ARM_SECONDS}
                return {**state, 'session': self.session['token'], 'expires_in_s': ARM_SECONDS,
                        'anchor': state['position'], 'max_step_ticks': MAX_STEP_TICKS,
                        'min_target': max(0, state['position'] - MAX_OFFSET_TICKS),
                        'max_target': min(4095, state['position'] + MAX_OFFSET_TICKS),
                        'message': f'仅授权 ID {sid}，30 秒后失效，尚未发送运动指令'}
            if op in ('move', 'jog'):
                return self._jog(action, sid)
            if op == 'save-zero':
                self._check_session(action, sid)
                if action.get('pose_confirmed') is not True:
                    raise ValueError('先人工对照模型摆到此关节的机械零位，再确认保存')
                state = self.status(sid)
                self._check_feedback(state)
                if state['torque_enabled']:
                    raise ValueError('先卸力并人工对准机械零位，再重新授权保存')
                saved = dict(self.zeros)
                saved[sid] = {'position': state['position'], 'saved_at': state['sampled_at'],
                              'voltage': state['voltage'], 'temperature': state['temperature'],
                              'pose_confirmed_by_user': True}
                self._atomic_json(self.root / 'commissioning-zero-captures.json',
                                  {'version': 1, 'records': {str(key): value for key, value in saved.items()}})
                self.zeros = saved
                return {**state, **self.calibration_state(),
                        'message': f'ID {sid} 本机实测机械零位 {state["position"]} 已保存，没有运动或舵机参数写入'}
            raise ValueError('未知操作')


bus = Bus()
provisioner = Provisioner()


class Handler(BaseHTTPRequestHandler):
    def send(self, status, data, ctype='application/json; charset=utf-8', filename=None):
        raw = data if isinstance(data, bytes) else data.encode('utf-8') if isinstance(data, str) else json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if filename:
            self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(raw)

    def _static(self, relative):
        resolved = (ROOT / relative).resolve()
        if not resolved.is_relative_to(ROOT.resolve()) or not resolved.is_file():
            return self.send(404, {'error': '文件不存在'})
        return self.send(200, resolved.read_bytes(), mimetypes.guess_type(resolved.name)[0] or 'application/octet-stream')

    def do_GET(self):
        path = unquote(urlsplit(self.path).path)
        if path == '/api/wireless-calibration/actions':
            with ACTION_DIAGNOSTICS_LOCK:
                events = list(ACTION_DIAGNOSTICS)
            return self.send(200, {'scope': 'Recent proxy outcomes; not physical acceptance', 'events': events})
        if path == '/api/local-session':
            # Same-origin read only. This token protects local HTTP requests;
            # it does not create or renew any hardware authorization.
            return self.send(200, {'token': TOKEN})
        if path == '/api/program-version':
            return self.send(200, program_release_info(ROOT))
        if path == '/api/imu-transport':
            from imu_stream import config_state
            try:
                config=config_state(ROOT)
                return self.send(200,{'enabled':config.get('enabled',False),'scope':'IMU observations only'})
            except (OSError,ValueError):
                return self.send(200,{'enabled':True,'error':'IMU transport configuration unavailable'})
        if path == '/api/imu-live':
            from imu_stream import source_for_imu
            try:source=source_for_imu(ROOT)
            except (OSError,ValueError):return self.send(503,{'error':'IMU read-only transport unavailable'})
            source.acquire()
            try:
                import socket
                self.send_response(200);self.send_header('Content-Type','text/event-stream; charset=utf-8')
                self.send_header('Cache-Control','no-store');self.send_header('Connection','keep-alive');self.end_headers()
                self.connection.settimeout(3);self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                cursor=-1
                while True:
                    sequence,packet=source.wait(cursor,.1)
                    if sequence!=cursor and packet:
                        event='offline' if packet.get('error') else 'status'
                        self.wfile.write(('event: '+event+'\ndata: '+json.dumps(packet,separators=(',',':'),allow_nan=False)+'\n\n').encode('utf-8'))
                        cursor=sequence
                    else:self.wfile.write(b': keepalive\n\n')
                    self.wfile.flush()
            except (OSError,ValueError):pass
            finally:source.release()
            return
        if path == '/api/live':
            from urllib.parse import parse_qs
            from live_relay import source_for
            try:source=source_for(ROOT,parse_qs(urlsplit(self.path).query).get('url',[''])[0])
            except ValueError as error:return self.send(400,{'error':str(error)})
            source.acquire()
            try:
                self.send_response(200);self.send_header('Content-Type','text/event-stream; charset=utf-8')
                self.send_header('Cache-Control','no-store');self.send_header('Connection','keep-alive');self.end_headers()
                import socket
                self.connection.settimeout(3);self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                cursor=-1
                while True:
                    next_cursor,batch=source.wait_batch(cursor,1)
                    if next_cursor!=cursor and batch:
                        for _,packet in batch:
                            event='offline' if packet.get('error') else 'status'
                            data=json.dumps(packet,separators=(',',':'),ensure_ascii=False,allow_nan=False)
                            self.wfile.write(('event: '+event+'\ndata: '+data+'\n\n').encode('utf-8'))
                        cursor=next_cursor
                    else:self.wfile.write(b': keepalive\n\n')
                    self.wfile.flush()
            except (OSError,ValueError):pass
            finally:source.release()
            return
        if path == '/api/wireless-calibration':
            from wireless_calibration_proxy import proxy
            return self.send(200, proxy.status())
        if path == '/api/provision/status':
            state=provisioner.status()
            from live_relay import configured_target
            state['configured_network_url']=configured_target(ROOT)
            return self.send(200, state)
        if path == '/api/auto':
            return self.send(200, bus.auto_state())
        if path == '/api/numbering':
            with bus.lock:
                return self.send(200, bus.numbering_state())
        if path == '/api/calibration':
            with bus.lock:
                return self.send(200, bus.calibration_state())
        if path == '/api/zeros/export':
            try:
                with bus.lock:
                    return self.send(200, bus.export_zeros(), filename='zero_pos.json')
            except ValueError as error:
                return self.send(400, {'error': str(error)})
        if path in ('/guide', '/runtime-guide'):
            return self._static('guide.html' if path == '/guide' else 'runtime-guide.html')
        if path == '/numbering':
            return self.send(200, (ROOT / 'numbering.html').read_text('utf-8').replace('__TOKEN__', TOKEN), 'text/html; charset=utf-8')
        if path in ('/commissioning-guide.json', '/model-view.html', '/model-view.js'):
            return self._static(path[1:])
        if path == '/api/board-status':
            try:
                from commissioning_support import board_status
                from urllib.parse import parse_qs
                target = parse_qs(urlsplit(self.path).query).get('url', [''])[0]
                return self.send(200, board_status(target))
            except Exception as error:
                return self.send(400, {'error': str(error)})
        if any(path.startswith(prefix) for prefix in ('/model-assets/', '/downloads/', '/assets/')):
            # Resolve traversal/symlinks inside that particular whitelisted tree.
            tree = path.split('/')[1]
            resolved = (ROOT / path[1:]).resolve()
            if not resolved.is_relative_to((ROOT / tree).resolve()):
                return self.send(404, {'error': '非法路径'})
            return self._static(path[1:])
        if path.startswith('/guide-image/'):
            try:
                number = int(path.split('/')[-1])
                if not 1 <= number <= 25:
                    raise ValueError()
                return self.send(200, (ROOT.parent / 'research' / f'Assembly_Guide_{number}.png').read_bytes(), 'image/png')
            except Exception:
                return self.send(404, {'error': '图片不存在'})
        if path == '/api/ports':
            return self.send(200, {'ports': [{'port': item.device, 'name': item.description} for item in list_ports.comports()]})
        if path in ('/api/health', '/api/state'):
            with bus.lock:
                return self.send(200, {'ok': True, **bus.safety_state()})
        if path == '/':
            return self.send(200, (ROOT / 'index.html').read_text('utf-8').replace('__TOKEN__', TOKEN), 'text/html; charset=utf-8')
        self.send(404, {'error': '不存在'})

    def do_POST(self):
        path = urlsplit(self.path).path
        if path not in ('/api/action', '/api/wireless-calibration/action', '/api/wireless-calibration/update', '/api/wireless-calibration/usb', '/api/provision/start', '/api/provision/discover', '/api/provision/update'):
            return self.send(403, {'error': '非法请求'})
        if self.headers.get('X-Token') != TOKEN:
            return self.send(403, {'code': 'local_session_expired',
                                   'error': '本地服务已重启；本次操作未发送，请重新点击。'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('请求大小无效')
            action = json.loads(self.rfile.read(length))
            if not isinstance(action, dict):
                raise ValueError('请求格式无效')
            if legacy_update_blocked(path):
                return self.send(409, {'code':'reviewed_update_required',
                    'error':'慢速保持程序使用本次已核验的增量更新包；旧版烧录入口已停用，避免覆盖新版。'})
            if path in ('/api/wireless-calibration/update','/api/wireless-calibration/usb'):
                from wireless_calibration_proxy import proxy
                with bus.lock:
                    if bus.ser or bus.session or bus.enabled or (bus.numbering and bus.numbering['active']) or proxy.busy():
                        raise ValueError('先停止所有校准会话并确认卸力，再更新无线校准程序')
                    kwargs={'calibration_usb':True} if path.endswith('/usb') else {'calibration_update':True}
                    return self.send(200, provisioner.start(action,**kwargs))
            if path == '/api/wireless-calibration/action':
                return self.send(200, perform_wireless_action(action))
            if path in ('/api/provision/start', '/api/provision/discover', '/api/provision/update'):
                with bus.lock:
                    from wireless_calibration_proxy import proxy
                    if proxy.busy():
                        raise ValueError('先停止无线小步并确认卸力，再更新主控')
                    if bus.ser or bus.session or bus.enabled or (bus.numbering and bus.numbering['active']):
                        raise ValueError('先断开舵机转接板总线并取消关节/编号授权，再进行部署、更新或无线发现')
                    result = provisioner.start(action, discover=path.endswith('/discover'), wireless_update=path.endswith('/update'))
            else:
                with bus.lock:
                    from wireless_calibration_proxy import proxy
                    if proxy.busy() and action.get('op') not in ('bootstrap', 'calibration', 'numbering-state', 'disarm', 'auto-stop'):
                        raise ValueError('无线校准占用中，先停止并确认卸力再使用备用工具')
                    if provisioner.status()['active'] and action.get('op') not in ('bootstrap', 'calibration', 'numbering-state', 'disarm', 'auto-stop'):
                        raise ValueError('部署、更新或无线发现流程正在执行，暂不允许舵机总线操作')
                    result = bus.action(action)
            self.send(200, result)
        except Exception as error:
            from wireless_calibration_proxy import CalibrationProxyError
            self.send(400, error.public() if isinstance(error, CalibrationProxyError) else {'error': str(error)})


if __name__ == '__main__':
    threading.Thread(target=bus.watch, daemon=True).start()
    print('Duck assembled commissioning http://127.0.0.1:8791/ (startup read-only)', flush=True)
    ThreadingHTTPServer(('127.0.0.1', 8791), Handler).serve_forever()
