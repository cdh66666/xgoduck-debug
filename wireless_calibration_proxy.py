"""Local UI proxy to the pinned Uno Q over an authenticated SSH tunnel.

GETs never arm, move, save, or renew a calibration lease. Mutations are sent
once; an uncertain reply is not retried. Device-side safety is authoritative.
"""
from device_config import DEVICE_SERIAL
import json
from collections import deque
import hashlib
import math
import os
import re
import socket
import subprocess
import threading
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener
from wireless_rediscovery import AutomaticRediscovery, persist_verified

ROOT = Path(__file__).resolve().parent
IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
VERSION = '2026-10-15-wireless-calibration-v7'
HOME_CONTRACT_REVISION = 5
OPS = frozenset(('read', 'off_all', 'capture_all','capture_group', 'arm', 'jog', 'cancel', 'heartbeat',
                 'home_prepare','home_run','home_heartbeat','home_stop', 'read_servo_parameters',
                 'sequence_begin', 'sequence_select', 'sequence_capture_hold', 'sequence_heartbeat', 'sequence_stop'))
MAX_REPLY = 128 * 1024


def _armed_active(status):
    return isinstance(status, dict) and isinstance(status.get('armed'), dict) and status['armed'].get('active') is True


def _hardware_may_be_active(status):
    if not isinstance(status, dict):
        return False
    flags = status.get('mcu_flags')
    rows = status.get('joints') if isinstance(status.get('joints'), list) else []
    home=status.get('home') if isinstance(status.get('home'),dict) else {}
    sequential=status.get('sequence_calibration') if isinstance(status.get('sequence_calibration'),dict) else {}
    return bool(_armed_active(status) or home.get('active') is True or home.get('session_active') is True
                or sequential.get('active') is True or sequential.get('held_ids')
                or home.get('phase') in ('preparing','prepared','enabling','moving','holding','releasing')
                or status.get('torque_may_be_on') is True
                or type(flags) is int and flags & 12
                or status.get('mcu_state') in (4, 5, 6)
                or status.get('operation')
                or any(isinstance(row, dict) and row.get('torque_enabled') is True for row in rows))


def _released_verified(status):
    if not isinstance(status, dict) or status.get('fresh') is not True or status.get('policy_locked') is not True or _hardware_may_be_active(status):
        return False
    flags = status.get('mcu_flags')
    declared = status.get('all_torque_off_verified') is True or type(flags) is int and bool(flags & 16)
    rows = {row.get('id'): row for row in status.get('joints', []) if isinstance(row, dict)} if isinstance(status.get('joints'), list) else {}
    return bool(declared and all(sid in rows and rows[sid].get('torque_enabled') is False
                                and type(rows[sid].get('metadata_age_ms')) in (int, float)
                                and 0 <= rows[sid]['metadata_age_ms'] <= 750 for sid in IDS))


class CalibrationProxyError(ValueError):
    def __init__(self, message, reason=None, blocked_ids=None):
        super().__init__(str(message)[:400])
        self.reason = reason if isinstance(reason, str) and re.fullmatch(r'[a-z0-9_]{1,80}', reason) else None
        self.blocked_ids = sorted({sid for sid in blocked_ids if type(sid) is int and sid in IDS}) if isinstance(blocked_ids, list) else []

    def public(self):
        result = {'error': str(self)}
        if self.reason:
            result['reason'] = self.reason
        if self.blocked_ids:
            result['blocked_ids'] = self.blocked_ids
        return result


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


def _small_json(path):
    try:
        if path.is_symlink() or (hasattr(path, 'is_junction') and path.is_junction()):
            return None
        raw = path.read_bytes()
        return json.loads(raw) if len(raw) <= 65536 else None
    except (OSError, ValueError, TypeError):
        return None


def _checked_action(action):
    if not isinstance(action, dict) or action.get('op') not in OPS:
        raise CalibrationProxyError('未知无线校准操作')
    op = action['op']
    output = {'op': op}
    if op == 'read_servo_parameters':
        if type(action.get('id')) is not int or action['id'] not in IDS or set(action) != {'op', 'id'}:
            raise CalibrationProxyError('寄存器读取只接受关节编号')
        return {'op': op, 'id': action['id']}
    if op.startswith('sequence_'):
        allowed = {'op', 'id', 'sequence_contract_revision'}
        target = action.get('id', 0)
        if type(target) is not int or target not in (IDS if op in ('sequence_select', 'sequence_capture_hold') else (0,)):
            raise CalibrationProxyError('请选择准确的逐颗校准关节')
        output['id'] = target
        if op != 'sequence_stop':
            if type(action.get('sequence_contract_revision')) is not int or action['sequence_contract_revision'] != 1:
                raise CalibrationProxyError('逐颗校准页面版本不匹配，请刷新', reason='sequence_contract_mismatch')
            output['sequence_contract_revision'] = 1
        if op == 'sequence_begin':
            allowed.update(('supported', 'clear'))
            if action.get('supported') is not True or action.get('clear') is not True:
                raise CalibrationProxyError('先托稳机身并确认活动余量')
            output.update(supported=True, clear=True)
        if op == 'sequence_capture_hold':
            allowed.update(('supported', 'pose_confirmed'))
            if action.get('supported') is not True or action.get('pose_confirmed') is not True:
                raise CalibrationProxyError('先确认本颗机械零位并托稳机身')
            output.update(supported=True, pose_confirmed=True)
        if op in ('sequence_select', 'sequence_capture_hold', 'sequence_heartbeat') or op == 'sequence_stop' and action.get('session') is not None:
            allowed.add('session')
            if not isinstance(action.get('session'), str) or not re.fullmatch(r'[0-9a-f]{16,128}', action['session']):
                raise CalibrationProxyError('逐颗校准会话已失效')
            output['session'] = action['session']
        if set(action) - allowed:
            raise CalibrationProxyError('逐颗校准不接受外部目标或参数')
        return output
    if op.startswith('home_'):
        allowed={'op'}
        if op in ('home_prepare','home_run','home_heartbeat'):
            revision=action.get('home_contract_revision')
            if type(revision) is not int or revision != HOME_CONTRACT_REVISION:
                raise CalibrationProxyError('归位页面版本已更新，请刷新后重新操作',reason='home_contract_mismatch')
            allowed.add('home_contract_revision');output['home_contract_revision']=revision
        elif op=='home_stop':
            # STOP must work across a rolling update. Optional client revision
            # is deliberately stripped before forwarding the unchanged stop.
            allowed.add('home_contract_revision')
        if op in ('home_prepare','home_run'):
            allowed.add('native_sha256')
            digest=action.get('native_sha256')
            if not isinstance(digest,str) or not re.fullmatch(r'[0-9a-f]{64}',digest):
                raise CalibrationProxyError('完整零位文件尚未核对')
            output['native_sha256']=digest
        if op=='home_prepare':
            allowed.update(('supported','clear'))
            if action.get('supported') is not True or action.get('clear') is not True:
                raise CalibrationProxyError('先托稳机身，确认可以慢速归位')
            output.update(supported=True,clear=True)
        if op in ('home_run','home_heartbeat') or op=='home_stop' and action.get('session') is not None:
            allowed.add('session');session=action.get('session')
            if not isinstance(session,str) or not re.fullmatch(r'[0-9a-f]{16,128}',session):
                raise CalibrationProxyError('归位授权已失效')
            output['session']=session
        if set(action)-allowed:raise CalibrationProxyError('归位仅使用已记录的零位')
        return output
    if op in ('read', 'arm', 'jog', 'heartbeat'):
        sid = action.get('id')
        if type(sid) is not int or sid not in IDS:
            raise CalibrationProxyError('请选择正确的关节 ID')
        output['id'] = sid
    if op in ('arm', 'off_all', 'capture_all','capture_group'):
        if action.get('supported') is not True:
            raise CalibrationProxyError('先托稳机身')
        output['supported'] = True
    if op == 'arm':
        if action.get('clear') is not True:
            raise CalibrationProxyError('先确认关节与线束有活动余量')
        output['clear'] = True
    if op == 'capture_all':
        if action.get('all_aligned') is not True:
            raise CalibrationProxyError('先按图摆好全部零位')
        output['pose_confirmed'] = True
    if op == 'capture_group':
        if action.get('group') not in ('left','right','head') or action.get('pose_confirmed') is not True:
            raise CalibrationProxyError('先选择并摆好这一组零位')
        output.update(group=action['group'],pose_confirmed=True)
    if op in ('jog', 'heartbeat'):
        session = action.get('session')
        if not isinstance(session, str) or not re.fullmatch(r'[0-9a-f]{16,128}', session):
            raise CalibrationProxyError('单关节授权已失效')
        output['session'] = session
    elif op == 'cancel' and action.get('session') is not None:
        session = action['session']
        if not isinstance(session, str) or not re.fullmatch(r'[0-9a-f]{16,128}', session):
            raise CalibrationProxyError('单关节授权无效')
        output['session'] = session
    if op == 'jog':
        delta = action.get('model_delta_deg')
        if type(delta) not in (int, float) or not math.isfinite(delta):
            raise CalibrationProxyError('小步角度无效')
        ticks = abs(delta) * 4095 / 360
        if not any(abs(ticks - allowed) <= 0.45 for allowed in (3, 11)):
            raise CalibrationProxyError('仅允许页面提供的小步角度')
        output['model_delta_deg'] = delta
    return output


class _StatusSnapshot(dict):
    def __init__(self, value, epoch, process):
        super().__init__(value)
        self.transport_epoch, self.transport_process = epoch, process


class WirelessCalibrationProxy:
    def __init__(self, root=ROOT, request=None, clock=time.monotonic):
        self.root, self.clock = Path(root), clock
        self.lock = threading.RLock()
        self.process = None
        self.local_port = None
        self.target = None
        self.retry_after = 0
        self.last_status = None
        self.inflight = 0
        self.uncertain = False
        self.request_override = request  # Pure offline test injection only.
        self.connection_failed = False
        self.connection_epoch = 0
        self.sessions_invalidated = False
        self.valid_sessions = set()
        self.read_timeout_epoch = None
        self.read_timeout_process = None
        self.read_timeout_observed_at = None
        self.transport_events = deque(maxlen=32)
        self.transport_event_id = 0
        self.rediscovery = AutomaticRediscovery(self.root, self._discovery_allowed,
                                               self._accept_discovery, clock=clock)

    def _discovery_allowed(self):
        with self.lock:
            return self.connection_failed and not self.update_interlocked() and not self.inflight

    def _accept_discovery(self, result):
        with self.lock:
            if not self._discovery_allowed():
                return False
            import discover_unoq as discovery
            url = discovery.literal_url(result.get('network_url')) if isinstance(result, dict) else None
            if not url or not persist_verified(self.root, result):
                return False
            self.retry_after = 0
            return True

    def _transport_event(self, kind, *, request_epoch=None, request_process=None, elapsed_ms=None):
        with self.lock:
            self.transport_event_id += 1
            pid = getattr(self.process, 'pid', None)
            request_pid = getattr(request_process, 'pid', None)
            event = {'id': self.transport_event_id, 'at_unix_s': round(time.time(), 3),
                     'kind': kind, 'epoch': self.connection_epoch,
                     'pid': pid if type(pid) is int else None}
            if request_epoch is not None:
                event['request_epoch'] = request_epoch
                event['request_pid'] = request_pid if type(request_pid) is int else None
            if elapsed_ms is not None:
                event['elapsed_ms'] = round(max(0, elapsed_ms), 2)
            self.transport_events.append(event)

    def transport_diagnostics(self):
        with self.lock:
            return {'epoch': self.connection_epoch,
                    'events': [dict(event) for event in self.transport_events],
                    'scope': 'PC tunnel lifecycle only; no command, address, path or credential data'}

    def _transport_failed(self, *, expected_epoch=None, expected_process=None,
                          owned=False, reason='transport_failed', elapsed_ms=None):
        with self.lock:
            # A late failure belongs to the exact request tunnel, not whichever
            # child a live reader has since established. Never touch that child.
            if owned and (expected_epoch != self.connection_epoch or expected_process is not self.process):
                self._transport_event('obsolete_failure_ignored', request_epoch=expected_epoch,
                                      request_process=expected_process, elapsed_ms=elapsed_ms)
                return False
            old_epoch, old_process = self.connection_epoch, self.process
            if not self.connection_failed:
                self.connection_epoch += 1
            self.connection_failed = self.sessions_invalidated = True
            self.valid_sessions.clear()
            if _hardware_may_be_active(self.last_status):
                self.uncertain = True
            self._transport_event(reason, request_epoch=old_epoch, request_process=old_process, elapsed_ms=elapsed_ms)
            if old_process is not None and old_process.poll() is None:
                old_process.terminate()  # Only the exact failed request's SSH child.
            self.process, self.local_port = None, None
            self.read_timeout_epoch = self.read_timeout_process = self.read_timeout_observed_at = None
        self.rediscovery.request()
        return True

    def _preserve_first_read_timeout(self, epoch, process, started, elapsed_ms):
        """A lone GET timeout does not prove the shared SSE tunnel has failed.

        Only a later GET begun after this timeout may trigger tunnel recovery;
        overlapping readers must not count as separate failures.
        """
        with self.lock:
            if epoch != self.connection_epoch or process is not self.process:
                return False
            if process is None or process.poll() is not None:
                return False
            same = self.read_timeout_epoch == epoch and self.read_timeout_process is process
            if same:
                if started > self.read_timeout_observed_at:
                    return False
                # All already-running readers share the first failed
                # observation; none may postpone its recovery boundary.
                return True
            self.read_timeout_epoch, self.read_timeout_process = epoch, process
            self.read_timeout_observed_at = self.clock()
            # The status is unknown, so no existing motion authorization may
            # be resumed. Emergency stop and fresh read operations remain open.
            self.sessions_invalidated = True
            self.valid_sessions.clear()
            if _hardware_may_be_active(self.last_status):
                self.uncertain = True
            self._transport_event('read_timeout_tunnel_preserved', request_epoch=epoch,
                                  request_process=process, elapsed_ms=elapsed_ms)
            return True

    def _clear_read_timeout(self, epoch, process):
        with self.lock:
            if epoch == self.connection_epoch and process is self.process:
                self.read_timeout_epoch = self.read_timeout_process = self.read_timeout_observed_at = None

    def reserve(self, action):
        _checked_action(action)
        with self.lock:
            self.inflight += 1

    def unreserve(self):
        with self.lock:
            self.inflight = max(0, self.inflight - 1)

    def busy(self):
        with self.lock:
            return bool(self.inflight or self.uncertain or _hardware_may_be_active(self.last_status) or self.update_interlocked())

    def update_interlocked(self):
        # This is a conservative interlock, not proof that a process is live.
        # An uncertain update keeps it until its exact job is audited.
        return (self.root / 'tools' / '.private' / 'wireless-calibration-update-in-progress.json').exists()

    def _remember(self, status):
        if not isinstance(status, dict):
            return
        with self.lock:
            previous = self.last_status
            if previous and previous.get('boot_id') == status.get('boot_id'):
                old, new = previous.get('sequence'), status.get('sequence')
                if type(old) is int and type(new) is int:
                    step = (new - old) & 0xffffffff
                    if step >= 0x80000000:
                        return  # A late GET must not replace a newer action ACK.
                    if step == 0 and _hardware_may_be_active(previous) and not _hardware_may_be_active(status):
                        return
            self.last_status = status
            if status.get('fresh') is True:
                self.connection_failed = False
            if _released_verified(status) and not self.inflight:
                self.uncertain = False

    def _configuration(self):
        import sys
        tools = str(self.root / 'tools')
        if tools not in sys.path:
            sys.path.insert(0, tools)
        import deploy_readonly_wifi as fixed_update
        import discover_unoq as discovery
        try:
            identity = fixed_update.strict_identity(self.root)
        except fixed_update.UpdateError:
            raise CalibrationProxyError('主控身份未配置', reason='credential_missing') from None
        if identity.get('serial') != DEVICE_SERIAL:
            raise CalibrationProxyError('主控身份未核对')
        choices = discovery.cached_candidates(self.root)
        if not choices:
            raise CalibrationProxyError('尚未发现已绑定的无线主控')
        target = discovery.literal_url(choices[0])
        if not target:
            raise CalibrationProxyError('无线地址无效')
        return identity, target, discovery

    def _auth_key(self):
        record = _small_json(self.root / 'tools' / '.private' / 'wireless-calibration-auth.json')
        key = record.get('key') if isinstance(record, dict) else None
        if (not isinstance(record, dict) or record.get('version') != 1
                or record.get('serial') != DEVICE_SERIAL
                or not isinstance(key, str) or not re.fullmatch(r'[0-9a-f]{64}', key)):
            raise CalibrationProxyError('无线校准程序待更新')
        return key

    def readonly_endpoint(self, expected_target):
        """Reuse the pinned tunnel for telemetry; never send an action or lease."""
        with self.lock:
            endpoint=self._ensure_tunnel(read_only=True)
            if self.target!=expected_target:
                raise CalibrationProxyError('无线地址已更新，等待页面重新订阅')
            return endpoint

    def _ensure_tunnel(self, read_only=False):
        with self.lock:
            try:
                identity, target, discovery = self._configuration()
            except (CalibrationProxyError, OSError, ValueError):
                self._transport_event('configuration_rejected')
                if self.process is None or self.process.poll() is not None:
                    self._transport_failed(expected_epoch=self.connection_epoch, expected_process=self.process,
                                           owned=True, reason='configuration_failed')
                raise CalibrationProxyError('无线主控配置不可用') from None
            if self.process is not None and self.process.poll() is None and self.local_port:
                if target == self.target:
                    return f'http://127.0.0.1:{self.local_port}'
                if (self.inflight or self.uncertain or _hardware_may_be_active(self.last_status)) and not (read_only and self.connection_failed):
                    raise CalibrationProxyError('主控地址已变化；等待当前操作停止后重连')
                self.process.terminate()  # Only this proxy's idle SSH child.
                self.process, self.local_port = None, None
                self.retry_after = 0
            if self.process is not None and self.process.poll() is not None:
                self._transport_failed()
            if self.clock() < self.retry_after:
                raise CalibrationProxyError('无线主控未连接')
            self.retry_after = self.clock() + 3
            try:
                ip = discovery.private_ipv4(urlsplit(target).hostname)
                if not ip:
                    raise CalibrationProxyError('无线地址无效')
                # Fail quickly while the user is charging/powering off the robot.
                with socket.create_connection((ip, 22), timeout=2.5):
                    pass
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as reserve:
                    reserve.bind(('127.0.0.1', 0))
                    port = reserve.getsockname()[1]
                ssh_exe, null_config = discovery.SSH_EXE, 'NUL'
                transport = _small_json(self.root / 'tools' / '.private' / 'ssh-client.json')
                if transport and transport.get('backend') == 'git-openssh':
                    candidate = Path('C:/Program Files/Git/usr/bin/ssh.exe')
                    if (not candidate.is_file() or hashlib.sha256(candidate.read_bytes()).hexdigest()
                            != transport.get('executable_sha256')):
                        raise CalibrationProxyError('已核对的 SSH 工具发生变化')
                    ssh_exe, null_config = candidate, '/dev/null'
                args = [str(ssh_exe), '-F', null_config, '-N', '-T',
                    '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
                    '-o', 'IdentityAgent=none', '-o', 'StrictHostKeyChecking=yes',
                    '-o', 'UpdateHostKeys=no', '-o', 'CheckHostIP=no',
                    '-o', 'PasswordAuthentication=no', '-o', 'KbdInteractiveAuthentication=no',
                    '-o', 'ConnectTimeout=5', '-o', 'ConnectionAttempts=1',
                    '-o', 'GlobalKnownHostsFile=' + null_config,
                    '-o', 'UserKnownHostsFile=' + str(identity['known_hosts_path']),
                    '-o', 'HostKeyAlias=' + identity['host_key_alias'],
                    '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=3',
                    '-o', 'ServerAliveCountMax=2', '-i', str(identity['key_path']),
                    '-L', f'127.0.0.1:{port}:127.0.0.1:9527', 'arduino@' + ip]
                process = subprocess.Popen(args, stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    shell=False, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                deadline = self.clock() + 7
                while self.clock() < deadline and process.poll() is None:
                    try:
                        with socket.create_connection(('127.0.0.1', port), timeout=.08):
                            self.process, self.local_port, self.target = process, port, target
                            self._transport_event('tunnel_started')
                            return f'http://127.0.0.1:{port}'
                    except OSError:
                        time.sleep(.04)
                if process.poll() is None:
                    process.terminate()  # Only the exact child created above.
                raise CalibrationProxyError('无线主控未连接')
            except CalibrationProxyError:
                self._transport_failed()
                raise
            except Exception:
                self._transport_failed()
                raise CalibrationProxyError('无线主控未连接') from None

    def _request(self, method, body=None):
        if self.request_override is not None:
            return self.request_override(method, body)
        with self.lock:
            base = self._ensure_tunnel(read_only=method == 'GET' or body.get('op') in ('cancel', 'home_stop', 'sequence_stop', 'off_all', 'read', 'read_servo_parameters'))
            epoch, process = self.connection_epoch, self.process
            if method == 'POST':
                self._check_session(body)
        path = '/api/calibration' if method == 'GET' else '/api/calibration/action'
        headers = {'Accept': 'application/json'}
        payload = None
        if method == 'POST':
            headers.update({'Content-Type': 'application/json', 'X-Cal-Key': self._auth_key()})
            payload = json.dumps(body, separators=(',', ':'), allow_nan=False).encode('utf-8')
        req = Request(base + path, data=payload, method=method, headers=headers)
        started = self.clock()
        try:
            with build_opener(ProxyHandler({}), NoRedirect()).open(req, timeout=8 if method == 'POST' and body.get('op') in ('capture_all','capture_group','sequence_capture_hold') else 3) as reply:
                raw = reply.read(MAX_REPLY + 1)
        except HTTPError as error:
            if error.code == 404:
                raise CalibrationProxyError('无线校准程序待更新', reason='not_installed') from None
            if error.code in (401, 403):
                raise CalibrationProxyError('无线校准身份校验未通过') from None
            raw = error.read(MAX_REPLY + 1)
        except (URLError, OSError, TimeoutError) as error:
            cause = error.reason if isinstance(error, URLError) else error
            reason = 'request_timeout' if isinstance(cause, TimeoutError) else 'request_transport_error'
            elapsed_ms = (self.clock() - started) * 1000
            if (method == 'GET' and reason == 'request_timeout'
                    and self._preserve_first_read_timeout(epoch, process, started, elapsed_ms)):
                raise CalibrationProxyError('无线只读状态超时；等待新鲜反馈', reason='read_timeout') from None
            applied = self._transport_failed(expected_epoch=epoch, expected_process=process, owned=True,
                                             reason=reason, elapsed_ms=elapsed_ms)
            if not applied:
                raise CalibrationProxyError('连接已变更；旧请求回执已作废', reason='connection_changed') from None
            raise CalibrationProxyError('无线请求未确认；已停止本地操作，请等待卸力状态') from None
        def invalid_response(kind, message='无线响应无效'):
            applied = self._transport_failed(expected_epoch=epoch, expected_process=process, owned=True,
                                             reason=kind, elapsed_ms=(self.clock()-started)*1000)
            if not applied:
                raise CalibrationProxyError('连接已变更；旧请求回执已作废', reason='connection_changed') from None
            raise CalibrationProxyError(message) from None
        if len(raw) > MAX_REPLY:
            invalid_response('invalid_reply_oversize')
        try:
            result = json.loads(raw)
        except (ValueError, UnicodeError):
            invalid_response('invalid_reply_json')
        if not isinstance(result, dict):
            invalid_response('invalid_reply_shape')
        with self.lock:
            if epoch != self.connection_epoch or process is not self.process:
                self._transport_event('obsolete_reply_ignored', request_epoch=epoch, request_process=process)
                raise CalibrationProxyError('连接已变更；旧请求回执已作废', reason='connection_changed')
        if method == 'GET':
            if result.get('schema') != 'wireless-calibration-v2':
                invalid_response('invalid_status_schema', '无线校准程序版本不符')
            self._clear_read_timeout(epoch, process)
            result['pc_roundtrip_ms'] = round((self.clock() - started) * 1000, 2)
            return _StatusSnapshot(result, epoch, process)
        self._clear_read_timeout(epoch, process)
        return result

    def status(self):
        try:
            with self.lock:
                fallback_epoch, fallback_process = self.connection_epoch, self.process
            status = self._request('GET')
            epoch = getattr(status, 'transport_epoch', fallback_epoch)
            process = getattr(status, 'transport_process', fallback_process)
            with self.lock:
                if epoch != self.connection_epoch or process is not self.process:
                    self._transport_event('obsolete_status_ignored', request_epoch=epoch, request_process=process)
                    raise CalibrationProxyError('连接已变更；旧状态已作废', reason='connection_changed')
                self._remember(status)
            return {'connection': {'online': True, 'url': self.target,
                    'transport': 'pinned_ssh', 'error': None,
                    'rediscovery': self.rediscovery.snapshot(),
                    'transport_diagnostics': self.transport_diagnostics(),
                    'update_interlocked': self.update_interlocked()}, 'status': status}
        except CalibrationProxyError as error:
            # _ensure_tunnel/_request own transport failures. An obsolete GET
            # or an application/schema rejection must not kill a newer reader.
            self._transport_event('status_rejected')
            return {'connection': {'online': error.reason == 'not_installed', 'url': self.target,
                    'transport': 'pinned_ssh', 'error': str(error),
                    'rediscovery': self.rediscovery.snapshot(),
                    'transport_diagnostics': self.transport_diagnostics(),
                    'update_interlocked': self.update_interlocked()}, 'status': None}

    def _check_session(self, body):
        if (self.sessions_invalidated and body['op'] in ('jog', 'heartbeat', 'home_run', 'home_heartbeat', 'sequence_select', 'sequence_capture_hold', 'sequence_heartbeat')
                and body.get('session') not in self.valid_sessions):
            raise CalibrationProxyError('连接已中断；请重新发起操作，旧授权不再续用', reason='connection_changed')

    def action(self, action):
        body = _checked_action(action)
        with self.lock:
            self._check_session(body)
            epoch = self.connection_epoch
            if self.uncertain and body['op'] not in ('cancel','off_all','read','heartbeat','home_stop','home_heartbeat','sequence_stop','sequence_heartbeat','read_servo_parameters'):
                raise CalibrationProxyError('先停止并确认卸力，再开始新动作',reason='release_unconfirmed')
        if self.update_interlocked() and body['op'] not in ('cancel', 'heartbeat', 'read','home_heartbeat','home_stop','sequence_heartbeat','sequence_stop','read_servo_parameters'):
            raise CalibrationProxyError('无线更新尚未确认，暂不开始新的校准操作')
        # No network lock spans a round trip: heartbeat and cancel must remain
        # deliverable while a jog waits for its device-side acknowledgement.
        try:
            result = self._request('POST', body)
            with self.lock:
                if epoch != self.connection_epoch:
                    raise CalibrationProxyError('连接已变更；旧请求回执已作废', reason='connection_changed')
        except CalibrationProxyError:
            if body['op'] in ('arm', 'jog','home_prepare','home_run','home_stop','sequence_begin','sequence_select','sequence_capture_hold','sequence_stop'):
                with self.lock:
                    self.uncertain = True
            raise
        if result.get('ok') is not True:
            detail = result.get('detail') if isinstance(result.get('detail'), dict) else result
            reason = detail.get('reason') or detail.get('code') or detail.get('error')
            definitely_unsent_prepare = body['op']=='home_prepare' and reason=='command_route_unavailable'
            if body['op'] in ('home_prepare','home_run','home_stop','sequence_begin','sequence_select','sequence_capture_hold','sequence_stop') and not definitely_unsent_prepare:
                with self.lock:self.uncertain=True
                self._remember(result.get('status'))
            message = detail.get('message') or detail.get('error') or reason or '无线操作未完成'
            message = message if isinstance(message, str) else '无线操作未完成'
            raise CalibrationProxyError(message, reason=reason, blocked_ids=detail.get('blocked_ids',detail.get('ids')))
        with self.lock:
            if epoch != self.connection_epoch:
                self.uncertain = True
                raise CalibrationProxyError('连接已变更；旧请求回执已作废', reason='connection_changed')
            if body['op'] in ('arm', 'home_prepare', 'sequence_begin') and isinstance(result.get('session'), str):
                self.valid_sessions = {result['session']}
            self._remember(result.get('status'))
        return result

    def close(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
            self.process = None
            self.local_port = None


proxy = WirelessCalibrationProxy()
