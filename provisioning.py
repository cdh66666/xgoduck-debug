"""Fixed-command, redacted USB deployment, wireless updates and discovery.

Import/restart never launches a process. Raw stdout/stderr and credentials are
never persisted or returned by this module; only known JSON fields are accepted.
"""
import shutil
import sys
import ipaddress
import json
import math
import queue
import re
import subprocess
import threading
import time
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit
from uuid import UUID

PROJECT_ROOT = Path(__file__).resolve().parent
PWSH = Path(shutil.which('pwsh') or 'pwsh')
PYTHON = Path(sys.executable)
STAGES = {
    'idle': '先用 USB 数据线连接 Uno Q，保持舵机电源断开',
    'prepare': '准备固定部署工具', 'detect_usb': '识别 USB 上的 Uno Q',
    'compile': '编译主控程序', 'upload': '烧录主控程序',
    'deploy': '安装无线调试服务', 'wifi': '使用电脑当前已保存的 Wi-Fi 配置',
    'verify_usb': '读取 USB 侧结果', 'verify_network': '读取无线服务结果',
    'discover': '只读发现局域网中的 Uno Q', 'detect_network': '核对无线设备身份与运动锁',
    'verify_probe': '读取本次启动舵机探测报告',
    'complete': '本次流程完成，冷启动仍待实测',
    'failed': '本次流程失败，请查看检查项', 'incomplete': '流程已结束，所需读回证据尚未齐全',
    'interrupted': '上次流程被服务重启中断，没有自动续跑',
}
STAGE_ALIASES = {'starting': 'prepare', 'done': 'complete', 'finished': 'complete',
                 'flash': 'upload', 'deployment': 'deploy', 'wireless': 'wifi'}
CHECK_KEYS = ('software_tests', 'software_compile', 'usb_readback', 'network_readback', 'probe_readback', 'firmware_readback', 'cold_boot')
CHECK_ALIASES = {'software': 'software_tests', 'compile': 'software_compile',
                 'usb': 'usb_readback', 'network': 'network_readback', 'probe': 'probe_readback'}
ERRORS = {
    'script_exit_failed': '固定部署工具返回失败状态',
    'script_reported_error': '固定部署工具报告错误，原始文本已隐藏',
    'verification_incomplete': '本次所需反馈与程序读回证据尚未齐全',
    'verification_failed': '本次流程有检查项未通过',
    'script_timeout': '固定流程超过执行时限，已停止本次进程',
    'launch_failed': '固定流程未能启动，请检查工具和连接',
    'state_save_failed': '无法保存本次安全状态记录',
    'usb_not_found': '未识别到 USB Uno Q，请检查数据线和连接',
    'wifi_not_available': '电脑当前 Wi-Fi 配置不可用',
    'compile_failed': '主控编译失败', 'upload_failed': '主控烧录失败',
    'deploy_failed': '无线服务安装失败', 'network_unreachable': '无线服务尚不可达',
    'wifi_password_missing': '电脑当前保存的 Wi-Fi 凭据不完整',
    'credential_missing': '设备的受信任连接身份尚未准备好',
    'ssh_failed': '设备的固定 SSH 身份验证未通过',
    'motion_lock_missing': '尚未确认主控新鲜反馈中的运动锁定状态',
    'dependency_failed': '所需部署依赖尚未准备好',
    'package_invalid': '固定只读程序包校验未通过',
    'rollback_failed': '无线更新回退未完成，保持停机并保留检查记录',
    'probe_readback_failed': '本次启动的探测报告尚未读回核对',
    'wireless_update_failed': '无线只读程序更新未完成',
    'provisioning_busy': '另一个部署、更新或发现流程正在执行',
}
MAX_LINE_CHARS = 16384
MAX_QA_BYTES = 32768
PRIVATE_V4_NETWORKS = tuple(ipaddress.ip_network(value) for value in
                            ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))


def safe_network_url(value):
    if not isinstance(value, str) or len(value) > 256:
        return None
    try:
        parsed = urlsplit(value)
        if parsed.scheme not in ('http', 'https') or parsed.username or parsed.password or parsed.query or parsed.fragment:
            return None
        host = parsed.hostname
        if not host or parsed.path not in ('', '/'):
            return None
        try:
            address = ipaddress.ip_address(host)
            allowed = address.is_private and not address.is_loopback and not address.is_unspecified
        except ValueError:
            allowed = bool(re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9-]{0,62}\.local', host))
        if not allowed:
            return None
        port = parsed.port
        if port is not None and not 1 <= port <= 65535:
            return None
        return urlunsplit((parsed.scheme, parsed.netloc, '', '', ''))
    except ValueError:
        return None


def safe_serial(value):
    if isinstance(value, str) and re.fullmatch(r'(?:[A-Fa-f0-9]{6,64}|COM[1-9][0-9]{0,3})', value):
        return value
    return None


def check_value(value):
    if type(value) is bool:
        return 'pass' if value else 'fail'
    if isinstance(value, str):
        return {'passed': 'pass', 'failed': 'fail', 'pending': 'unknown'}.get(value, value) if value in ('pass', 'fail', 'unknown', 'running', 'passed', 'failed', 'pending') else None
    return None


def qa_datetime(value):
    """Accept a dated observation, never a naive/local or non-finite timestamp."""
    if not isinstance(value, str) or len(value) > 48:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
        return parsed.astimezone(timezone.utc) if parsed.tzinfo is not None else None
    except (ValueError, OverflowError):
        return None


def qa_uuid(value):
    if not isinstance(value, str) or len(value) != 36:
        return None
    try:
        parsed = UUID(value)
        return str(parsed) if parsed.int and str(parsed) == value.lower() else None
    except ValueError:
        return None


def qa_network_url(value):
    """Historical board evidence must name the real IPv4 endpoint, not DNS."""
    safe = safe_network_url(value)
    if safe is None:
        return None
    try:
        parsed = urlsplit(safe)
        address = ipaddress.IPv4Address(parsed.hostname)
        if parsed.scheme != 'http' or parsed.port != 9527 or not any(address in item for item in PRIVATE_V4_NETWORKS):
            return None
        return f'http://{address}:9527'
    except (ValueError, TypeError):
        return None


def locked_qa_telemetry(value):
    """Check stored observations with the same conservative 150 ms lock gate."""
    if not isinstance(value, dict) or value.get('mode') != 'read_only':
        return False
    for field in ('host_read_only', 'no_rpc_writes', 'commissioning_motion_locked',
                  'mcu_motion_locked', 'feedback_fresh'):
        if value.get(field) is not True:
            return False
    for field in ('enabled', 'mcu_calibrating', 'mcu_servo_debug'):
        if value.get(field) is not False:
            return False
    age, received, seq, flags = (value.get(field) for field in
                                  ('feedback_age_ms', 'received', 'mcu_seq', 'mcu_flags'))
    return (type(age) in (int, float) and math.isfinite(age) and 0 <= age <= 150
            and type(received) is int and received > 0
            and type(seq) is int and 0 <= seq <= 0xFFFFFFFF
            and type(flags) is int and 0 <= flags <= 0xFF
            and bool(flags & 0x20) and not bool(flags & 0x1A))


class Provisioner:
    def __init__(self, project_root=PROJECT_ROOT, process_factory=subprocess.Popen, timeout_s=2100):
        self.project_root = Path(project_root).resolve()
        self.state_path = self.project_root / 'qa' / 'provisioning-state.json'
        self.process_factory, self.timeout_s = process_factory, timeout_s
        self.lock = threading.RLock()
        self.process = None
        self.state = self._empty_state()
        self._restore()

    @staticmethod
    def _empty_state():
        return {'active': False, 'mode': None, 'stage': 'idle', 'progress': 0,
                'message': STAGES['idle'], 'network_url': None, 'serial': None,
                'checks': {key: 'unknown' for key in CHECK_KEYS}, 'errors': [],
                'cold_boot': 'unknown', 'snapshot_only': False,
                'deployment_started_at': None}

    def _restore(self):
        try:
            if self.state_path.stat().st_size > 32768:
                return
            saved = json.loads(self.state_path.read_text('utf-8'))
            if not isinstance(saved, dict):
                return
            if saved.get('mode') in ('deploy', 'discover', 'wireless_update', 'wireless_calibration_update', 'usb_calibration_install'):
                self.state['mode'] = saved['mode']
            self._apply_packet(saved)
            # These are service-generated dates, deliberately ignored in stdout.
            deployment_started = saved.get('deployment_started_at')
            if deployment_started is None and saved.get('mode') in ('deploy', 'wireless_update', 'wireless_calibration_update', 'usb_calibration_install'):
                deployment_started = saved.get('started_at')
            if qa_datetime(deployment_started) is not None:
                self.state['deployment_started_at'] = deployment_started
            self.state['active'] = False
            self.state['snapshot_only'] = True
            if saved.get('active') is True:
                self.state['stage'], self.state['message'] = 'interrupted', STAGES['interrupted']
            else:
                self.state['message'] = '此前安全状态记录；服务重启未自动部署或重连，需重新读取无线状态'
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass

    def status(self):
        with self.lock:
            snapshot = deepcopy(self.state)
        # QA is separate historical evidence. GET only reads local small files;
        # it never verifies the live board or promotes a script cold_boot value.
        last_known = self._last_known_connection()
        snapshot['last_known_network_url'] = last_known.get('network_url')
        snapshot['last_known_serial'] = last_known.get('serial')
        # A known configured device identity is distinct from a successful live
        # connection. Keep network_url/checks untouched when discovery fails.
        snapshot['serial_source'] = 'job' if safe_serial(snapshot.get('serial')) else 'unknown'
        if snapshot.get('serial') is None and snapshot['last_known_serial'] is not None:
            snapshot['serial'] = snapshot['last_known_serial']
            snapshot['serial_source'] = 'last_known'
        snapshot['cold_boot_evidence'] = self._cold_boot_evidence(snapshot)
        return snapshot

    def _qa_object(self, relative_path):
        try:
            path = (self.project_root / relative_path).resolve()
            if not path.is_relative_to(self.project_root) or path.stat().st_size > MAX_QA_BYTES:
                return None
            with path.open('rb') as stream:
                raw = stream.read(MAX_QA_BYTES + 1)
            if len(raw) > MAX_QA_BYTES:
                return None
            parsed = json.loads(raw.decode('utf-8'))
            return parsed if isinstance(parsed, dict) else None
        except (OSError, ValueError, UnicodeError, TypeError):
            return None

    def _qa_identity(self):
        # Metadata only. Never read private keys or known_hosts contents, and
        # never bypass an invalid primary identity with a legacy fallback.
        identity_relative = 'tools/.private/ssh-identity.json'
        if not (self.project_root / identity_relative).exists():
            identity_relative = '.private/identity.json'
        return self._qa_object(identity_relative)

    def _last_known_connection(self):
        connection, identity = self._qa_object('qa/wireless-connection.json'), self._qa_identity()
        if connection is None or identity is None or connection.get('source') != 'usb_provision':
            return {}
        serial, url = safe_serial(connection.get('serial')), qa_network_url(connection.get('network_url'))
        if (serial is None or serial.startswith('COM') or serial != identity.get('serial')
                or url is None or connection.get('board_ip') != urlsplit(url).hostname):
            return {}
        return {'serial': serial, 'network_url': url}

    def _cold_boot_evidence(self, snapshot):
        evidence = {'verified': False, 'source': 'physical_qa', 'historical': True,
                    'verified_at': None, 'serial': None, 'transaction_id': None,
                    'network_url': None, 'reason': 'not_recorded'}

        def rejected(reason):
            evidence['reason'] = reason
            return evidence

        if snapshot.get('active') is True and snapshot.get('mode') in ('deploy', 'wireless_update', 'wireless_calibration_update', 'usb_calibration_install'):
            return rejected('deployment_active')
        proof = self._qa_object('qa/wireless-coldboot.json')
        if proof is None:
            return evidence
        connection = self._qa_object('qa/wireless-connection.json')
        identity = self._qa_identity()
        if connection is None or identity is None:
            return rejected('binding_missing')
        if (type(proof.get('version')) is not int or proof['version'] != 1
                or proof.get('source') != 'physical_qa'
                or proof.get('connection_source') != 'usb_provision'
                or connection.get('source') != 'usb_provision'):
            return rejected('source_mismatch')
        transaction = qa_uuid(proof.get('transaction_id'))
        if transaction is None or transaction != qa_uuid(connection.get('transaction_id')):
            return rejected('transaction_mismatch')
        serial = safe_serial(proof.get('serial'))
        if (serial is None or serial.startswith('COM')
                or any(item != serial for item in (connection.get('serial'), identity.get('serial')))):
            return rejected('identity_mismatch')
        network_url = qa_network_url(proof.get('network_url'))
        connection_url = qa_network_url(connection.get('network_url'))
        # DHCP and failed discovery change present reachability, not the prior
        # acceptance. Identity/transaction/initial boot below bind this record;
        # both stored addresses must still be safe literal private endpoints.
        if (network_url is None or connection_url is None
                or connection.get('board_ip') != urlsplit(connection_url).hostname):
            return rejected('network_mismatch')
        verified_at = qa_datetime(proof.get('verified_at'))
        connection_at = qa_datetime(connection.get('checked_at_utc'))
        deployment_at = qa_datetime(snapshot.get('deployment_started_at'))
        if snapshot.get('mode') in ('wireless_update', 'wireless_calibration_update', 'usb_calibration_install') and deployment_at is None:
            return rejected('deployment_time_missing')
        now = datetime.now(timezone.utc)
        if (verified_at is None or connection_at is None or verified_at < connection_at
                or (verified_at - now).total_seconds() > 30
                or (deployment_at is not None and verified_at < deployment_at)):
            return rejected('observation_time_invalid')
        if (proof.get('power_source') != 'external_5v3a'
                or proof.get('power_source_confirmed') is not True
                or proof.get('adb_probe_ok') is not True
                or proof.get('adb_device_present') is not False):
            return rejected('power_or_usb_unverified')
        ssh = proof.get('ssh')
        if not isinstance(ssh, dict):
            return rejected('ssh_identity_unverified')
        fingerprint, alias = ssh.get('host_key_fingerprint'), ssh.get('host_key_alias')
        if (ssh.get('pinned') is not True or ssh.get('sys_serial') != serial
                or not isinstance(fingerprint, str)
                or re.fullmatch(r'SHA256:[A-Za-z0-9+/]{43}', fingerprint) is None
                or fingerprint != identity.get('host_key_fingerprint')
                or not isinstance(alias, str) or re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', alias) is None
                or alias != identity.get('host_key_alias')):
            return rejected('ssh_identity_unverified')
        before, after = qa_uuid(proof.get('boot_id_before')), qa_uuid(proof.get('boot_id_after'))
        if before is None or after is None or before == after or before != qa_uuid(connection.get('initial_boot_id')):
            return rejected('boot_change_unverified')
        first, second = proof.get('telemetry_first'), proof.get('telemetry_second')
        if not locked_qa_telemetry(first) or not locked_qa_telemetry(second):
            return rejected('fresh_motion_lock_unverified')
        sequence_delta = (second['mcu_seq'] - first['mcu_seq']) & 0xFFFFFFFF
        if second['received'] <= first['received'] or not 0 < sequence_delta < 0x80000000:
            return rejected('feedback_not_advancing')
        evidence.update(verified=True, verified_at=verified_at.isoformat(), serial=serial,
                        transaction_id=transaction, network_url=network_url, reason='verified')
        return evidence

    def _save(self):
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.state_path.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(self.state, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            temporary.replace(self.state_path)
            return True
        except OSError:
            self._error('state_save_failed')
            return False

    def _error(self, code):
        code = code if isinstance(code, str) and code in ERRORS else 'script_reported_error'
        if code not in self.state['errors']:
            self.state['errors'].append(code)
        self.state['message'] = ERRORS[code]

    def _apply_packet(self, packet):
        if not isinstance(packet, dict):
            return
        if (packet.get('stage')=='complete' and packet.get('not_needed') is True
                and (packet.get('outcome')=='already_current'
                     or self.state.get('mode')=='usb_calibration_install' and packet.get('outcome')=='software_already_installed')
                and packet.get('app_code_version')=='2026-10-13-wireless-calibration-v6'):
            self.state['not_needed']=True
            self.state['outcome']=packet['outcome']
        stage = packet.get('stage')
        stage = STAGE_ALIASES.get(stage, stage) if isinstance(stage, str) else None
        if stage in STAGES:
            self.state['stage'], self.state['message'] = stage, STAGES[stage]
        progress = packet.get('progress')
        if type(progress) in (int, float) and math.isfinite(progress) and 0 <= progress <= 100:
            self.state['progress'] = round(progress, 1)
        if 'network_url' in packet:
            self.state['network_url'] = safe_network_url(packet['network_url'])
        if 'serial' in packet:
            self.state['serial'] = safe_serial(packet['serial'])
        checks = packet.get('checks')
        if isinstance(checks, dict):
            for key, value in checks.items():
                key = CHECK_ALIASES.get(key, key)
                normalized = check_value(value)
                if (key in CHECK_KEYS and key != 'cold_boot' and normalized
                        and not (self.state.get('mode') == 'wireless_update' and key == 'usb_readback')):
                    self.state['checks'][key] = normalized
        errors = packet.get('errors')
        if isinstance(errors, list):
            for value in errors[:20]:
                code = value if isinstance(value, str) else value.get('code') if isinstance(value, dict) else None
                self._error(code)
        if packet.get('stage') == 'failed' and not errors:
            self._error('script_reported_error')
        self.state['checks']['cold_boot'] = self.state['cold_boot'] = 'unknown'
        # Deliberately ignore message, stdout, stderr, keys, password, SSID and
        # every unspecified field, even when a script accidentally prints them.

    def _command(self, mode):
        if mode == 'discover':
            return [str(PYTHON), str(self.project_root / 'tools' / 'discover_wireless_unoq.py')]
        if mode == 'usb_calibration_install':
            script=(self.project_root/'tools/provision_wireless_calibration_usb.py').resolve()
            if not PYTHON.is_file() or not script.is_file() or not script.is_relative_to(self.project_root):
                raise ValueError('USB 首次烧录工具尚未就绪')
            return [str(PYTHON),str(script),'--deploy','--acknowledged','--usb-connected','--servo-power-off','--use-saved-wifi']
        if mode == 'wireless_calibration_update':
            script = (self.project_root / 'tools' / 'deploy_wireless_calibration.py').resolve()
            if not PYTHON.is_file() or not script.is_file() or not script.is_relative_to(self.project_root):
                raise ValueError('固定无线校准更新工具尚未就绪')
            return [str(PYTHON), str(script), '--deploy']
        if mode == 'wireless_update':
            script = (self.project_root / 'tools' / 'deploy_readonly_wifi.py').resolve()
            if not PYTHON.is_file() or not script.is_file() or not script.is_relative_to(self.project_root):
                raise ValueError('固定 Python 或无线更新脚本不存在，未开始更新')
            return [str(PYTHON), str(script), '--deploy']
        if mode not in ('deploy', 'discover'):
            raise ValueError('未知固定流程')
        filename = 'provision_unoq.ps1' if mode == 'deploy' else 'discover_unoq.ps1'
        script = (self.project_root / 'tools' / filename).resolve()
        if not PWSH.is_file() or not script.is_file() or not script.is_relative_to(self.project_root):
            raise ValueError('固定 PowerShell 或项目脚本不存在，未开始部署')
        args = [str(PWSH), '-NoLogo', '-NoProfile', '-NonInteractive', '-File', str(script)]
        if mode == 'deploy':
            args.extend(['-WifiSource', 'SavedCurrent'])
        return args

    def start(self, action, discover=False, wireless_update=False, calibration_update=False,calibration_usb=False):
        if not isinstance(action, dict):
            raise ValueError('部署请求应为 JSON 对象')
        if sum(bool(value) for value in (discover, wireless_update, calibration_update,calibration_usb)) > 1:
            raise ValueError('无线发现与更新不能同时启动')
        if (wireless_update or calibration_update) and action.get('acknowledged') is not True:
            raise ValueError('确认无线更新后才启动本次更新')
        if calibration_usb and any(action.get(name) is not True for name in ('acknowledged','use_saved_wifi','usb_connected','servo_power_off')):
            raise ValueError('首次烧录需确认主控 USB 已接、舵机断电并使用本机已保存 Wi-Fi')
        if not discover and not wireless_update and not calibration_update and not calibration_usb and (action.get('acknowledged') is not True or action.get('use_saved_wifi') is not True):
            raise ValueError('先确认 USB 已连接、舵机电源已断开，并明确使用电脑当前保存的 Wi-Fi')
        mode = 'usb_calibration_install' if calibration_usb else 'wireless_calibration_update' if calibration_update else 'wireless_update' if wireless_update else 'discover' if discover else 'deploy'
        args = self._command(mode)  # No request-controlled command/path/arguments.
        with self.lock:
            if self.state['active']:
                raise ValueError('已有部署或发现流程正在执行，请等待当前流程结束')
            previous_deployment = self.state.get('deployment_started_at')
            self.state = self._empty_state()
            self.state.update(active=True, mode=mode, stage='discover' if discover else 'prepare',
                              message=STAGES['discover' if discover else 'prepare'],
                              started_at=datetime.now(timezone.utc).isoformat())
            self.state['deployment_started_at'] = (self.state['started_at'] if mode in ('deploy', 'wireless_update', 'wireless_calibration_update','usb_calibration_install')
                                                   else previous_deployment)
            if not self._save():
                self.state['active'], self.state['stage'] = False, 'failed'
                raise ValueError('无法保存安全状态记录，本次流程未启动')
            threading.Thread(target=self._run, args=(args, mode), daemon=True).start()
            return self.status()

    @staticmethod
    def _read_lines(stream, packets):
        try:
            while True:
                line = stream.readline(MAX_LINE_CHARS + 1)
                if not line:
                    break
                if len(line) > MAX_LINE_CHARS:
                    while line and not line.endswith('\n'):
                        line = stream.readline(MAX_LINE_CHARS + 1)
                    continue
                packets.put(line)
        except Exception:
            pass
        finally:
            packets.put(None)

    def _run(self, args, mode):
        process, reader = None, None
        try:
            process = self.process_factory(args, cwd=str(self.project_root), shell=False,
                                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                           stderr=subprocess.DEVNULL, text=True, encoding='utf-8', errors='replace',
                                           creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            self.process = process
            packets = queue.Queue(maxsize=64)
            reader = threading.Thread(target=self._read_lines, args=(process.stdout, packets), daemon=True)
            reader.start()
            deadline, ended = time.monotonic() + self.timeout_s, False
            while not ended:
                if time.monotonic() >= deadline:
                    process.kill()  # Only the exact fixed process launched above.
                    with self.lock:
                        self._error('script_timeout')
                    break
                try:
                    line = packets.get(timeout=.1)
                except queue.Empty:
                    continue
                if line is None:
                    ended = True
                    continue
                try:
                    packet = json.loads(line.lstrip('\ufeff'))
                except (ValueError, TypeError, json.JSONDecodeError):
                    continue
                with self.lock:
                    self._apply_packet(packet)
                    self._save()
            try:
                exit_code = process.wait(timeout=max(.1, min(5, deadline - time.monotonic())))
            except subprocess.TimeoutExpired:
                process.kill()
                exit_code = None
                with self.lock:
                    self._error('script_timeout')
            with self.lock:
                if type(exit_code) is int:
                    self.state['exit_code'] = exit_code
                if exit_code not in (0, None):
                    self._error('script_exit_failed')
                if 'fail' in self.state['checks'].values():
                    self._error('verification_failed')
                software_only=(mode=='usb_calibration_install' and self.state.get('not_needed') is True
                               and self.state.get('outcome')=='software_already_installed')
                required = (('firmware_readback',) if software_only
                            else ('network_readback','firmware_readback') if self.state.get('not_needed') is True and mode in ('wireless_calibration_update','usb_calibration_install')
                            else ('software_compile', 'network_readback', 'firmware_readback') if mode in ('wireless_calibration_update','usb_calibration_install')
                            else ('software_compile', 'network_readback', 'probe_readback') if mode == 'wireless_update'
                            else ('network_readback',) if mode == 'discover' else ('usb_readback', 'network_readback'))
                verified = all(self.state['checks'][key] == 'pass' for key in required) and (software_only or bool(self.state['network_url']))
                if exit_code == 0 and verified and not self.state['errors']:
                    self.state.update(stage='complete', progress=100, message=STAGES['complete'])
                elif not self.state['errors']:
                    self._error('verification_incomplete')
                    self.state['stage'] = 'incomplete'
                    self.state['progress'] = min(self.state['progress'], 99)
                else:
                    self.state['stage'] = 'failed'
                    self.state['message'] = ERRORS[self.state['errors'][-1]]
                    self.state['progress'] = min(self.state['progress'], 99)
        except Exception:
            # Exception text may contain stdout or credentials: never retain it.
            with self.lock:
                self._error('launch_failed')
                self.state['stage'] = 'failed'
        finally:
            try:
                if process and process.poll() is None:
                    process.kill()
                if reader:
                    reader.join(timeout=.2)
                # Do not block the HTTP job if a child retained the output pipe.
                if process and process.stdout and (not reader or not reader.is_alive()):
                    process.stdout.close()
            except Exception:
                pass
            with self.lock:
                self.process = None
                self.state['active'] = False
                self.state['checks']['cold_boot'] = self.state['cold_boot'] = 'unknown'
                self.state['finished_at'] = datetime.now(timezone.utc).isoformat()
                self._save()
