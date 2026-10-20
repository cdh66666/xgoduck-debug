import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from device_config import DEVICE_SERIAL
"""Read-only discovery: cached real IP, then official Arduino mDNS events.

No subnet/port scan, OS hostname lookup, environment proxy, redirect, password,
credential export, upload, configuration write or actuator command is used.
Arduino protocol: https://arduino.github.io/arduino-cli/latest/pluggable-discovery-specification/
"""
import ipaddress
import os,shutil
import hashlib
import json
import math
import queue
import re
import shlex
import subprocess
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parent.parent
MDNS_EXE = Path(os.environ.get('XGODUCK_MDNS') or shutil.which('mdns-discovery') or next(iter(sorted((Path(os.environ.get('LOCALAPPDATA', str(Path.home()/'AppData/Local')))/'Arduino15/packages/builtin/tools/mdns-discovery').glob('*/mdns-discovery.exe'),reverse=True)),Path('mdns-discovery')))
SSH_EXE = Path(shutil.which('ssh') or 'ssh')
GIT_SSH_EXE = Path('C:/Program Files/Git/usr/bin/ssh.exe')
PRIVATE_NETWORKS = tuple(ipaddress.ip_network(value) for value in ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))
MAX_RESPONSE = 128 * 1024
FRESH_LIMIT_MS = 150
SSH_READ_COMMAND = 'hostname; cat /sys/devices/soc0/serial_number'
HTTP_TIMEOUT_S = 3
SSH_CONNECT_TIMEOUT_S = 5
SSH_TIMEOUT_S = 15
DISCOVERY_BUDGET_S = 45
MDNS_WINDOW_S = 18
MAX_CANDIDATE_ATTEMPTS = 3
RETRY_INTERVAL_S = 3
CALIBRATION_VERSION = '2026-10-13-wireless-calibration-v6'
CALIBRATION_SCHEMA = 'wireless-calibration-v2'
CALIBRATION_CONTRACTS = {
    '2026-10-13-wireless-calibration-v1': ('wireless-calibration-v1', 2),
    '2026-10-13-wireless-calibration-v2': ('wireless-calibration-v2', 3),
    '2026-10-13-wireless-calibration-v3': ('wireless-calibration-v2', 3),
    '2026-10-13-wireless-calibration-v4': ('wireless-calibration-v2', 3),
    '2026-10-13-wireless-calibration-v5': ('wireless-calibration-v2', 3),
    CALIBRATION_VERSION: (CALIBRATION_SCHEMA, 3),
    '2026-10-15-wireless-calibration-v7': (CALIBRATION_SCHEMA, 3),
    '2026-10-17-wireless-calibration-v8-passive-id10': ('wireless-calibration-v2', 3),
}
CALIBRATION_SERIAL = DEVICE_SERIAL
SAFE_ERROR_CODES = frozenset(('credential_missing', 'ssh_failed', 'motion_lock_missing',
                              'dependency_failed', 'network_unreachable'))
SAFE_DIAGNOSTIC_CODES = SAFE_ERROR_CODES | frozenset(('http_not_ready', 'http_rejected',
    'http_transport_failed', 'http_invalid_response', 'feedback_not_ready',
    'unsafe_motion_state', 'ssh_timeout', 'discovery_budget_exhausted'))


class DiscoveryError(Exception):
    def __init__(self, code, retryable=False, reason=None):
        self.code = code if code in SAFE_ERROR_CODES else 'network_unreachable'
        self.retryable = retryable is True
        self.reason = reason if reason in SAFE_DIAGNOSTIC_CODES else self.code
        super().__init__(self.code)


def private_ipv4(value):
    try:
        address = ipaddress.ip_address(value)
        return str(address) if address.version == 4 and any(address in network for network in PRIVATE_NETWORKS) else None
    except (ValueError, TypeError):
        return None


def literal_url(value):
    if not isinstance(value, str) or len(value) > 256:
        return None
    try:
        parsed = urlsplit(value)
        ip = private_ipv4(parsed.hostname)
        if parsed.scheme != 'http' or not ip or parsed.username or parsed.password or parsed.query or parsed.fragment:
            return None
        if parsed.port != 9527 or parsed.path not in ('', '/'):
            return None
        return f'http://{ip}:9527'
    except ValueError:
        return None


def read_small_json(path):
    if not path.is_file() or path.stat().st_size > 65536:
        return None
    try:
        value = json.loads(path.read_text('utf-8-sig'))
        return value if isinstance(value, dict) else None
    except (OSError, ValueError, TypeError):
        return None


def cached_candidates(root):
    result = []
    filenames=('wireless-connection.json','provisioning-state.json','discovery-last.json')
    # A recent verified discovery must win after DHCP assigns the board a new IP.
    filenames=sorted(filenames,key=lambda name:(Path(root)/'qa'/name).stat().st_mtime if (Path(root)/'qa'/name).is_file() else 0,reverse=True)
    for filename in filenames:
        data = read_small_json(Path(root) / 'qa' / filename)
        if not data:
            continue
        url = literal_url(data.get('network_url'))
        ip = private_ipv4(data.get('board_ip'))
        if not url and ip:
            url = f'http://{ip}:9527'
        if url and url not in result:
            result.append(url)
    return result


def load_identity(root):
    root = Path(root).resolve()
    for relative in ('tools/.private/ssh-identity.json', '.private/identity.json'):
        identity_path = root / relative
        data = read_small_json(identity_path)
        if not data:
            continue
        serial = data.get('serial')
        alias = data.get('host_key_alias')
        if not isinstance(serial, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', serial):
            raise DiscoveryError('credential_missing')
        if not isinstance(alias, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,100}', alias):
            raise DiscoveryError('credential_missing')
        paths = {}
        for name in ('key_path', 'known_hosts_path'):
            value = data.get(name)
            if not isinstance(value, str) or not value:
                raise DiscoveryError('credential_missing')
            target = Path(value)
            if not target.is_absolute():
                target = identity_path.parent / target
            target = target.resolve()
            if not target.is_relative_to(identity_path.parent.resolve()) or not target.is_file():
                raise DiscoveryError('credential_missing')
            paths[name] = target
        # Only metadata/path existence is read; private key contents are never read.
        return {'serial': serial, 'host_key_alias': alias, **paths}
    raise DiscoveryError('credential_missing')


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def fetch_status(url, timeout=HTTP_TIMEOUT_S):
    url = literal_url(url)
    if not url:
        raise DiscoveryError('network_unreachable')
    request = Request(url + '/api/status', headers={'Accept': 'application/json'})
    try:
        with build_opener(ProxyHandler({}), NoRedirect()).open(request, timeout=min(HTTP_TIMEOUT_S, timeout)) as response:
            raw = response.read(MAX_RESPONSE + 1)
    except HTTPError as error:
        raise DiscoveryError('network_unreachable', retryable=error.code in (502, 503, 504),
                             reason='http_not_ready' if error.code in (502, 503, 504) else 'http_rejected') from None
    except (TimeoutError, URLError, OSError):
        raise DiscoveryError('network_unreachable', retryable=True, reason='http_transport_failed') from None
    if len(raw) > MAX_RESPONSE:
        raise DiscoveryError('network_unreachable')
    try:
        result = json.loads(raw)
    except (ValueError, UnicodeError, TypeError):
        raise DiscoveryError('network_unreachable', reason='http_invalid_response') from None
    if not isinstance(result, dict):
        raise DiscoveryError('network_unreachable')
    return result


def feedback_not_ready(status):
    """Only telemetry startup/staleness may wait; unsafe motion flags never do."""
    if (not isinstance(status, dict) or status.get('host_read_only') is not True
            or status.get('mode') != 'read_only' or status.get('no_rpc_writes') is not True):
        return False
    for name in ('enabled', 'mcu_enabled', 'mcu_calibrating', 'mcu_servo_debug'):
        if status.get(name) is not None and status.get(name) is not False:
            return False
    flags = status.get('mcu_flags')
    if flags is not None and (type(flags) is not int or not 0 <= flags <= 0xFF
                              or flags & 0x1A or not flags & 0x20):
        return False
    if status.get('mcu_motion_locked') is False:
        return False
    age = status.get('state_age_ms', status.get('feedback_age_ms'))
    received = status.get('received_frames', status.get('received'))
    if age is not None and (type(age) not in (int, float) or not math.isfinite(age) or age < 0):
        return False
    if received is not None and (type(received) is not int or received < 0):
        return False
    return (age is None or age > FRESH_LIMIT_MS or received in (None, 0)
            or status.get('feedback_fresh') is False)


def status_is_safe(status):
    if not isinstance(status, dict):
        return False
    if any(status.get(name) is not True for name in ('host_read_only', 'commissioning_motion_locked', 'mcu_motion_locked')):
        return False
    if status.get('mode') not in ('read_only', None) or status.get('no_rpc_writes') is False:
        return False
    if status.get('enabled', status.get('mcu_enabled')) is not False:
        return False
    if any(status.get(name, False) is not False for name in ('mcu_calibrating', 'mcu_servo_debug')):
        return False
    age = status.get('state_age_ms', status.get('feedback_age_ms'))
    if type(age) not in (float, int) or not math.isfinite(age) or not 0 <= age <= FRESH_LIMIT_MS:
        return False
    if status.get('feedback_fresh', True) is not True:
        return False
    flags = status.get('mcu_flags')
    if type(flags) is not int or not 0 <= flags <= 0xFF or not flags & 0x20 or flags & 0x1A:
        return False
    received = status.get('received_frames', status.get('received'))
    sequence = status.get('mcu_seq')
    return type(received) is int and received > 0 and type(sequence) is int and 0 <= sequence <= 0xFFFFFFFF


def calibration_envelope_matches(status):
    """Fixed app/DWR identity only. This does not certify off or motion readiness."""
    if (not isinstance(status, dict) or status.get('mode') != 'wireless_calibration'
            or status.get('app_code_version') not in CALIBRATION_CONTRACTS
            or status.get('host_read_only') is not False or status.get('no_rpc_writes') is not False
            or status.get('enabled') is not False):
        return False
    instance = status.get('app_instance_id')
    calibration = status.get('calibration')
    expected_schema, expected_revision = CALIBRATION_CONTRACTS[status['app_code_version']]
    if (not isinstance(instance, str) or not re.fullmatch(r'[0-9a-f]{32}', instance)
            or not isinstance(calibration, dict)
            or calibration.get('schema') != expected_schema
            or calibration.get('version') != status['app_code_version']
            or calibration.get('supported') is not True
            or calibration.get('snapshot_protocol') != 'DWR1'):
        return False
    for value in (status, calibration):
        if (type(value.get('snapshot_version')) is not int or value['snapshot_version'] != expected_revision
                or type(value.get('mcu_contract_revision')) is not int or value['mcu_contract_revision'] != expected_revision
                or value.get('sticky_supported') is not True):
            return False
    boot, sequence = calibration.get('boot_id'), calibration.get('sequence')
    return (type(boot) is int and 0 < boot <= 0xFFFFFFFF
            and type(sequence) is int and 0 <= sequence <= 0xFFFFFFFF)


def calibration_status_is_readable(status):
    """Fresh DWR v2 plus policy lock; ARMED/FAULT/off uncertainty remain readable."""
    if not calibration_envelope_matches(status):
        return False
    calibration = status['calibration']
    age = calibration.get('age_ms')
    return (status.get('policy_locked') is True and calibration.get('policy_locked') is True
            and calibration.get('fresh') is True
            and type(age) in (int, float) and math.isfinite(age) and 0 <= age < FRESH_LIMIT_MS)


def calibration_feedback_not_ready(status):
    # Only an otherwise exact app/DWR envelope may wait for new feedback.
    # A fresh packet reporting policy unlocked is always a hard failure.
    if not calibration_envelope_matches(status):
        return False
    calibration = status['calibration']
    age = calibration.get('age_ms')
    if type(age) not in (int, float) or not math.isfinite(age) or age < 0:
        return False
    return calibration.get('fresh') is False or age >= FRESH_LIMIT_MS


def verify_calibration_status_pair(first, second):
    if not calibration_status_is_readable(first) or not calibration_status_is_readable(second):
        waiting = all(calibration_status_is_readable(value) or calibration_feedback_not_ready(value)
                      for value in (first, second))
        raise DiscoveryError('motion_lock_missing', retryable=waiting,
                             reason='feedback_not_ready' if waiting else 'unsafe_motion_state')
    a, b = first['calibration'], second['calibration']
    delta = (b['sequence'] - a['sequence']) & 0xFFFFFFFF
    if (first['app_code_version'] != second['app_code_version']
            or first['app_instance_id'] != second['app_instance_id'] or a['boot_id'] != b['boot_id']
            or not 0 < delta < 0x80000000):
        raise DiscoveryError('motion_lock_missing')
    return {'schema': 'wireless-calibration-connection-v1', 'scope': 'identity_and_live_dwr_only',
            'mode': 'wireless_calibration', 'app_code_version': second['app_code_version'],
            'app_instance_id': second['app_instance_id'], 'calibration_schema': b['schema'],
            'snapshot_protocol': 'DWR1', 'snapshot_version': b['snapshot_version'],
            'mcu_contract_revision': b['mcu_contract_revision'],
            'sticky_supported': True, 'boot_id': b['boot_id'], 'sequence': b['sequence'],
            'feedback_age_ms': b['age_ms'], 'policy_locked': True, 'enabled': False,
            'counters_advanced': True, 'motion_authorized': False,
            'physical_off_verified': False}


def verify_status_pair(first, second, allow_calibration=False):
    if allow_calibration is True and any(isinstance(value, dict) and value.get('mode') == 'wireless_calibration'
                                        for value in (first, second)):
        return verify_calibration_status_pair(first, second)
    if not status_is_safe(first) or not status_is_safe(second):
        waiting = all(status_is_safe(value) or feedback_not_ready(value) for value in (first, second))
        raise DiscoveryError('motion_lock_missing', retryable=waiting,
                             reason='feedback_not_ready' if waiting else 'unsafe_motion_state')
    received_a = first.get('received_frames', first.get('received'))
    received_b = second.get('received_frames', second.get('received'))
    delta = (second['mcu_seq'] - first['mcu_seq']) & 0xFFFFFFFF
    if received_b <= received_a or not 0 < delta < 0x80000000:
        raise DiscoveryError('motion_lock_missing')
    return {'host_read_only': True, 'mcu_motion_locked': True, 'commissioning_motion_locked': True,
            'enabled': False, 'feedback_age_ms': second.get('state_age_ms', second.get('feedback_age_ms')),
            'received': received_b, 'mcu_seq': second['mcu_seq'], 'counters_advanced': True}


def ssh_transport(root=ROOT):
    """Honor the already approved executable without weakening host-key checks."""
    path = Path(root) / 'tools' / '.private' / 'ssh-client.json'
    if path.is_symlink() or (hasattr(path, 'is_junction') and path.is_junction()):
        raise DiscoveryError('credential_missing')
    record = read_small_json(path)
    if path.exists() and not record:
        raise DiscoveryError('credential_missing')
    if record:
        if (record.get('backend') != 'git-openssh' or not GIT_SSH_EXE.is_file()
                or hashlib.sha256(GIT_SSH_EXE.read_bytes()).hexdigest() != record.get('executable_sha256')):
            raise DiscoveryError('ssh_failed')
        return GIT_SSH_EXE, '/dev/null'
    return SSH_EXE, 'NUL'


def verify_ssh_identity(ip, identity, runner=subprocess.run, timeout=SSH_TIMEOUT_S, root=ROOT):
    ip = private_ipv4(ip)
    executable, null_config = ssh_transport(root)
    if not ip or not executable.is_file():
        raise DiscoveryError('ssh_failed')
    args = [str(executable), '-F', null_config, '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
            '-o', 'IdentityAgent=none', '-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no',
            '-o', 'VerifyHostKeyDNS=no', '-o', 'CheckHostIP=no', '-o', 'PasswordAuthentication=no',
            '-o', 'KbdInteractiveAuthentication=no', '-o', f'ConnectTimeout={min(SSH_CONNECT_TIMEOUT_S, max(1, math.ceil(timeout)))}',
            '-o', 'ConnectionAttempts=1', '-o', 'GlobalKnownHostsFile=' + null_config,
            '-o', 'UserKnownHostsFile=' + str(identity['known_hosts_path']),
            '-o', 'HostKeyAlias=' + identity['host_key_alias'], '-i', str(identity['key_path']),
            'arduino@' + ip, SSH_READ_COMMAND]
    # This command only reads hostname and the previously verified board serial.
    try:
        result = runner(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                        text=True, encoding='utf-8', errors='replace', timeout=min(SSH_TIMEOUT_S, timeout), shell=False,
                        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    except subprocess.TimeoutExpired:
        raise DiscoveryError('ssh_failed', retryable=True, reason='ssh_timeout') from None
    except Exception:
        raise DiscoveryError('ssh_failed') from None
    if result.returncode != 0 or not isinstance(result.stdout, str) or len(result.stdout) > 4096:
        raise DiscoveryError('ssh_failed')
    lines = result.stdout.replace('\x00', '').strip().splitlines()
    if len(lines) != 2 or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,100}', lines[0]):
        raise DiscoveryError('ssh_failed')
    if lines[1].strip().lower() != identity['serial'].lower():
        raise DiscoveryError('ssh_failed')
    return {'ssh_pin_verified': True, 'hostname': lines[0], 'sys_serial': lines[1].strip()}


PINNED_STATUS_PAIR_SCRIPT = r'''
import json, os, socket, time
from pathlib import Path
from urllib.request import Request, ProxyHandler, HTTPRedirectHandler, build_opener
class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs): return None
def read_status():
    request = Request('http://127.0.0.1:9527/api/status', headers={'Accept':'application/json'}, method='GET')
    with build_opener(ProxyHandler({}), NoRedirect()).open(request, timeout=2) as reply:
        raw = reply.read(131073)
    if len(raw) > 131072: raise ValueError('oversize status')
    value = json.loads(raw)
    if not isinstance(value, dict): raise ValueError('invalid status')
    return value
connection = os.environ.get('SSH_CONNECTION', '').split()
if len(connection) != 4: raise ValueError('missing ssh connection')
serial = Path('/sys/devices/soc0/serial_number').read_text().strip()
first = read_status()
time.sleep(.25)
second = read_status()
print(json.dumps({'serial':serial, 'hostname':socket.gethostname(), 'ssh_server_ip':connection[2],
                  'first':first, 'second':second}, separators=(',', ':'), allow_nan=False))
'''


def pinned_status_pair(ip, identity, root=ROOT, runner=subprocess.run, timeout=12):
    """One pinned SSH session, only serial/address reads and two loopback GETs."""
    ip = private_ipv4(ip)
    executable, null_config = ssh_transport(root)
    if not ip or not executable.is_file() or identity.get('serial') != CALIBRATION_SERIAL:
        raise DiscoveryError('ssh_failed')
    args = [str(executable), '-F', null_config, '-T', '-o', 'BatchMode=yes',
        '-o', 'IdentitiesOnly=yes', '-o', 'IdentityAgent=none', '-o', 'StrictHostKeyChecking=yes',
        '-o', 'UpdateHostKeys=no', '-o', 'VerifyHostKeyDNS=no', '-o', 'CheckHostIP=no',
        '-o', 'PasswordAuthentication=no', '-o', 'KbdInteractiveAuthentication=no',
        '-o', 'ConnectTimeout=' + str(min(5, max(1, math.ceil(timeout)))),
        '-o', 'ConnectionAttempts=1', '-o', 'GlobalKnownHostsFile=' + null_config,
        '-o', 'UserKnownHostsFile=' + str(identity['known_hosts_path']),
        '-o', 'HostKeyAlias=' + identity['host_key_alias'], '-i', str(identity['key_path']),
        'arduino@' + ip, 'python3 -c ' + shlex.quote(PINNED_STATUS_PAIR_SCRIPT)]
    try:
        reply = runner(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, encoding='utf-8', errors='replace', timeout=min(12, timeout), shell=False,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    except subprocess.TimeoutExpired:
        raise DiscoveryError('ssh_failed', retryable=True, reason='ssh_timeout') from None
    except Exception:
        raise DiscoveryError('ssh_failed') from None
    if reply.returncode != 0 or not isinstance(reply.stdout, str) or len(reply.stdout) > 2 * MAX_RESPONSE + 2048:
        raise DiscoveryError('ssh_failed')
    try:
        data = json.loads(reply.stdout)
    except (ValueError, TypeError):
        raise DiscoveryError('ssh_failed') from None
    if (not isinstance(data, dict) or data.get('serial') != identity['serial']
            or data.get('ssh_server_ip') != ip or not isinstance(data.get('hostname'), str)
            or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,100}', data['hostname'])):
        raise DiscoveryError('ssh_failed')
    evidence = verify_status_pair(data.get('first'), data.get('second'), allow_calibration=True)
    return {**evidence, 'ssh_pin_verified':True, 'sys_serial':data['serial'], 'hostname':data['hostname'],
            'ssh_server_ip':ip, 'transport':'pinned_ssh_loopback', 'direct_http_verified':False}


def discover_pinned(root=ROOT, probe=pinned_status_pair, browse=None, clock=time.monotonic,
                    identity_loader=load_identity, should_continue=lambda:True, emit=lambda _:None):
    """Discover this bound controller even when its HTTP service is loopback-only.

    Return verified evidence without publishing; caller owns update interlock and
    atomic cache commit. No HTTP request is sent directly to an announced IP.
    """
    root = Path(root).resolve()
    identity = identity_loader(root)
    if identity.get('serial') != CALIBRATION_SERIAL:
        raise DiscoveryError('ssh_failed')
    deadline = clock() + DISCOVERY_BUDGET_S
    def checkpoint():
        if not should_continue() or clock() >= deadline:
            raise DiscoveryError('network_unreachable')
    def attempt(urls):
        for value in urls[:8]:
            checkpoint()
            url = literal_url(value)
            if not url: continue
            ip = urlsplit(url).hostname
            try:
                proof = probe(ip, identity, root=root, timeout=min(12, deadline-clock()))
                checkpoint()
                if (not isinstance(proof, dict) or proof.get('ssh_pin_verified') is not True
                        or proof.get('sys_serial') != CALIBRATION_SERIAL or proof.get('ssh_server_ip') != ip
                        or proof.get('counters_advanced') is not True):
                    raise DiscoveryError('ssh_failed')
                calibration = proof.get('mode') == 'wireless_calibration'
                result = {'stage':'complete','progress':100,'network_url':url,'board_ip':ip,
                    'serial':CALIBRATION_SERIAL,'checked_at':time.time(),
                    'checks':{'network_readback':'pass','cold_boot':'unknown'},
                    ('connection_evidence' if calibration else 'discovery_evidence'):proof}
                if calibration:result['discovered_mode']='wireless_calibration'
                emit(result)
                return result
            except DiscoveryError:
                checkpoint()
        return None
    checkpoint()
    cached = cached_candidates(root)
    found = attempt(cached[:1])
    if found:return found
    checkpoint()
    announced = (browse or mdns_candidates)(seconds=min(MDNS_WINDOW_S, deadline-clock()))
    checkpoint()
    candidates = list(dict.fromkeys([url for value in announced[:8] if (url := literal_url(value))] + cached))
    found = attempt(candidates)
    if found:return found
    raise DiscoveryError('network_unreachable')


def event_candidates(event):
    if not isinstance(event, dict) or event.get('eventType') not in ('add', 'list'):
        return []
    ports = [event.get('port')] if event['eventType'] == 'add' else event.get('ports', [])
    if not isinstance(ports, list):
        return []
    result = []

    def addresses(value):
        if isinstance(value, str):
            ip = private_ipv4(value)
            if ip:
                result.append(f'http://{ip}:9527')
        elif isinstance(value, list):
            if len(value) == 4 and all(type(item) is int and 0 <= item <= 255 for item in value):
                addresses('.'.join(str(item) for item in value))
            else:
                for item in value[:16]:
                    addresses(item)
        elif isinstance(value, dict):
            for key in ('address', 'ip', 'ipv4', 'IPv4', 'addresses', 'ips', 'AddrV4', 'value', 'ipAddress'):
                if key in value:
                    addresses(value[key])

    for port in ports[:16]:
        if isinstance(port, dict) and port.get('protocol') == 'network':
            addresses(port.get('address'))
            addresses(port.get('addressData'))
    return list(dict.fromkeys(result))[:8]


def mdns_candidates(seconds=MDNS_WINDOW_S, process_factory=subprocess.Popen, clock=time.monotonic):
    if not MDNS_EXE.is_file():
        raise DiscoveryError('dependency_failed')
    process, reader = None, None
    chunks = queue.Queue(maxsize=32)

    def read_chunks():
        try:
            while True:
                line = process.stdout.readline(16384)
                if not line:
                    break
                chunks.put(line)
        finally:
            chunks.put(None)

    results, buffer = [], ''
    decoder = json.JSONDecoder()
    try:
        process = process_factory([str(MDNS_EXE)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, text=True, encoding='utf-8', errors='replace',
                                  shell=False, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        reader = threading.Thread(target=read_chunks, daemon=True)
        reader.start()
        process.stdin.write('HELLO 1 "XGO Duck read-only discovery"\nSTART_SYNC\n')
        process.stdin.flush()
        deadline = clock() + min(max(seconds, .1), MDNS_WINDOW_S)
        while clock() < deadline:
            try:
                chunk = chunks.get(timeout=min(.2, max(.01, deadline - clock())))
            except queue.Empty:
                continue
            if chunk is None:
                break
            buffer = (buffer + chunk).lstrip('\ufeff \r\n\t')
            if len(buffer) > MAX_RESPONSE:
                buffer = ''
                continue
            while buffer:
                try:
                    event, end = decoder.raw_decode(buffer)
                except ValueError:
                    break
                results.extend(event_candidates(event))
                buffer = buffer[end:].lstrip()
        return list(dict.fromkeys(results))[:8]
    except DiscoveryError:
        raise
    except Exception:
        raise DiscoveryError('network_unreachable') from None
    finally:
        if process:
            try:
                process.stdin.write('STOP\nQUIT\n')
                process.stdin.flush()
                process.wait(timeout=1)
            except Exception:
                process.kill()  # Exact official discovery process created here.
            if reader:
                reader.join(timeout=.2)
            if process.stdin:
                process.stdin.close()
            if process.stdout and (not reader or not reader.is_alive()):
                process.stdout.close()


def discover(root=ROOT, fetch=fetch_status, ssh_verify=verify_ssh_identity, browse=mdns_candidates,
             sleeper=time.sleep, clock=time.monotonic,
             emit=lambda packet: print(json.dumps(packet, ensure_ascii=False), flush=True),
             allow_calibration=False, persist=True, should_continue=lambda: True,
             identity_loader=load_identity):
    root = Path(root).resolve()
    identity = identity_loader(root)
    def checkpoint():
        if not should_continue():
            raise DiscoveryError('network_unreachable')
    checkpoint()
    emit({'stage': 'discover', 'progress': 10, 'serial': identity['serial'],
          'checks': {'network_readback': 'running', 'cold_boot': 'unknown'}})
    attempts, next_try, blocked = {}, {}, set()
    last_error = DiscoveryError('network_unreachable')
    deadline = clock() + DISCOVERY_BUDGET_S

    def remember(error):
        nonlocal last_error
        priority = {'network_unreachable': 0, 'dependency_failed': 1, 'ssh_failed': 2, 'motion_lock_missing': 3}
        if priority.get(error.code, 0) >= priority.get(last_error.code, 0):
            last_error = error

    def diagnostic(step, started, error=None, attempt=0):
        emit({'stage': 'discover', 'serial': identity['serial'], 'diagnostic': {
            'step': step, 'errorcode': error.reason if error else 'none',
            'duration_ms': round(max(0, clock() - started) * 1000), 'attempt': attempt}})

    def try_urls(urls):
        for url in urls:
            checkpoint()
            url = literal_url(url)
            remaining = deadline - clock()
            if (url is None or url in blocked or attempts.get(url, 0) >= MAX_CANDIDATE_ATTEMPTS
                    or clock() < next_try.get(url, 0) or remaining <= 0):
                continue
            attempts[url] = attempts.get(url, 0) + 1
            started, step = clock(), 'http_feedback_pair'
            try:
                first = fetch(url, timeout=min(HTTP_TIMEOUT_S, remaining))
                if not status_is_safe(first) and not (allow_calibration is True and calibration_status_is_readable(first)):
                    verify_status_pair(first, first, allow_calibration=allow_calibration)
                remaining = deadline - clock()
                if remaining <= .25:
                    raise DiscoveryError('network_unreachable', reason='discovery_budget_exhausted')
                sleeper(.25)
                checkpoint()
                remaining = deadline - clock()
                if remaining <= 0:
                    raise DiscoveryError('network_unreachable', reason='discovery_budget_exhausted')
                second = fetch(url, timeout=min(HTTP_TIMEOUT_S, remaining))
                evidence = verify_status_pair(first, second, allow_calibration=allow_calibration)
                calibration_connection = evidence.get('mode') == 'wireless_calibration'
                if calibration_connection and identity['serial'] != CALIBRATION_SERIAL:
                    raise DiscoveryError('ssh_failed')
                diagnostic(step, started, attempt=attempts[url])
                started, step, remaining = clock(), 'pinned_ssh', deadline - clock()
                if remaining <= 0:
                    raise DiscoveryError('network_unreachable', reason='discovery_budget_exhausted')
                ssh = ssh_verify(urlsplit(url).hostname, identity, timeout=min(SSH_TIMEOUT_S, remaining))
                checkpoint()
                diagnostic(step, started, attempt=attempts[url])
                result = {'stage': 'complete', 'progress': 100, 'network_url': url,
                          'board_ip': urlsplit(url).hostname, 'serial': identity['serial'],
                          'checks': {'network_readback': 'pass', 'cold_boot': 'unknown'},
                          ('connection_evidence' if calibration_connection else 'discovery_evidence'): {**evidence, **ssh},
                          'checked_at': time.time()}
                if calibration_connection:
                    result['discovered_mode'] = 'wireless_calibration'
                if persist:
                    (root / 'qa').mkdir(parents=True, exist_ok=True)
                    temporary = root / 'qa' / 'discovery-last.json.tmp'
                    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
                    checkpoint()
                    temporary.replace(root / 'qa' / 'discovery-last.json')
                emit(result)
                return result
            except DiscoveryError as error:
                remember(error)
                diagnostic(step, started, error, attempts[url])
                if error.retryable:
                    next_try[url] = clock() + RETRY_INTERVAL_S
                else:
                    blocked.add(url)
            except (TimeoutError, URLError, OSError):
                error = DiscoveryError('network_unreachable', retryable=True, reason='http_transport_failed')
                remember(error)
                diagnostic(step, started, error, attempts[url])
                next_try[url] = clock() + RETRY_INTERVAL_S
            except Exception:
                error = DiscoveryError('network_unreachable')
                remember(error)
                diagnostic(step, started, error, attempts[url])
                blocked.add(url)
        return None

    candidates = cached_candidates(root)
    cached = try_urls(candidates)
    if cached:
        return cached
    emit({'stage': 'discover', 'progress': 45, 'serial': identity['serial']})
    if deadline - clock() > 0:
        checkpoint()
        started = clock()
        try:
            announced = browse(seconds=min(MDNS_WINDOW_S, deadline - clock()))
            # New DHCP announcements get the remaining budget before stale caches.
            candidates = list(dict.fromkeys([url for value in announced[:8]
                                            if (url := literal_url(value)) is not None] + candidates))
            diagnostic('mdns', started)
        except DiscoveryError as error:
            remember(error)
            diagnostic('mdns', started, error)
        except Exception:
            error = DiscoveryError('network_unreachable')
            remember(error)
            diagnostic('mdns', started, error)
    while clock() < deadline:
        found = try_urls(candidates)
        if found:
            return found
        eligible = [url for url in candidates if url not in blocked
                    and attempts.get(url, 0) < MAX_CANDIDATE_ATTEMPTS]
        if not eligible:
            break
        delay = max(0, min(next_try.get(url, 0) for url in eligible) - clock())
        remaining = deadline - clock()
        if remaining <= 0:
            break
        sleeper(min(max(.01, delay), RETRY_INTERVAL_S, remaining))
    raise last_error


def main(allow_calibration=False):
    try:
        if allow_calibration:
            # The UI's explicit "find controller" button must use the same
            # topology as automatic recovery: 9527 is not a LAN listener.
            from deploy_readonly_wifi import strict_identity
            allowed = lambda: not (ROOT / 'tools/.private/wireless-calibration-update-in-progress.json').exists()
            result = discover_pinned(root=ROOT, identity_loader=strict_identity, should_continue=allowed)
            if not allowed():
                raise DiscoveryError('network_unreachable')
            (ROOT / 'qa').mkdir(parents=True, exist_ok=True)
            temporary = ROOT / 'qa/discovery-last.manual.tmp'
            temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            if not allowed():
                raise DiscoveryError('network_unreachable')
            temporary.replace(ROOT / 'qa/discovery-last.json')
            print(json.dumps(result, ensure_ascii=False), flush=True)
        else:
            discover(allow_calibration=False)
        return 0
    except DiscoveryError as error:
        code = error.code if error.code in ('credential_missing', 'ssh_failed', 'motion_lock_missing',
                                           'dependency_failed', 'network_unreachable') else 'network_unreachable'
    except Exception:
        code = 'network_unreachable'
    print(json.dumps({'stage': 'failed', 'progress': 0,
                      'checks': {'network_readback': 'fail', 'cold_boot': 'unknown'}, 'errors': [code]}), flush=True)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
