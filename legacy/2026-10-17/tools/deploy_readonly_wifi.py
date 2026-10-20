"""Reviewed, fixed-target Uno Q update. Default invocation is offline --plan.

Only --deploy permits network access or board changes. No arbitrary command,
remote path, archive path, username, key path, ID or motion target is accepted.
Desktop tests do not establish MCU compilation, upload or physical acceptance.
"""
import argparse
import base64
import hashlib
import json
import math
import os
import queue
import re
import shlex
import stat
import subprocess
import threading
import time
import zipfile
from pathlib import Path
from urllib.parse import urlsplit

import discover_unoq as discovery

ROOT = Path(__file__).resolve().parent.parent
PACKAGE_NAME = 'downloads/xgoduck-commissioning-usb.zip'
MANIFEST_NAME = 'downloads/package-check.json'
APP_CODE_VERSION = '2026-10-12-probe2'
REMOTE_APP = '/home/arduino/ArduinoApps/xgoduck-commissioning'
REMOTE_STAGE_ROOT = '/home/arduino/.cache/xgoduck-readonly-update'
ALLOWLIST = (
    'app.yaml', 'assets/index.html', 'python/main.py', 'python/wire.py',
    'python/servo_probe.py', 'python/requirements.txt', 'sketch/duck_config.h',
    'sketch/protocol.h', 'sketch/scs_bus.h', 'sketch/qmi8658.h',
    'sketch/uart_fifo.h', 'sketch/servo_probe.h', 'sketch/sketch.ino',
    'sketch/sketch.yaml', 'tools/reuse-toolchain.py', 'tools/prepare-bridge.py',
    'tools/verify-elf.py', 'tools/router-2m.sh', 'tools/patches/parse_packet.txt',
    'tools/patches/duck_msgpack_frame.h',
)
MAX_ARCHIVE = 16 * 1024 * 1024
MAX_EXPANDED = 32 * 1024 * 1024
MAX_FILE = 4 * 1024 * 1024
SAFE_CODES = frozenset((
    'package_missing', 'package_rejected', 'manifest_invalid', 'credential_missing',
    'network_unreachable', 'ssh_failed', 'motion_lock_missing', 'serial_mismatch',
    'path_guard_failed', 'dependency_missing', 'stage_failed', 'compile_failed',
    'elf_failed', 'app_stop_failed', 'app_start_failed', 'verification_failed',
    'rollback_failed', 'transport_lost', 'update_failed',
    'time_budget_exhausted',
))
STAGES = frozenset(('package_plan', 'discover', 'preflight', 'stage', 'dependencies',
    'compile', 'elf_check', 'backup', 'stop', 'merge', 'start', 'verify', 'rollback',
    'complete', 'failed'))
CHECK_NAMES = frozenset(('package_review', 'software_compile', 'elf_check',
    'network_readback', 'probe_readback', 'wireless_update', 'cold_boot', 'usb_readback'))
CHECK_VALUES = frozenset(('unknown', 'running', 'pass', 'fail', 'pending'))
STAGE_MAP = {'package_plan': 'prepare', 'preflight': 'prepare', 'stage': 'prepare',
             'dependencies': 'prepare', 'elf_check': 'compile', 'backup': 'deploy',
             'stop': 'deploy', 'merge': 'deploy', 'start': 'upload',
             'verify': 'verify_network', 'rollback': 'deploy'}
ERROR_MAP = {'package_missing': 'package_invalid', 'package_rejected': 'package_invalid',
    'manifest_invalid': 'package_invalid', 'dependency_missing': 'dependency_failed',
    'path_guard_failed': 'deploy_failed', 'stage_failed': 'deploy_failed',
    'elf_failed': 'compile_failed', 'app_stop_failed': 'deploy_failed',
    'app_start_failed': 'upload_failed', 'serial_mismatch': 'ssh_failed',
    'transport_lost': 'network_unreachable', 'update_failed': 'deploy_failed',
    'time_budget_exhausted': 'script_timeout'}
STAGES = STAGES | frozenset(STAGE_MAP.values())
SAFE_CODES = SAFE_CODES | frozenset(ERROR_MAP.values())


class UpdateError(Exception):
    def __init__(self, code):
        self.code = code if code in SAFE_CODES else 'update_failed'
        super().__init__(self.code)


def _fixed_file(root, relative):
    root = Path(root).resolve(strict=True)
    path = root / relative
    for parent in (path, *path.parents):
        if parent == root:
            break
        if parent.is_symlink() or (hasattr(parent, 'is_junction') and parent.is_junction()):
            raise UpdateError('package_rejected')
    if not path.is_file() or not path.resolve(strict=True).is_relative_to(root):
        raise UpdateError('package_missing')
    return path


def review_package(root=ROOT):
    """Read the two fixed local review artifacts; no identity/network operations."""
    archive_path = _fixed_file(root, PACKAGE_NAME)
    manifest_path = _fixed_file(root, MANIFEST_NAME)
    if archive_path.stat().st_size > MAX_ARCHIVE or manifest_path.stat().st_size > 65536:
        raise UpdateError('package_rejected')
    try:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8-sig'))
    except (OSError, UnicodeError, ValueError):
        raise UpdateError('manifest_invalid') from None
    if (not isinstance(manifest, dict) or manifest.get('files') != list(ALLOWLIST)
            or manifest.get('app_code_version') != APP_CODE_VERSION
            or manifest.get('entrypoint') != 'ReadOnlyTelemetry'
            or manifest.get('mcu_lock_marker') != '0x20'
            or manifest.get('physical_acceptance') != 'pending'
            or manifest.get('real_mcu_compile') != 'pending_board'
            or not isinstance(manifest.get('files_sha256'), dict)
            or set(manifest['files_sha256']) != set(ALLOWLIST)
            or type(manifest.get('size')) is not int
            or not re.fullmatch(r'[0-9a-f]{64}', str(manifest.get('sha256', '')))):
        raise UpdateError('manifest_invalid')
    raw = archive_path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if len(raw) != manifest['size'] or digest != manifest['sha256']:
        raise UpdateError('package_rejected')
    files, total = {}, 0
    try:
        with zipfile.ZipFile(archive_path) as archive:
            entries = archive.infolist()
            if len(entries) != len(ALLOWLIST):
                raise UpdateError('package_rejected')
            for entry in entries:
                mode = entry.external_attr >> 16
                kind = stat.S_IFMT(mode)
                if (entry.filename not in ALLOWLIST or entry.filename in files
                        or entry.is_dir() or kind not in (0, stat.S_IFREG)
                        or entry.flag_bits & 1 or entry.file_size > MAX_FILE
                        or entry.file_size < 0
                        or entry.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED)):
                    raise UpdateError('package_rejected')
                total += entry.file_size
                if total > MAX_EXPANDED:
                    raise UpdateError('package_rejected')
                data = archive.read(entry)
                if (len(data) != entry.file_size
                        or hashlib.sha256(data).hexdigest() != manifest['files_sha256'][entry.filename]):
                    raise UpdateError('package_rejected')
                files[entry.filename] = data
    except UpdateError:
        raise
    except (OSError, ValueError, zipfile.BadZipFile, RuntimeError, EOFError):
        raise UpdateError('package_rejected') from None
    if set(files) != set(ALLOWLIST):
        raise UpdateError('package_rejected')
    return {'sha256': digest, 'size': len(raw), 'expanded_size': total, 'files': files}


def locked_status(status):
    return (discovery.status_is_safe(status) and status.get('mode') == 'read_only'
            and status.get('no_rpc_writes') is True
            and status.get('mcu_enabled_seen', False) is False)


def strict_identity(root):
    """Reject redirected metadata/key paths; never read private key contents."""
    root = Path(root).resolve(strict=True)
    for relative in ('tools/.private/ssh-identity.json', '.private/identity.json'):
        path = root / relative
        for part in (path, *path.parents):
            if part.is_symlink() or (hasattr(part, 'is_junction') and part.is_junction()):
                raise UpdateError('credential_missing')
            if part == root:
                break
        data = discovery.read_small_json(path)
        if not data:
            continue
        private = path.parent
        for name in ('key_path', 'known_hosts_path'):
            value = data.get(name)
            if not isinstance(value, str):
                raise UpdateError('credential_missing')
            target = Path(value)
            if not target.is_absolute():
                target = private / target
            for part in (target, *target.parents, path):
                if part.is_symlink() or (hasattr(part, 'is_junction') and part.is_junction()):
                    raise UpdateError('credential_missing')
                if part == root:
                    break
        try:
            return discovery.load_identity(root)
        except discovery.DiscoveryError:
            raise UpdateError('credential_missing') from None
    raise UpdateError('credential_missing')


def verify_locked_pair(first, second):
    if not locked_status(first) or not locked_status(second):
        raise UpdateError('motion_lock_missing')
    try:
        return discovery.verify_status_pair(first, second)
    except discovery.DiscoveryError:
        raise UpdateError('motion_lock_missing') from None


def verify_new_status(first, second, previous_instance=None):
    verify_locked_pair(first, second)
    expected = {(op, attempt) for op in (1, 2, 0x82) for attempt in (1, 2, 3)}
    for status in (first, second):
        probe = status.get('servo_probe')
        instance = status.get('app_instance_id')
        if (status.get('app_code_version') != APP_CODE_VERSION or not isinstance(probe, dict)
                or not isinstance(instance, str) or not re.fullmatch(r'[0-9a-f]{32}', instance)
                or instance == previous_instance
                or instance != second.get('app_instance_id')):
            raise UpdateError('verification_failed')
        reports = probe.get('reports')
        if (probe.get('scope') != 'startup_id10' or probe.get('complete') is not True
                or probe.get('startup_only') is not True
                or probe.get('firmware_evidence') != 'received_reports'
                or type(probe.get('received_reports')) is not int
                or probe['received_reports'] < 9 or not isinstance(reports, list)
                or len(reports) != 9):
            raise UpdateError('verification_failed')
        contexts = set()
        for report in reports:
            if (not isinstance(report, dict) or report.get('magic') != 'DQP1'
                    or report.get('version') != 1 or report.get('target_id') != 10
                    or type(report.get('opcode')) is not int
                    or type(report.get('attempt')) is not int):
                raise UpdateError('verification_failed')
            contexts.add((report['opcode'], report['attempt']))
        if contexts != expected:
            raise UpdateError('verification_failed')
    # No position/servo-mask requirement: a validated no-RX report is useful
    # deployment evidence, not physical servo communication acceptance.
    return {'software_compile': 'pass', 'elf_check': 'pass',
            'network_readback': 'pass', 'probe_readback': 'pass', 'wireless_update': 'pass'}


def redact_packet(packet):
    if (not isinstance(packet, dict) or not isinstance(packet.get('stage'), str)
            or packet['stage'] not in STAGES):
        return None
    result = {'stage': STAGE_MAP.get(packet['stage'], packet['stage'])}
    if type(packet.get('progress')) is int and 0 <= packet['progress'] <= 100:
        result['progress'] = packet['progress']
    serial = packet.get('serial')
    if isinstance(serial, str) and re.fullmatch(r'[A-Za-z0-9_-]{1,64}', serial):
        result['serial'] = serial
    url = discovery.literal_url(packet.get('network_url'))
    if url:
        result['network_url'] = url
    checks = packet.get('checks')
    if isinstance(checks, dict):
        result['checks'] = {('probe_readback' if key == 'probe_reports' else key): value
                           for key, value in checks.items()
                           if (key in CHECK_NAMES or key == 'probe_reports')
                           and isinstance(value, str) and value in CHECK_VALUES}
    errors = packet.get('errors')
    if isinstance(errors, list):
        result['errors'] = list(dict.fromkeys(ERROR_MAP.get(code, code)
                                              for code in errors
                                              if isinstance(code, str) and code in SAFE_CODES))
    if packet.get('rollback') in ('not_needed', 'restored', 'failed', 'pending'):
        result['rollback'] = packet['rollback']
    return result


def emit_packet(packet):
    safe = redact_packet(packet)
    if safe is not None:
        print(json.dumps(safe, ensure_ascii=False), flush=True)


# Fixed code only; payload is data on stdin, never interpolated into this code
# or a remote shell command. The helper verifies serial and live lock again.
REMOTE_HELPER = r'''
import base64, hashlib, json, math, os, re, shutil, stat, subprocess, sys, tempfile, time
from pathlib import Path
from urllib.request import Request, ProxyHandler, HTTPRedirectHandler, build_opener

TARGET = Path('/home/arduino/ArduinoApps/xgoduck-commissioning')
STAGING = Path('/home/arduino/.cache/xgoduck-readonly-update')
HOME = Path('/home/arduino')
VERSION = '2026-10-12-probe2'
ALLOW = (
 'app.yaml','assets/index.html','python/main.py','python/wire.py','python/servo_probe.py',
 'python/requirements.txt','sketch/duck_config.h','sketch/protocol.h','sketch/scs_bus.h',
 'sketch/qmi8658.h','sketch/uart_fifo.h','sketch/servo_probe.h','sketch/sketch.ino',
 'sketch/sketch.yaml','tools/reuse-toolchain.py','tools/prepare-bridge.py',
 'tools/verify-elf.py','tools/router-2m.sh','tools/patches/parse_packet.txt',
 'tools/patches/duck_msgpack_frame.h')
PINS = (('Arduino_RouterBridge','0.4.3'),('Arduino_RPClite','0.3.1'),('MsgPack','0.4.2'),
        ('ArxContainer','0.7.0'),('ArxTypeTraits','0.3.2'),('DebugLog','0.8.4'))
SAFE = {'serial_mismatch','path_guard_failed','dependency_missing','stage_failed',
        'compile_failed','elf_failed','app_stop_failed','app_start_failed',
        'verification_failed','motion_lock_missing','rollback_failed','update_failed','time_budget_exhausted'}
UPDATE_DEADLINE=None
RECOVERY_RESERVE=1000

class Failure(Exception):
 def __init__(self,code): self.code=code if code in SAFE else 'update_failed'

class NoRedirect(HTTPRedirectHandler):
 def redirect_request(self,*args,**kwargs): return None

def budget(requested):
 remaining=UPDATE_DEADLINE-time.monotonic() if UPDATE_DEADLINE is not None else requested
 if remaining<=1: raise Failure('time_budget_exhausted')
 return min(requested,remaining)

def packet(stage,progress,serial,checks=None,errors=None,rollback=None):
 value={'stage':stage,'progress':progress,'serial':serial}
 if checks is not None: value['checks']=checks
 if errors is not None: value['errors']=errors
 if rollback is not None: value['rollback']=rollback
 print(json.dumps(value),flush=True)

def guarded(path,base=HOME):
 path=Path(path)
 if not path.is_absolute() or not path.is_relative_to(base): raise Failure('path_guard_failed')
 for part in (path,*path.parents):
  if part.is_symlink(): raise Failure('path_guard_failed')
  if part==base: break
 if path.resolve(strict=False)!=path: raise Failure('path_guard_failed')
 return path

def guarded_tree(path):
 path=guarded(path)
 # Only the fixed app cache and this updater's fixed-name staging/backup cache
 # roots may be moved. Atomic directory rename never follows links inside it.
 if path!=TARGET/'.cache':
  if (path.parent.parent!=STAGING or not (
   (re.fullmatch(r'stage-[a-z0-9_]{8}',path.parent.name) and path.name=='.cache') or
   (re.fullmatch(r'backup-[a-z0-9_]{8}',path.parent.name) and path.name in ('old-cache','failed-new-cache')))):
   raise Failure('path_guard_failed')
 if not path.is_dir(): raise Failure('path_guard_failed')
 for parent,dirs,files in os.walk(path,followlinks=False):
  retained=[]
  for name in dirs+files:
   item=Path(parent)/name
   mode=item.lstat().st_mode
   # Venv links such as lib64 -> lib and python -> /usr/local/bin/python are
   # retained as directory entries. Never resolve, read, or traverse the target.
   if stat.S_ISLNK(mode): continue
   if not stat.S_ISDIR(mode) and not stat.S_ISREG(mode):
    raise Failure('path_guard_failed')
   if name in dirs: retained.append(name)
  dirs[:]=retained

def atomic(path,data):
 path=guarded(path)
 guarded(path.parent)
 path.parent.mkdir(parents=True,exist_ok=True)
 fd,name=tempfile.mkstemp(prefix='.duck-update-',dir=str(path.parent))
 temporary=guarded(Path(name))
 try:
  with os.fdopen(fd,'wb') as out:
   out.write(data); out.flush(); os.fsync(out.fileno())
  os.chmod(temporary,0o600)
  os.replace(temporary,path)
 finally:
  if temporary.exists(): temporary.unlink()

def fetch():
 try:
  with build_opener(ProxyHandler({}),NoRedirect()).open(
       Request('http://127.0.0.1:9527/api/status',headers={'Accept':'application/json'}),timeout=budget(3)) as r:
   raw=r.read(131073)
  if len(raw)>131072: raise Failure('verification_failed')
  value=json.loads(raw)
  if not isinstance(value,dict): raise Failure('verification_failed')
  return value
 except Failure: raise
 except Exception: raise Failure('verification_failed') from None

def locked(s):
 if not isinstance(s,dict): return False
 if any(s.get(k) is not True for k in ('host_read_only','no_rpc_writes',
        'commissioning_motion_locked','mcu_motion_locked','feedback_fresh')): return False
 if s.get('mode')!='read_only' or s.get('enabled') is not False: return False
 if any(s.get(k,False) is not False for k in ('mcu_enabled','mcu_enabled_seen',
        'mcu_calibrating','mcu_servo_debug')): return False
 flags=s.get('mcu_flags'); age=s.get('state_age_ms',s.get('feedback_age_ms'))
 seq=s.get('mcu_seq'); received=s.get('received_frames',s.get('received'))
 return (type(flags) is int and 0<=flags<=255 and flags&0x20 and not flags&0x1A
  and type(age) in (int,float) and math.isfinite(age) and 0<=age<=150
  and type(seq) is int and 0<=seq<=0xFFFFFFFF and type(received) is int and received>0)

def pair(a,b):
 if not locked(a) or not locked(b): raise Failure('motion_lock_missing')
 delta=(b['mcu_seq']-a['mcu_seq'])&0xFFFFFFFF
 if not 0<delta<0x80000000 or b.get('received_frames',b.get('received'))<=a.get('received_frames',a.get('received')):
  raise Failure('motion_lock_missing')

def probes(s):
 p=s.get('servo_probe'); expected={(op,n) for op in (1,2,130) for n in (1,2,3)}
 if s.get('app_code_version')!=VERSION or not isinstance(p,dict): return False
 reports=p.get('reports')
 if (p.get('scope')!='startup_id10' or p.get('complete') is not True
  or p.get('startup_only') is not True or p.get('firmware_evidence')!='received_reports'
  or type(p.get('received_reports')) is not int or p['received_reports']<9
  or not isinstance(reports,list) or len(reports)!=9): return False
 contexts=set()
 for r in reports:
  if (not isinstance(r,dict) or r.get('magic')!='DQP1' or r.get('version')!=1
   or r.get('target_id')!=10 or type(r.get('opcode')) is not int or type(r.get('attempt')) is not int): return False
  contexts.add((r['opcode'],r['attempt']))
 return contexts==expected

def run(args,cwd,log,timeout,code):
 # No shell, sudo, network configuration or download commands. Output never
 # reaches stdout; the fixed stage log is private and contains compiler output.
 env=os.environ.copy(); env['TMPDIR']='/tmp'
 try:
  with guarded(log).open('ab') as out:
   os.chmod(log,0o600)
   result=subprocess.run(args,cwd=cwd,stdout=out,stderr=out,stdin=subprocess.DEVNULL,
                         timeout=budget(timeout),check=False,shell=False,env=env)
  if result.returncode: raise Failure(code)
 except Failure: raise
 except Exception: raise Failure(code) from None

def dependencies(files):
 cache=guarded(HOME/'.arduino15')
 internal=guarded(cache/'internal')
 dependency='zephyr:arm-zephyr-eabi@1.0.1'
 uid=dependency.replace(':','_').replace('@','_')+'_'+hashlib.sha256(dependency.encode()).hexdigest()[:16]
 profile_tool=internal/uid
 # Arduino CLI normally extracts the exact SDK into its isolated-profile UID.
 # Earlier provisioning could instead link that UID directly to the already
 # installed fixed packages SDK. Accept only these two reviewed layouts.
 if profile_tool.is_symlink():
  tool=guarded(cache/'packages/zephyr/tools/arm-zephyr-eabi/1.0.1')
  if (not profile_tool.exists() or Path(os.readlink(profile_tool))!=tool
   or profile_tool.resolve(strict=True)!=tool):
   raise Failure('dependency_missing')
 else:
  tool=guarded(profile_tool)
 if not tool.is_dir():
  raise Failure('dependency_missing')
 for binary in ('arm-zephyr-eabi-g++','arm-zephyr-eabi-readelf'):
  if not guarded(tool/'bin'/binary).is_file(): raise Failure('dependency_missing')
 libraries={}
 for name,version in PINS:
  choices=list(internal.glob(name+'_'+version+'_*/'+name+'/library.properties'))
  if len(choices)!=1: raise Failure('dependency_missing')
  guarded(choices[0])
  libraries[name]=choices[0].parent
 # The isolated profile core must already be resolved. A packages-only core
 # is insufficient: compile must not silently install a missing profile core.
 candidates=list(internal.glob('arduino_zephyr_1.0.0_*/platform.txt'))
 if len(candidates)!=1: raise Failure('dependency_missing')
 for candidate in candidates: guarded(candidate)
 if shutil.which('arduino-cli') is None or shutil.which('arduino-app-cli') is None:
  raise Failure('dependency_missing')
 # Wireless updates never execute prepare-bridge.py or repair a shared cache.
 # Verify the exact already-installed bounded decoder and stack patch read-only.
 try:
  bridge=guarded(libraries['Arduino_RouterBridge']/'src/bridge.h').read_text(encoding='utf-8')
  decoder=guarded(libraries['Arduino_RPClite']/'src/decoder.h').read_text(encoding='utf-8')
  frame=guarded(libraries['Arduino_RPClite']/'src/duck_msgpack_frame.h').read_text(encoding='utf-8')
  expected_parse=files['tools/patches/parse_packet.txt'].decode('utf-8').replace('\r\n','\n')
  expected_frame=files['tools/patches/duck_msgpack_frame.h'].decode('utf-8').replace('\r\n','\n')
  start=decoder.index('    void parse_packet(){')
  end=decoder.index('    bool packet_incoming()',start)
  if ('#define UPDATE_THREAD_STACK_SIZE    4096' not in bridge
   or 'DUCK_BOUNDED_FRAMING' not in decoder or '#include "duck_msgpack_frame.h"' not in decoder
   or decoder[start:end]!=expected_parse or frame!=expected_frame):
   raise Failure('dependency_missing')
 except Failure: raise
 except Exception: raise Failure('dependency_missing') from None

def upload_evidence(log):
 # This file belongs to THIS app-start invocation, never a historical QA log.
 # A host process restart alone cannot prove that the MCU loaded the new ELF.
 path=guarded(log)
 if not path.is_file() or path.stat().st_size>1024*1024: raise Failure('app_start_failed')
 text=path.read_text(encoding='utf-8',errors='replace')
 markers=('arduino:zephyr:unoq','Open On-Chip Debugger',
          '/tmp/remoteocd/sketch.ino.elf-zsk.bin','Progress[sketch updated]',
          'started successfully')
 if not all(marker in text for marker in markers): raise Failure('app_start_failed')

def rollback(target,backup,old_files,cache_moved,new_cache,log,old_elf_hash):
 try:
  # Stop only the fixed app before restoring source/cache. A failed start may
  # already have left it stopped; accept only explicit benign stop messages.
  env=os.environ.copy(); env['TMPDIR']='/tmp'
  stopped=subprocess.run(['arduino-app-cli','app','stop',str(target)],cwd=target,
   stdout=subprocess.PIPE,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,
   timeout=budget(60),check=False,shell=False,env=env)
  if stopped.returncode and (len(stopped.stdout)>16384 or not re.search(
   r'not running|already stopped|app.*not found|no app.*running',stopped.stdout.decode('utf-8','replace'),re.I)):
   return 'failed'
  for relative in ALLOW:
   dst=guarded(target/relative)
   if relative in old_files: atomic(dst,(guarded(backup/'source'/relative)).read_bytes())
   elif dst.exists():
    parked=guarded(backup/'introduced'/relative)
    parked.parent.mkdir(parents=True,exist_ok=True); os.replace(dst,parked)
  if new_cache and (target/'.cache').exists():
   guarded_tree(target/'.cache'); os.replace(target/'.cache',guarded(backup/'failed-new-cache'))
  if cache_moved:
   guarded_tree(backup/'old-cache'); os.replace(backup/'old-cache',guarded(target/'.cache'))
  restored_elf=guarded(target/'.cache/sketch/sketch.ino.elf')
  if not restored_elf.is_file() or hashlib.sha256(restored_elf.read_bytes()).hexdigest()!=old_elf_hash:
   return 'failed'
  rollback_log=guarded(backup/'rollback-app-start.log')
  run(['arduino-app-cli','app','start',str(target)],target,rollback_log,240,'rollback_failed')
  # File restoration plus a locked old host cannot establish MCU rollback.
  # Require upload evidence from THIS recovery invocation as well.
  upload_evidence(rollback_log)
  run(['python3',str(target/'tools/verify-elf.py'),str(restored_elf)],target,log,60,'rollback_failed')
  a=fetch(); time.sleep(.3); b=fetch(); pair(a,b)
  return 'restored'
 except Exception: return 'failed'

def main():
 global UPDATE_DEADLINE
 UPDATE_DEADLINE=time.monotonic()+1800
 serial='unknown'; stopped=False; cache_moved=False; new_cache=False
 backup=None; stage=None; old_files=set(); log=None; old_elf_hash=None
 checks={'package_review':'pass','software_compile':'unknown','elf_check':'unknown',
         'network_readback':'unknown','probe_reports':'unknown','wireless_update':'running',
         'usb_readback':'unknown','cold_boot':'unknown'}
 try:
  raw=sys.stdin.buffer.read(48*1024*1024+1)
  if len(raw)>48*1024*1024: raise Failure('stage_failed')
  payload=json.loads(raw)
  serial=payload.get('serial','')
  if not isinstance(serial,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',serial): raise Failure('serial_mismatch')
  actual=Path('/sys/devices/soc0/serial_number').read_text().strip()
  if actual.lower()!=serial.lower(): raise Failure('serial_mismatch')
  data=payload.get('files')
  if (payload.get('app_code_version')!=VERSION or not isinstance(data,dict)
   or set(data)!=set(ALLOW) or not re.fullmatch(r'[0-9a-f]{64}',str(payload.get('package_sha256','')))):
   raise Failure('stage_failed')
  decoded={}; total=0
  for name in ALLOW:
   item=data[name]
   if not isinstance(item,dict) or set(item)!= {'base64','sha256'}: raise Failure('stage_failed')
   content=base64.b64decode(item['base64'],validate=True)
   total+=len(content)
   if len(content)>4*1024*1024 or total>32*1024*1024 or hashlib.sha256(content).hexdigest()!=item['sha256']:
    raise Failure('stage_failed')
   decoded[name]=content
  guarded(TARGET); guarded(STAGING)
  if not TARGET.is_dir(): raise Failure('path_guard_failed')
  a=fetch(); time.sleep(.3); b=fetch(); pair(a,b)
  previous_instance=b.get('app_instance_id')
  host_previous=payload.get('previous_instance')
  if host_previous is not None and host_previous!=previous_instance: raise Failure('verification_failed')
  checks['network_readback']='pass'
  packet('preflight',15,serial,checks)
  dependencies(decoded); packet('dependencies',20,serial,checks)
  STAGING.mkdir(parents=True,exist_ok=True)
  stage=guarded(Path(tempfile.mkdtemp(prefix='stage-',dir=STAGING)))
  log=guarded(stage/'update.log')
  for name in ALLOW: atomic(stage/name,decoded[name])
  packet('stage',25,serial,checks)
  compile_budget=UPDATE_DEADLINE-time.monotonic()-RECOVERY_RESERVE
  if compile_budget<=1: raise Failure('time_budget_exhausted')
  checks['software_compile']='running'; packet('compile',35,serial,checks)
  run(['arduino-cli','compile','--profile','default','--build-path',str(stage/'.cache/sketch'),'--jobs','2'],
      stage/'sketch',log,min(1200,compile_budget),'compile_failed')
  checks['software_compile']='pass'; packet('elf_check',55,serial,checks)
  elf=guarded(stage/'.cache/sketch/sketch.ino.elf')
  if not elf.is_file(): raise Failure('elf_failed')
  run(['python3',str(stage/'tools/verify-elf.py'),str(elf)],stage,log,60,'elf_failed')
  checks['elf_check']='pass'
  # Recheck live motion lock immediately before the first app lifecycle action.
  a=fetch(); time.sleep(.3); b=fetch(); pair(a,b)
  backup=guarded(Path(tempfile.mkdtemp(prefix='backup-',dir=STAGING)))
  for name in ALLOW:
   src=guarded(TARGET/name)
   if src.exists():
    if not src.is_file(): raise Failure('path_guard_failed')
    atomic(backup/'source'/name,src.read_bytes()); old_files.add(name)
  if (TARGET/'.cache').exists(): guarded_tree(TARGET/'.cache')
  old_elf=guarded(TARGET/'.cache/sketch/sketch.ino.elf')
  if not old_elf.is_file(): raise Failure('dependency_missing')
  old_elf_hash=hashlib.sha256(old_elf.read_bytes()).hexdigest()
  guarded_tree(stage/'.cache')
  packet('backup',65,serial,checks)
  if UPDATE_DEADLINE-time.monotonic()<RECOVERY_RESERVE:
   raise Failure('time_budget_exhausted')
  # A stop may already have taken effect even if the CLI times out/fails.
  # Recovery is required from the FIRST lifecycle attempt onward.
  stopped=True
  run(['arduino-app-cli','app','stop',str(TARGET)],TARGET,log,60,'app_stop_failed')
  packet('stop',70,serial,checks)
  if (TARGET/'.cache').exists():
   os.replace(guarded(TARGET/'.cache'),guarded(backup/'old-cache')); cache_moved=True
  # The prior cache remains in backup. No recursive delete occurs on any path.
  new_cache=True
  # Both paths are fixed under /home/arduino and already checked. Atomic rename
  # keeps the stop window short; cross-filesystem failure triggers recovery.
  os.replace(guarded(stage/'.cache'),guarded(TARGET/'.cache'))
  for name in ALLOW: atomic(TARGET/name,decoded[name])
  packet('merge',75,serial,checks)
  start_log=guarded(stage/'app-start.log')
  run(['arduino-app-cli','app','start',str(TARGET)],TARGET,start_log,240,'app_start_failed')
  upload_evidence(start_log)
  # App CLI may rebuild after source paths change. Check its final target ELF,
  # too; failure requires rollback rather than inheriting the staged ELF pass.
  final_elf=guarded(TARGET/'.cache/sketch/sketch.ino.elf')
  run(['python3',str(TARGET/'tools/verify-elf.py'),str(final_elf)],TARGET,log,60,'elf_failed')
  packet('verify',90,serial,checks)
  checks['network_readback']='running'; checks['probe_reports']='running'
  deadline=min(time.monotonic()+90,UPDATE_DEADLINE); first=None
  while time.monotonic()<deadline:
   try:
    current=fetch()
    if not locked(current):
     if current.get('enabled') is True or (type(current.get('mcu_flags')) is int and current['mcu_flags']&0x1A):
      raise Failure('motion_lock_missing')
     first=None
    elif (probes(current) and isinstance(current.get('app_instance_id'),str)
     and re.fullmatch(r'[0-9a-f]{32}',current['app_instance_id'])
     and current['app_instance_id']!=previous_instance):
     if first is not None and first.get('app_instance_id')!=current['app_instance_id']:
      first=None
     if first is not None: pair(first,current); break
     first=current
    else: first=None
   except Failure as failure:
    if failure.code=='motion_lock_missing': raise
    first=None
   time.sleep(.5)
  else: raise Failure('verification_failed')
  checks.update(network_readback='pass',probe_reports='pass',wireless_update='pass')
  packet('complete',100,serial,checks,rollback='not_needed')
  return 0
 except Exception as error:
  code=error.code if isinstance(error,Failure) else 'update_failed'
  if checks['software_compile']=='running': checks['software_compile']='fail'
  if code=='elf_failed': checks['elf_check']='fail'
  if checks['network_readback']=='running': checks['network_readback']='fail'
  if checks['probe_reports']=='running': checks['probe_reports']='fail'
  checks['wireless_update']='fail'
  restored='not_needed'
  if stopped and backup is not None:
   packet('rollback',95,serial,checks,rollback='pending')
   restored=rollback(TARGET,backup,old_files,cache_moved,new_cache,log,old_elf_hash)
  errors=[code]+(['rollback_failed'] if restored=='failed' else [])
  packet('failed',0,serial,checks,errors=errors,rollback=restored)
  return 1

if __name__=='__main__': raise SystemExit(main())
'''


def ssh_args(ip, identity):
    ip = discovery.private_ipv4(ip)
    if not ip:
        raise UpdateError('network_unreachable')
    return [str(discovery.SSH_EXE), '-F', 'NUL', '-o', 'BatchMode=yes',
        '-o', 'IdentitiesOnly=yes', '-o', 'IdentityAgent=none',
        '-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no',
        '-o', 'VerifyHostKeyDNS=no', '-o', 'CheckHostIP=no',
        '-o', 'PasswordAuthentication=no', '-o', 'KbdInteractiveAuthentication=no',
        '-o', 'ConnectTimeout=5', '-o', 'ConnectionAttempts=1',
        '-o', 'GlobalKnownHostsFile=NUL',
        '-o', 'UserKnownHostsFile=' + str(identity['known_hosts_path']),
        '-o', 'HostKeyAlias=' + identity['host_key_alias'], '-i', str(identity['key_path']),
        'arduino@' + ip, 'python3 -c ' + shlex.quote(REMOTE_HELPER)]


def payload_bytes(package, serial, previous_instance=None):
    if not isinstance(serial, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', serial):
        raise UpdateError('credential_missing')
    if previous_instance is not None and (not isinstance(previous_instance, str)
            or not re.fullmatch(r'[0-9a-f]{32}', previous_instance)):
        raise UpdateError('verification_failed')
    payload = {'serial': serial, 'app_code_version': APP_CODE_VERSION,
               'previous_instance': previous_instance,
               'package_sha256': package['sha256'], 'files': {
        name: {'base64': base64.b64encode(package['files'][name]).decode('ascii'),
               'sha256': hashlib.sha256(package['files'][name]).hexdigest()}
        for name in ALLOWLIST}}
    return json.dumps(payload, separators=(',', ':')).encode('utf-8')


def run_remote_update(ip, identity, package, emit=emit_packet, previous_instance=None,
                      process_factory=subprocess.Popen, clock=time.monotonic):
    """Only called from explicit deploy. No stderr/raw log forwarding."""
    process = None
    lines = queue.Queue(maxsize=64)
    reader = None
    writer = None
    write_finished = threading.Event()
    write_errors = []
    last = None
    def read_lines():
        try:
            while True:
                line = process.stdout.readline(16385)
                if not line:
                    break
                lines.put(line)
        finally:
            lines.put(None)
    def write_payload():
        try:
            process.stdin.write(payload_bytes(package, identity['serial'], previous_instance))
            process.stdin.close()
        except Exception:
            write_errors.append(True)
        finally:
            write_finished.set()
    try:
        deadline, total = clock() + 1950, 0
        process = process_factory(ssh_args(ip, identity), stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, shell=False,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        reader = threading.Thread(target=read_lines, daemon=True)
        reader.start()
        writer = threading.Thread(target=write_payload, daemon=True)
        writer.start()
        while clock() < deadline:
            if write_finished.is_set() and write_errors:
                raise UpdateError('transport_lost')
            try:
                line = lines.get(timeout=.25)
            except queue.Empty:
                continue
            if line is None:
                break
            total += len(line)
            if len(line) > 16384 or total > 256 * 1024:
                raise UpdateError('transport_lost')
            try:
                packet = json.loads(line)
            except (ValueError, UnicodeError):
                continue
            safe = redact_packet(packet)
            if safe is not None:
                if safe.get('serial') not in (None, identity['serial']):
                    raise UpdateError('serial_mismatch')
                safe['network_url'] = f'http://{ip}:9527'
                # The board helper's final report still needs independent PC
                # readback; do not announce complete before that succeeds.
                if packet.get('stage') == 'complete':
                    preliminary = dict(safe, stage='verify_network', progress=95)
                    preliminary['checks'] = dict(safe.get('checks', {}),
                        network_readback='running', probe_readback='running', wireless_update='running')
                    emit(preliminary)
                else:
                    emit(safe)
                last = safe
        else:
            raise UpdateError('transport_lost')
        code = process.wait(timeout=5)
        if code != 0 or not last or last.get('stage') != 'complete':
            raise UpdateError((last or {}).get('errors', ['transport_lost'])[0]
                              if (last or {}).get('errors') else 'transport_lost')
        return last
    except UpdateError:
        raise
    except Exception:
        raise UpdateError('transport_lost') from None
    finally:
        if process and process.poll() is None:
            process.kill()  # Exact SSH child created for this update only.
        if reader:
            reader.join(timeout=.2)
        if writer:
            writer.join(timeout=.2)


def update(root=ROOT, deploy=False, url=None, emit=emit_packet,
           fetch=discovery.fetch_status, ssh_verify=discovery.verify_ssh_identity,
           discover=discovery.discover, remote=run_remote_update, sleeper=time.sleep):
    package = review_package(root)
    plan = {'stage': 'package_plan', 'progress': 10,
            'checks': {'package_review': 'pass', 'software_compile': 'unknown',
                       'elf_check': 'unknown', 'network_readback': 'unknown',
                       'probe_readback': 'unknown', 'wireless_update': 'pending',
                       'cold_boot': 'unknown', 'usb_readback': 'unknown'}}
    emit(plan)
    if not deploy:
        return plan
    try:
        identity = strict_identity(root)
    except discovery.DiscoveryError:
        raise UpdateError('credential_missing') from None
    if url is None:
        def discovery_emit(packet):
            # Discovery completion is only the preflight for this update.
            emit({'stage': 'discover', 'progress': 10, 'serial': identity['serial'],
                  'checks': {'network_readback': 'running', 'wireless_update': 'running'}})
        try:
            found = discover(root=root, emit=discovery_emit)
            url = found['network_url']
        except discovery.DiscoveryError as error:
            raise UpdateError(error.code) from None
    else:
        url = discovery.literal_url(url)
        if not url or url not in discovery.cached_candidates(root):
            raise UpdateError('network_unreachable')
    try:
        first = fetch(url)
        sleeper(.3)
        second = fetch(url)
        verify_locked_pair(first, second)
        ssh_verify(urlsplit(url).hostname, identity)
    except discovery.DiscoveryError as error:
        raise UpdateError(error.code) from None
    emit({'stage': 'preflight', 'progress': 12, 'serial': identity['serial'],
          'network_url': url, 'checks': {'network_readback': 'pass'}})
    previous_instance = second.get('app_instance_id')
    result = remote(urlsplit(url).hostname, identity, package, emit=emit,
                    previous_instance=previous_instance)
    # Independent host verification: remote compile/start exit status alone is
    # insufficient. Historical cold-boot evidence is never marked pass here.
    first = fetch(url)
    sleeper(.3)
    second = fetch(url)
    checks = verify_new_status(first, second, previous_instance)
    checks.update(package_review='pass', cold_boot='unknown', usb_readback='unknown')
    final = {'stage': 'complete', 'progress': 100, 'serial': identity['serial'],
             'network_url': url, 'checks': checks, 'rollback': result.get('rollback', 'not_needed')}
    emit(final)
    return final


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group()
    choice.add_argument('--plan', action='store_true', help='Offline package validation (default)')
    choice.add_argument('--deploy', action='store_true', help='Explicit fixed-app wireless update')
    parser.add_argument('--url', help='Previously verified literal private IPv4 URL, port 9527')
    args = parser.parse_args(argv)
    checks = {key: 'unknown' for key in CHECK_NAMES}
    def emit_with_state(packet):
        safe = redact_packet(packet)
        if safe is not None:
            checks.update(safe.get('checks', {}))
            emit_packet(safe)
    try:
        update(deploy=args.deploy, url=args.url, emit=emit_with_state)
        return 0
    except UpdateError as error:
        code = error.code
    except Exception:
        code = 'update_failed'
    checks = {key: ('fail' if value == 'running' else value) for key, value in checks.items()}
    checks.update(wireless_update='fail', cold_boot='unknown', usb_readback='unknown')
    emit_packet({'stage': 'failed', 'progress': 0, 'checks': checks, 'errors': [code]})
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
