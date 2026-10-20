"""Lazy, serial-guarded SSH dispatcher for explicit maintenance RPCs.

Only the SSH/Python session is reused. Each request executes in a fresh namespace;
failed or ambiguous calls are never replayed, including after a reconnect.
"""
from __future__ import annotations

import base64
import json
from pathlib import Path
import queue
import shlex
import subprocess
import threading
import time
import zlib

from model_transport import _configuration

MAX_REPLY = 256 * 1024

# No remote file or daemon: this interpreter exists only for this SSH session.
DISPATCHER = r'''
import contextlib,io,json,sys
from pathlib import Path
MAX_REPLY=256*1024
wire_in=sys.stdin
wire_out=sys.stdout
class ReplyBuffer(io.StringIO):
 def write(self,value):
  if self.tell()+len(value)>MAX_REPLY:raise RuntimeError('reply_too_large')
  return super().write(value)
def send(value):
 data=json.dumps(value,ensure_ascii=True,separators=(',',':'))
 if len(data.encode('utf-8'))>MAX_REPLY:raise RuntimeError('reply_too_large')
 wire_out.write(data+'\n');wire_out.flush()
send({'kind':'ready'})
for line in wire_in:
 request=None;namespace={};captured=ReplyBuffer()
 try:
  request=json.loads(line)
  if Path('/sys/devices/soc0/serial_number').read_text().strip()!=request['serial']:
   raise RuntimeError('controller_identity_mismatch')
  namespace={'__name__':'__main__'}
  sys.stdin=io.StringIO(json.dumps(request['payload']))
  with contextlib.redirect_stdout(captured):
   exec(compile(request['source'],'<duck-calibration-rpc>','exec'),namespace,namespace)
  result=json.loads(captured.getvalue())
  if not isinstance(result,dict):raise RuntimeError('reply_not_dict')
  send({'id':request['id'],'ok':True,'value':result})
 except BaseException:
  # Do not expose source, request payloads, SSH credentials or raw exception text.
  send({'id':request.get('id') if isinstance(request,dict) else None,
        'ok':False,'error':'remote_execution_failed'})
  break
 finally:
  sys.stdin=wire_in
  sys.stdout=wire_out
  namespace.clear()  # release operation.lock and other per-call resources now
  captured.close()
'''


class CalibrationTransport:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.lock = threading.Lock()
        self.process = None
        self.replies = None
        self.serial = None
        self.sequence = 0
        self.last_timing = {}

    @staticmethod
    def _read_stdout(stream, replies):
        try:
            while True:
                line = stream.readline(MAX_REPLY + 2)
                if not line:
                    replies.put(('error', '主控 SSH 已断开；操作结果待确认，未自动重试'))
                    return
                if len(line) > MAX_REPLY or not line.endswith(b'\n'):
                    replies.put(('error', '主控回复过大或不完整；操作未确认'))
                    return
                replies.put(('line', line))
        except (OSError, ValueError):
            replies.put(('error', '主控 SSH 读取失败；操作未确认'))

    def _receive(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('主控 SSH 超时；操作结果待确认，未自动重试')
        try:
            kind, data = self.replies.get(timeout=remaining)
        except queue.Empty:
            raise TimeoutError('主控 SSH 超时；操作结果待确认，未自动重试') from None
        if kind != 'line':
            raise RuntimeError(data)
        try:
            value = json.loads(data)
        except (ValueError, UnicodeError):
            raise RuntimeError('主控回复无效；操作未确认') from None
        if not isinstance(value, dict):
            raise RuntimeError('主控回复格式无效；操作未确认')
        return value

    def _connect(self, deadline):
        identity, address, executable, null_config = _configuration(self.root)
        self.serial = identity['serial']
        encoded = base64.b64encode(zlib.compress(DISPATCHER.encode('utf-8'), 9)).decode('ascii')
        command = "import base64,zlib;exec(compile(zlib.decompress(base64.b64decode('" + encoded + "')),'<duck-calibration-dispatcher>','exec'))"
        args = [str(executable), '-F', null_config, '-T', '-o', 'BatchMode=yes',
                '-o', 'IdentitiesOnly=yes', '-o', 'IdentityAgent=none',
                '-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no',
                '-o', 'VerifyHostKeyDNS=no', '-o', 'CheckHostIP=no',
                '-o', 'PasswordAuthentication=no', '-o', 'KbdInteractiveAuthentication=no',
                '-o', 'ConnectTimeout=3', '-o', 'ConnectionAttempts=1',
                '-o', 'GlobalKnownHostsFile=' + null_config,
                '-o', 'UserKnownHostsFile=' + str(identity['known_hosts_path']),
                '-o', 'HostKeyAlias=' + identity['host_key_alias'],
                '-i', str(identity['key_path']), 'arduino@' + address,
                'python3 -u -c ' + shlex.quote(command)]
        self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, shell=False,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self.replies = queue.Queue()
        threading.Thread(target=self._read_stdout,
            args=(self.process.stdout, self.replies), daemon=True).start()
        if self._receive(deadline) != {'kind': 'ready'}:
            raise RuntimeError('主控 SSH 初始化回复无效；操作未执行')

    def __call__(self, root, source, payload, timeout=45):
        if Path(root).resolve() != self.root:
            raise ValueError('校准传输项目路径不匹配')
        if timeout <= 0:
            raise ValueError('timeout 必须大于 0')
        with self.lock:
            started = time.monotonic()
            deadline = started + timeout
            reused = self.process is not None
            connected = started
            self.last_timing = {'connect_ms': 0.0, 'roundtrip_ms': 0.0, 'reused': reused}
            try:
                if not reused:
                    self._connect(deadline)
                    connected = time.monotonic()
                    self.last_timing['connect_ms'] = (connected - started) * 1000
                self.sequence += 1
                request = {'id': self.sequence, 'serial': self.serial,
                           'source': source, 'payload': payload}
                # One submission only. EOF/timeouts never reconnect and replay it.
                self.process.stdin.write((json.dumps(request, ensure_ascii=True) + '\n').encode('utf-8'))
                self.process.stdin.flush()
                reply = self._receive(deadline)
                if reply.get('id') != self.sequence or reply.get('ok') is not True:
                    raise RuntimeError('主控维护执行失败；操作结果待确认，未自动重试')
                if not isinstance(reply.get('value'), dict):
                    raise RuntimeError('主控维护回复格式无效；操作未确认')
                return reply['value']
            except TimeoutError:
                self._close()
                raise
            except (OSError, ValueError) as error:
                self._close()
                raise RuntimeError('主控 SSH 连接或数据传输失败；操作未确认，未自动重试') from error
            except BaseException:
                self._close()
                raise
            finally:
                self.last_timing['roundtrip_ms'] = (time.monotonic() - connected) * 1000

    def _close(self):
        process = self.process
        self.process = None
        self.replies = None
        self.serial = None
        if process is None:
            return
        try:
            process.terminate()
            process.wait(timeout=1)
        except (OSError, subprocess.TimeoutExpired):
            try:
                process.kill()
                process.wait(timeout=1)
            except (OSError, subprocess.TimeoutExpired):
                pass
        finally:
            for stream in (process.stdin, process.stdout):
                if stream is not None:
                    try:
                        stream.close()
                    except (OSError, ValueError):
                        pass

    def close(self):
        with self.lock:
            self._close()
