"""Loopback-only wireless access to the official robot WebUI.

This module never generates servo targets, renews control heartbeats, changes
mode, or resumes an action. The official browser page owns its normal commands.
"""
from device_config import DEVICE_SERIAL
import atexit
from release_profile import constrain_command, validate_release_action
from functools import lru_cache
import http.client
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'tools'))
import discover_unoq as discovery
from deploy_readonly_wifi import strict_identity

WAITING='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta http-equiv="refresh" content="5"><title>XGO Duck 官方控制台</title><body style="background:#101923;color:#e4edf5;font:18px system-ui;padding:40px;line-height:1.8"><h1>XGO Duck 官方控制台</h1><p>正在通过无线连接官方控制程序…</p><p>页面会自动刷新。重连只读取状态，不会自动上力或恢复动作。</p></body></html>'''.encode('utf-8')

@lru_cache(maxsize=1)
def wireless_key():
    auth=json.loads((ROOT/'tools/.private/wireless-calibration-auth.json').read_text('utf-8'))
    key=auth.get('key')
    if not isinstance(key,str) or not key:raise ValueError('wireless_key_missing')
    return key


class WirelessTunnel:
    def __init__(self):
        self.lock=threading.Lock();self.process=None;self.port=None;self.next_attempt=0
        self.stopped=threading.Event();self.error='not_connected';self.official=False
        threading.Thread(target=self._watch,name='official-wireless',daemon=True).start()
        atexit.register(self.close)
    def _watch(self):
        while not self.stopped.wait(2):
            try:self.ensure()
            except Exception:self.error='wireless_connection_pending'
    def ensure(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:return self.port
            self.port=None;self.official=False
            if time.monotonic()<self.next_attempt:return None
            self.next_attempt=time.monotonic()+10
            identity=strict_identity(ROOT)
            if identity.get('serial')!=DEVICE_SERIAL:raise ValueError('wrong_bound_controller')
            choices=discovery.cached_candidates(ROOT)
            cached=discovery.read_small_json(ROOT/'qa/official-connection.json')
            if cached and (u:=discovery.literal_url(cached.get('network_url'))):choices=[u]+[x for x in choices if x!=u]
            if not choices:choices=discovery.mdns_candidates(seconds=5)
            selected=None
            for url in choices[:2]:
                ip=urlsplit(url).hostname
                try:proof=discovery.verify_ssh_identity(ip,identity,timeout=4,root=ROOT)
                except discovery.DiscoveryError:continue
                if proof.get('ssh_pin_verified') and proof.get('sys_serial')==DEVICE_SERIAL:selected=(url,ip);break
            if selected is None and choices:
                # DHCP may change after cold boot. Rediscover advertised boards,
                # then authenticate the same pinned controller before using one.
                for url in discovery.mdns_candidates(seconds=5)[:8]:
                    ip=urlsplit(url).hostname
                    try:proof=discovery.verify_ssh_identity(ip,identity,timeout=4,root=ROOT)
                    except discovery.DiscoveryError:continue
                    if proof.get('ssh_pin_verified') and proof.get('sys_serial')==DEVICE_SERIAL:selected=(url,ip);break
            if selected is None:
                self.error='bound_controller_unreachable';return None
            executable,null_config=discovery.ssh_transport(ROOT)
            with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
            args=[str(executable),'-F',null_config,'-N','-T','-o','BatchMode=yes','-o','IdentitiesOnly=yes',
                  '-o','IdentityAgent=none','-o','StrictHostKeyChecking=yes','-o','UpdateHostKeys=no',
                  '-o','VerifyHostKeyDNS=no','-o','CheckHostIP=no','-o','PasswordAuthentication=no',
                  '-o','KbdInteractiveAuthentication=no','-o','ConnectTimeout=3','-o','ConnectionAttempts=1',
                  '-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=2',
                  '-o','GlobalKnownHostsFile='+null_config,'-o','UserKnownHostsFile='+str(identity['known_hosts_path']),
                  '-o','HostKeyAlias='+identity['host_key_alias'],'-i',str(identity['key_path']),
                  '-L',f'127.0.0.1:{port}:127.0.0.1:9527','arduino@'+selected[1]]
            self.process=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            self.port=port;self.error=None
            (ROOT/'qa').mkdir(exist_ok=True)
            (ROOT/'qa/official-connection.json').write_text(json.dumps({'network_url':selected[0],
                'ssh_pin_verified':True,'sys_serial':DEVICE_SERIAL,'checked_at':time.time()},indent=2)+'\n',encoding='utf-8')
            return port
    def close(self):
        self.stopped.set()
        with self.lock:
            process,self.process=self.process,None;self.port=None
        if process is not None:
            try:
                if process.poll() is None:process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=.5)
            except (OSError,subprocess.TimeoutExpired):pass

    def invalidate_stalled(self, port, process):
        """Retire only the same owned relay after repeated read failures.

        The next watcher pass reauthenticates and opens a new relay. Requests
        are never replayed and no robot mode/target is restored.
        """
        with self.lock:
            if process is None or self.process is not process or self.port != port:
                return False
            try:
                if process.poll() is None:process.terminate()
            except OSError:
                if process.poll() is None:return False
            self.process=None;self.port=None;self.official=False
            self.error='wireless_reconnecting';self.next_attempt=0
        try:
            try:process.wait(timeout=.5)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=.5)
        except (OSError,subprocess.TimeoutExpired):pass
        return True


class OfficialServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,address,tunnel):self.tunnel=tunnel;super().__init__(address,OfficialHandler)


class OfficialHandler(BaseHTTPRequestHandler):
    protocol_version='HTTP/1.1'
    def setup(self):
        self._upstream=None;self._upstream_port=None;self._upstream_completed=0
        super().setup()
    def close_upstream(self):
        connection,self._upstream=self._upstream,None
        if connection is not None:connection.close()
    def upstream(self,port):
        if self._upstream is not None and (self._upstream_port!=port or time.monotonic()-self._upstream_completed>=1):
            self.close_upstream()
        if self._upstream is None:
            self._upstream=http.client.HTTPConnection('127.0.0.1',port,timeout=2)
            self._upstream_port=port
        return self._upstream
    def upstream_completed(self,response):
        if response.will_close:self.close_upstream()
        else:self._upstream_completed=time.monotonic()
    def finish(self):
        try:super().finish()
        finally:self.close_upstream()
    def log_message(self,*args):pass
    def send(self,status,raw,kind='application/json; charset=utf-8'):
        if not isinstance(raw,bytes):raw=json.dumps(raw,ensure_ascii=False).encode('utf-8')
        self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(raw)))
        self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
        try:self.wfile.write(raw)
        except OSError:pass
    def do_GET(self):self.forward()
    def do_POST(self):self.forward()
    def do_OPTIONS(self):self.send(403,{'detail':'Use the local official control page'})
    def forward(self):
        # Forward each request once. Only official endpoints are writable.
        path=urlsplit(self.path).path
        writes={'/api/mode','/api/command','/api/filter','/api/pick','/api/roulade',
                '/api/cal/enter','/api/cal/exit','/api/cal/targets','/api/cal/finish',
                '/api/servo/enter','/api/servo/exit','/api/servo'}
        if self.command=='POST':
            if path not in writes:return self.send(410,{'detail':'旧控制入口已停用，请使用官方控制台。'})
            origin=self.headers.get('Origin')
            if origin and origin not in ('http://127.0.0.1:8791','http://localhost:8791'):
                return self.send(403,{'detail':'Local origin required'})
            try:length=int(self.headers.get('Content-Length','0'))
            except ValueError:return self.send(400,{'detail':'Invalid length'})
            if not 0<length<=8192:return self.send(400,{'detail':'Invalid length'})
            body=self.rfile.read(length)
            try:
                if path in ('/api/pick','/api/roulade'):validate_release_action(path.rsplit('/',1)[1])
                if path=='/api/command':body=json.dumps(constrain_command(json.loads(body))).encode('utf-8')
            except (ValueError,TypeError,KeyError) as error:return self.send(403,{'detail':str(error)})
        else:body=None
        if path=='/api/wireless-health' and self.command=='GET':
            t=self.server.tunnel
            return self.send(200,{'ok':True,'interface':'official-webui-wireless-proxy','connected':t.port is not None and t.official,
                'official_runtime_observed':t.official,'error':t.error,'automatic_motion':False})
        port=self.server.tunnel.port
        if port is None:
            self.close_upstream()
            if self.command=='GET' and path=='/':return self.send(200,WAITING,'text/html; charset=utf-8')
            return self.send(503,{'detail':'无线连接尚未就绪；操作没有自动重试。'})
        connection=self.upstream(port)
        try:
            if self.command=='GET' and path=='/' and not self.server.tunnel.official:
                connection.request('GET','/api/status')
                probe=connection.getresponse();state=json.loads(probe.read(256*1024))
                self.upstream_completed(probe)
                if state.get('deployment',{}).get('reference')!='rl/raspi_deploy':
                    return self.send(200,WAITING,'text/html; charset=utf-8')
                self.server.tunnel.official=True
                connection=self.upstream(port)
            headers={'Host':'127.0.0.1:9527'}
            if body is not None:
                headers['Content-Type']=self.headers.get('Content-Type','application/json')
                headers['X-Cal-Key']=wireless_key()
            connection.request(self.command,self.path,body=body,headers=headers)
            response=connection.getresponse();raw=response.read(16*1024*1024+1)
            if len(raw)>16*1024*1024:raise ValueError('oversize_response')
            self.upstream_completed(response)
            self.server.tunnel.error=None
            if path=='/api/status' and response.status==200:
                try:
                    state=json.loads(raw)
                    self.server.tunnel.official=state.get('deployment',{}).get('reference')=='rl/raspi_deploy'
                except ValueError:pass
            self.send(response.status,raw,response.getheader('Content-Type','application/octet-stream'))
        except (OSError,ValueError,http.client.HTTPException):
            self.close_upstream()
            self.server.tunnel.error='wireless_reply_unconfirmed'
            if self.command=='GET' and path=='/':return self.send(200,WAITING,'text/html; charset=utf-8')
            return self.send(503,{'detail':'主控连接未返回；操作结果待确认，没有自动重试。'})


def main():
    tunnel=WirelessTunnel()
    print('XGO Duck official WebUI via pinned wireless: http://127.0.0.1:8791/ (no automatic motion)',flush=True)
    try:OfficialServer(('127.0.0.1',8791),tunnel).serve_forever()
    finally:tunnel.close()

if __name__=='__main__':main()
