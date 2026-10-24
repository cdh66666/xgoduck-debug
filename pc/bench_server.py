"""Preserved workbench UI over the tested official runtime; no startup writes."""
from __future__ import annotations
import http.client
import json
import mimetypes
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import unquote, urlsplit

from official_control_proxy import ROOT, OfficialHandler, OfficialServer, wireless_key
from bench_tests import BenchTests
from bench_calibration import FactoryMaintenance, FactoryCalibration, board_request
from active_runtime import verified_sources
from official_status_stream import OfficialStatusStream
from release_profile import PROFILE

REVISION = '2026-10-24-official-console-r1'


def baseline():
    manifest=verified_sources(ROOT)
    sources=manifest['new_source_manifest']
    unified=dict(active=True,sha256=sources['python/xgoduck_walk.onnx'],
                 model='xgoduck_walk.onnx',label='新版基础键盘 · 同一actor',
                 single_actor=True,command_points=PROFILE.get('command_points',[]),
                 physical_acceptance='pending')
    return {'revision':REVISION,'reference':'9dc36787bc61b102c2da0f97c9c758e639e90ab8',
            'local_sources_verified':True,'files_checked':len(sources),'live_sources_verified':False,
            'zero_sha256':manifest['zero_sha256'],
            'exceptions':['SDK/无线接入','直接起身','厂家校准命令','编号后重锁']+
                         (['显示相对航向与低频舵机电压'] if manifest.get('display_telemetry') else [])+
                         ['新版基础键盘单一actor'],
            'walk_policy':dict(unified),'fast_policy':dict(unified),'navigation_policy':dict(unified),
            'manual_pick':{'active':False,'physical_acceptance':'pending'},
            'basic_keyboard_policy':dict(unified),'release_profile':PROFILE,
            'calibration_zero_home':bool(manifest.get('calibration_home')),
            'servo_current_monitor':bool(manifest.get('current_telemetry'))}


class WorkbenchServer(OfficialServer):
    def __init__(self, address, tunnel, *, reader_factory=OfficialStatusStream):
        self.reader = reader_factory(tunnel)
        self.baseline = baseline()
        self.session_token = secrets.token_urlsafe(32)
        self.tests = BenchTests(ROOT, self.reader.get)
        self.events = []
        self.maintenance = FactoryMaintenance(ROOT)
        self.calibration = FactoryCalibration(ROOT,self.maintenance,board_request(tunnel,wireless_key))
        super().__init__(address, tunnel)
        self.RequestHandlerClass = WorkbenchHandler

    def server_close(self):
        import simulation_http
        simulation_http.close(ROOT)
        self.reader.close()
        super().server_close()


class WorkbenchHandler(OfficialHandler):
    def maintenance_locked(self):
        calibration = self.server.calibration.snapshot()
        return self.server.maintenance.snapshot()['active'] or (calibration['active'] and calibration['operation'] in ('enter','restore'))

    def do_GET(self):
        path = urlsplit(self.path).path
        import simulation_http
        from model_http import handle_simulation_asset
        if simulation_http.handle_get(self,path,ROOT,self.server.session_token):return
        if handle_simulation_asset(self,path,ROOT):return
        if path == '/api/bench/calibration':
            return self.send(200, {'maintenance':self.server.maintenance.snapshot(),
                                   'calibration':self.server.calibration.snapshot()})
        if path == '/api/bench/tests':
            return self.send(200, self.server.tests.snapshot())
        if path in ('/api/local-session', '/api/bench/session'):
            return self.send(200, {'token': self.server.session_token})
        if path.startswith('/api/bench/report/'):
            relative = unquote(path.removeprefix('/api/bench/report/'))
            target = (ROOT/'bench-results'/relative).resolve()
            if not target.is_relative_to((ROOT/'bench-results').resolve()) or target.suffix != '.json':
                return self.send(403, {'detail': 'Invalid report path'})
            if target.is_file(): return self.send(200, target.read_bytes())
            return self.send(404, {'detail': 'Report not found'})
        if path == '/official':
            self.path = '/'
            return super().do_GET()
        local = ROOT/('index.html' if path == '/' else unquote(path).lstrip('/'))
        static_root = ROOT if path == '/' else next((ROOT/prefix.strip('/') for prefix in ('/assets/', '/model-assets/', '/downloads/') if path.startswith(prefix)), None)
        if static_root and not local.resolve().is_relative_to(static_root):
            return self.send(403, {'detail': 'Invalid static path'})
        if static_root and local.is_file() and local.suffix.lower() in ('.html', '.js', '.css', '.json', '.stl', '.png', '.jpg', '.webp', '.pdf', '.svg'):
            if local.suffix.lower() == '.html':
                raw = local.read_text('utf-8').replace('__TOKEN__', self.server.session_token).encode('utf-8')
            else: raw = local.read_bytes()
            return self.send(200, raw, mimetypes.guess_type(local.name)[0] or 'application/octet-stream')
        if path.startswith('/api/') or path in ('/servo', '/servo.html', '/servo.js', '/servo.css', '/app.js', '/style.css', '/calibrate.html', '/calibrate.js') or path.startswith('/assets/'):
            return super().do_GET()
        return self.send(404, {'detail': 'Not found'})

    def do_POST(self):
        path = urlsplit(self.path).path
        origin = self.headers.get('Origin')
        if origin and origin not in ('http://127.0.0.1:8791', 'http://localhost:8791'):
            return self.send(403, {'detail': 'Local origin required'})
        import simulation_http
        if simulation_http.handle_post(self,path,ROOT,self.server.session_token):return
        if path.startswith('/api/bench/'):
            if self.headers.get('X-Token') != self.server.session_token:
                return self.send(403, {'detail': 'Local session required'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 8192: raise ValueError('Invalid length')
                body = json.loads(self.rfile.read(size))
                if path == '/api/bench/calibration/action':
                    action = body['action']
                    if action in ('enter','recalibrate') and self.server.tests.snapshot().get('active'):
                        raise RuntimeError('先结束姿态测试记录，再进入校准维护')
                    if action == 'enter':
                        state = self.server.reader.get()
                        if state.get('mode') != 'off' or state.get('enabled') is not False:
                            raise RuntimeError('请先停止/卸力并确认 OFF，再进入厂家校准维护')
                    result = self.server.calibration.request(action,body.get('id',10),body.get('confirmed',False))
                    self.server.reader.value = None
                    self.server.events.append({'at':time.time(),'operation':'calibration/'+action,'state':'accepted; readback required'})
                    return self.send(202,result)
                if path == '/api/bench/tests/start':
                    if self.maintenance_locked():
                        raise RuntimeError('校准维护中不能启动模型测试')
                    return self.send(200, self.server.tests.start(body['profile'], body.get('duration_s', 15)))
                if path == '/api/bench/tests/stop':
                    return self.send(200, self.server.tests.stop())
                if path == '/api/bench/tests/annotate':
                    return self.send(200, self.server.tests.annotate(body['physical'], body.get('note', '')))
                return self.send(404, {'detail': 'Unknown test operation'})
            except (ValueError, KeyError, RuntimeError, OSError, http.client.HTTPException) as error:
                return self.send(409, {'detail': str(error)})
        if self.maintenance_locked():
            return self.send(409, {'detail':'厂家校准维护中，模型写接口已暂停；请在校准面板操作或全部卸力。'})
        # Record outcome independently of whether the official request succeeded.
        if path in ('/api/mode', '/api/pick', '/api/roulade', '/api/servo', '/api/servo/enter', '/api/servo/exit'):
            self.server.events.append({'at': time.time(), 'operation': path, 'state': 'requested; readback required'})
            del self.server.events[:-50]
        return super().do_POST()
