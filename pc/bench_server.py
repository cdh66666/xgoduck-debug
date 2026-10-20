"""Preserved workbench UI over the tested official runtime; no startup writes."""
from __future__ import annotations
import hashlib
import http.client
import json
import mimetypes
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import unquote, urlsplit

from official_control_proxy import ROOT, OfficialHandler, OfficialServer, WirelessTunnel
from bench_tests import BenchTests
from bench_maintenance import MaintenanceSession
from bench_calibration import Calibration
from active_runtime import runtime_manifest, runtime_source
from official_status_reader import OfficialReader

REVISION = '2026-10-20-fixture-calibration-r2'
EXPECTED_ZERO = [2048]*15
EXPECTED_ZERO[13] = 2046


def baseline():
    manifest = runtime_manifest(ROOT)
    overlay = manifest.get('active_overlay')
    walk_source = runtime_source(ROOT, 'python/xgoduck_walk.onnx', manifest)
    source_parts = walk_source.parts
    run_id = source_parts[source_parts.index('runs')+1] if 'runs' in source_parts and source_parts.index('runs')+1 < len(source_parts) else (walk_source.parent.name if overlay else None)
    walk_policy = {'active': bool(overlay), 'sha256': manifest['new_source_manifest']['python/xgoduck_walk.onnx'],
                   'run_id': run_id, 'label': 'R5无EVA·速度训练' if overlay else '官方站立 / 行走',
                   'physical_acceptance': 'pending' if overlay else 'historical_results_only'}
    extension = manifest.get('keyboard_extension')
    fast_policy = {'active': bool(extension),
                   'sha256': manifest['new_source_manifest'].get('python/xgoduck_fast_forward.onnx'),
                   'label': 'R5无EVA·前进 / 快跑', 'command_points': [[.2,0,0],[.3,0,0]],
                   'physical_acceptance': 'pending', 'all_direction_replacement': False}
    admission = manifest.get('keyboard_admission', {})
    navigation_scope = admission.get('navigation_scope', {})
    navigation_policy = {'active':bool(extension and admission.get('navigation_deployable_offline')),
                         'sha256':navigation_scope.get('artifact_sha256'),
                         'label':'原版模型·已验证后退 / 转向',
                         'command_points':navigation_scope.get('command_points', []),
                         'observed_backward_vx_m_s':navigation_scope.get('backward_observed_vx_m_s'),
                         'physical_acceptance':'pending'}
    checked = {}
    for name, digest in manifest['new_source_manifest'].items():
        path = runtime_source(ROOT, name, manifest)
        checked[name] = path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest
    return {'revision': REVISION, 'reference': '9dc36787bc61b102c2da0f97c9c758e639e90ab8',
            'local_sources_verified': all(checked.values()), 'files_checked': len(checked),
            'zero_sha256': manifest['zero_sha256'], 'exceptions': ['SDK/无线接入', '直接起身', '起身说明'] + (['本次训练行走权重'] if manifest.get('active_overlay') else []) + (['渐进全身低头控制','限定前进PPO权重'] if extension else []),
            'walk_policy': walk_policy,
            'fast_policy': fast_policy,
            'navigation_policy': navigation_policy,
            'manual_pick': {'active':bool(extension), 'physical_acceptance':'pending'},
            'live_sources_verified': False}


class WorkbenchServer(OfficialServer):
    def __init__(self, address, tunnel, *, reader_factory=OfficialReader):
        self.reader = reader_factory(tunnel)
        self.baseline = baseline()
        self.session_token = secrets.token_urlsafe(32)
        self.tests = BenchTests(ROOT, self.reader.get)
        self.events = []
        self.maintenance = MaintenanceSession(ROOT)
        self.calibration = Calibration(ROOT, self.maintenance)
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
        if path in ('/api/bench/status', '/api/health'):
            try:
                if self.maintenance_locked():
                    raise ConnectionError('厂家校准维护中；正式模型控制已暂停')
                state = self.server.reader.get()
                expected = self.server.maintenance.expected_zeros or EXPECTED_ZERO
                ready = state.get('zero_pos') == expected and state.get('recovery', {}).get('thresholds', {}).get('default_pose_s') == 0
                return self.send(200, {'ok': True, 'revision': REVISION, 'connected': True,
                    'startup_read_only': True, 'runtime': state, 'baseline': self.server.baseline,
                    'runtime_basis_matches': ready, 'test': self.server.tests.snapshot(),
                    'events': self.server.events[-20:]})
            except (OSError, ValueError, http.client.HTTPException) as error:
                return self.send(200, {'ok': True, 'revision': REVISION, 'connected': False,
                    'startup_read_only': True, 'runtime': None, 'runtime_basis_matches': False,
                    'error': str(error), 'baseline': self.server.baseline,
                    'test': self.server.tests.snapshot(), 'events': self.server.events[-20:]})
        if path == '/api/bench/tests':
            return self.send(200, self.server.tests.snapshot())
        if path == '/api/bench/history':
            history = []
            for name in ('pick', 'roulade', 'walk'):
                history.append(json.loads((ROOT/f'direct-getup-20261019/latest-{name}.json').read_text('utf-8')) | {'profile': name, 'historical': True})
            return self.send(200, {'items': history})
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


def main():
    tunnel = WirelessTunnel()
    server = WorkbenchServer(('127.0.0.1', 8791), tunnel)
    print(f'XGO Duck workbench {REVISION}: http://127.0.0.1:8791/ (startup read-only)', flush=True)
    try: server.serve_forever()
    finally:
        server.tests.stop()
        server.calibration.transport.close()
        server.server_close()
        tunnel.close()


if __name__ == '__main__': main()
