"""The preserved workbench with calibration supplied by its normal firmware."""
import hashlib
import http.client
import json
import math
from urllib.parse import urlsplit,parse_qs

from bench_server import WorkbenchServer, WorkbenchHandler, baseline as policy_baseline
from official_control_proxy import ROOT, WirelessTunnel, wireless_key
from factory_calibration import FactoryMaintenance, FactoryCalibration, board_request
from active_runtime import verified_sources
from official_status_stream import OfficialStatusStream
from servo_current_recorder import CurrentRecorder, export_csv, current_series
from release_profile import PROFILE

REVISION = '2026-10-22-rx-stream-r4'


def wireless_transport_alive(tunnel):
    process = getattr(tunnel, 'process', None)
    return tunnel.port is not None and process is not None and process.poll() is None


def integrated_baseline():
    manifest=verified_sources(ROOT)
    sources=manifest['new_source_manifest']
    basic=bool(manifest.get('basic_keyboard_release'))
    stand_only=bool(manifest.get('stand_profile')) and not basic
    policy=policy_baseline()
    unified=dict(active=True,sha256=sources['python/xgoduck_walk.onnx'],
                 model='xgoduck_walk.onnx',label='新版基础键盘 · 同一actor',
                 single_actor=True,command_points=PROFILE.get('command_points',[]),
                 physical_acceptance='pending') if basic else None
    return {'revision':REVISION,'reference':'9dc36787bc61b102c2da0f97c9c758e639e90ab8',
            'local_sources_verified':True,'files_checked':len(sources),'live_sources_verified':False,
            'zero_sha256':manifest['zero_sha256'],'exceptions':['SDK/无线接入','直接起身','厂家校准命令','编号后重锁']+(['显示相对航向与低频舵机电压'] if manifest.get('display_telemetry') else [])+(['当前训练行走权重'] if manifest.get('active_overlay') else [])+(['新版基础键盘单一actor'] if basic else ['渐进全身低头控制','限定前进PPO权重'] if manifest.get('keyboard_extension') else []),
            'walk_policy':dict(unified) if basic else policy['walk_policy'],
            'fast_policy':dict(unified) if basic else dict(policy['fast_policy'],active=False) if stand_only else policy['fast_policy'],
            'navigation_policy':dict(unified) if basic else dict(policy['navigation_policy'],active=False) if stand_only else policy['navigation_policy'],
            'manual_pick':dict(policy['manual_pick'],active=False) if basic or stand_only else policy['manual_pick'],
            'basic_keyboard_policy':dict(unified) if basic else {'active':False},
            'release_profile':PROFILE if basic or stand_only else None,
            'calibration_zero_home':bool(manifest.get('calibration_home')),
            'servo_current_monitor':bool(manifest.get('current_telemetry'))}


class FactoryWorkbenchServer(WorkbenchServer):
    def __init__(self,address,tunnel):
        super().__init__(address,tunnel,reader_factory=OfficialStatusStream)
        self.calibration.transport.close()
        self.maintenance=FactoryMaintenance(ROOT)
        self.calibration=FactoryCalibration(ROOT,self.maintenance,board_request(tunnel,wireless_key))
        self.baseline=integrated_baseline()
        self.overlay_modified=(ROOT/'active-runtime.json').stat().st_mtime_ns if (ROOT/'active-runtime.json').exists() else None
        self.RequestHandlerClass=FactoryWorkbenchHandler
        self.current_recorder=CurrentRecorder(ROOT) if self.baseline.get('servo_current_monitor') else None

    def server_close(self):
        recorder=getattr(self,'current_recorder',None)
        if recorder is not None:recorder.close()
        super().server_close()


class FactoryWorkbenchHandler(WorkbenchHandler):
    def received_frames(self, snapshot=None):
        options={}
        observation_id=snapshot.get('pc_observation_id') if snapshot else None
        if isinstance(observation_id,int):options['through_observation']=observation_id
        query=parse_qs(urlsplit(self.path).query)
        observation=query.get('after_observation',[None])[0]
        if observation is not None:
            try:
                observation=int(observation)
                if observation>=0:return self.server.reader.frames(after_observation=observation,**options)
            except ValueError:pass
        raw=query.get('after_observed',[None])[0]
        if raw is not None:
            try:
                after=float(raw)
                if math.isfinite(after) and after>=0:
                    return self.server.reader.frames(after=after,**options)
            except ValueError:pass
        return self.server.reader.frames(**options)

    def observe_factory(self, state):
        factory=state.get('factory_calibration',{})
        self.server.maintenance.active=factory.get('active',False)
        self.server.maintenance.expected_zeros=state.get('zero_pos')
        calibration=self.server.calibration
        with calibration.lock:
            calibration.state['supported']=factory.get('supported',False)
            calibration.state['held_mask']=factory.get('held_mask',0) if factory.get('active') else 0
            calibration.state['readback_live']=False
            calibration.state['registers']=factory.get('registers',{})
            calibration.state['last_read_at']=factory.get('last_read')
            calibration.state['boot_epoch']=factory.get('boot_epoch')

    def do_GET(self):
        path=urlsplit(self.path).path
        if path=='/api/bench/current-series':
            try:return self.send(200,current_series(ROOT/'current-recordings'))
            except (OSError,ValueError) as error:return self.send(200,{'ok':False,'samples':[],'error':str(error)})
        if path=='/api/bench/current-log.csv':
            try:return self.send(200,export_csv(ROOT/'current-recordings'),'text/csv; charset=utf-8')
            except (OSError,ValueError) as error:return self.send(404,{'detail':str(error)})
        if path in ('/api/health','/api/bench/status','/api/bench/calibration'):
            try:
                modified=(ROOT/'active-runtime.json').stat().st_mtime_ns if (ROOT/'active-runtime.json').exists() else None
                if modified!=self.server.overlay_modified:
                    self.server.baseline=integrated_baseline();self.server.overlay_modified=modified
                state=self.server.reader.get()
                self.observe_factory(state)
                if path=='/api/bench/calibration':
                    return self.send(200,{'maintenance':self.server.maintenance.snapshot(),
                        'calibration':self.server.calibration.snapshot()})
                return self.send(200,{'ok':True,'revision':REVISION,'connected':True,'wireless_transport_alive':wireless_transport_alive(self.server.tunnel),'startup_read_only':True,
                    'runtime':state,'runtime_frames':self.received_frames(state),'baseline':self.server.baseline,
                    'status_stream':self.server.reader.diagnostics() if hasattr(self.server.reader,'diagnostics') else None,
                    'runtime_basis_matches':self.server.baseline['local_sources_verified'] and state.get('recovery',{}).get('thresholds',{}).get('default_pose_s')==0,
                    'test':self.server.tests.snapshot(),'events':self.server.events[-20:],
                    'current_recording':self.server.current_recorder.snapshot() if getattr(self.server,'current_recorder',None) else None})
            except (OSError,ValueError,http.client.HTTPException) as error:
                if path=='/api/bench/calibration':
                    value=self.server.calibration.snapshot();value['message']='主控未连接，保持与卸力待读取';value['readback_live']=False
                    return self.send(200,{'maintenance':self.server.maintenance.snapshot(),'calibration':value})
                return self.send(200,{'ok':True,'revision':REVISION,'connected':False,'wireless_transport_alive':wireless_transport_alive(self.server.tunnel),'startup_read_only':True,
                    'runtime':None,'runtime_frames':self.received_frames(),'runtime_basis_matches':False,'error':str(error),'baseline':self.server.baseline,
                    'status_stream':self.server.reader.diagnostics() if hasattr(self.server.reader,'diagnostics') else None,
                    'test':self.server.tests.snapshot(),'events':self.server.events[-20:],
                    'current_recording':self.server.current_recorder.snapshot() if getattr(self.server,'current_recorder',None) else None})
        return super().do_GET()


def main():
    tunnel=WirelessTunnel()
    server=FactoryWorkbenchServer(('127.0.0.1',8791),tunnel)
    print(f'XGO Duck {REVISION}: http://127.0.0.1:8791/',flush=True)
    try:server.serve_forever()
    finally:
        server.tests.stop();server.calibration.close();server.server_close();tunnel.close()


if __name__=='__main__':main()
