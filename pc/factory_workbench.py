"""The preserved workbench with calibration supplied by its normal firmware."""
import http.client
import math
from urllib.parse import urlsplit,parse_qs

from bench_server import WorkbenchServer, WorkbenchHandler, baseline as integrated_baseline, REVISION
from official_control_proxy import ROOT, WirelessTunnel
from servo_current_recorder import CurrentRecorder, export_csv, current_series
from motion_archive import MotionArchive



def wireless_transport_alive(tunnel):
    process = getattr(tunnel, 'process', None)
    return tunnel.port is not None and process is not None and process.poll() is None


def pose_basis_matches(baseline, state):
    """Match the deployed pose contract, not a recovery delay from an old build.

    This is parameter compatibility only. Running binary identity is reported
    separately by /api/runtime and the firmware panel.
    """
    if not baseline.get('local_sources_verified'):
        return False
    deployment=state.get('deployment',{})
    if deployment.get('reference')=='rl/raspi_deploy':
        return (deployment.get('control_hz')==50 and
                deployment.get('home_deg')==[0,-5,-22,0,18,20,20,0,0,0,5,22,0,-18] and
                deployment.get('default_deg')==[0,-5,-22,0,18,0,5,22,0,-18,20,20,0,0,0] and
                state.get('encoder_center')==2047)
    return state.get('recovery',{}).get('thresholds',{}).get('default_pose_s')==0


class FactoryWorkbenchServer(WorkbenchServer):
    def __init__(self,address,tunnel):
        super().__init__(address,tunnel)
        self.RequestHandlerClass=FactoryWorkbenchHandler
        self.current_recorder=CurrentRecorder(ROOT) if self.baseline.get('servo_current_monitor') else None
        self.motion_archive=MotionArchive(ROOT,tunnel)

    def server_close(self):
        archive=getattr(self,'motion_archive',None)
        if archive is not None:archive.close()
        recorder=getattr(self,'current_recorder',None)
        if recorder is not None:recorder.close()
        super().server_close()


class FactoryWorkbenchHandler(WorkbenchHandler):
    def received_frames(self, snapshot=None):
        options={}
        observation_id=snapshot.get('pc_observation_id') if snapshot else None
        if isinstance(observation_id,int):options['through_observation']=observation_id
        query=parse_qs(urlsplit(self.path).query)
        if query.get('latest')==['1']:return []
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
        return []

    def observe_factory(self, state):
        factory=state.get('factory_calibration',{})
        self.server.maintenance.active=factory.get('active',False)
        self.server.maintenance.expected_zeros=state.get('zero_pos')
        calibration=self.server.calibration
        with calibration.lock:
            calibration.state['supported']=factory.get('supported',False)
            calibration.state['unloaded_calibration_supported']=factory.get('unloaded_calibration_supported',False)
            calibration.state['calibration_loads_motor']=factory.get('calibration_loads_motor')
            calibration.state['held_mask']=factory.get('held_mask',0) if factory.get('active') else 0
            calibration.state['readback_live']=False
            calibration.state['registers']=factory.get('registers',{})
            calibration.state['last_read_at']=factory.get('last_read')
            calibration.state['boot_epoch']=factory.get('boot_epoch')

    def do_GET(self):
        path=urlsplit(self.path).path
        if path=='/api/bench/motions':
            return self.send(200,self.server.motion_archive.listing())
        if path.startswith('/api/bench/motions/'):
            sid=path.removeprefix('/api/bench/motions/').removesuffix('/download')
            try:
                if path.endswith('/download'):
                    return self.send(200,self.server.motion_archive.path(sid).read_bytes(),'application/gzip')
                return self.send(200,self.server.motion_archive.episode(sid))
            except (OSError,ValueError) as error:
                return self.send(404,{'detail':str(error)})
        if path=='/api/bench/current-series':
            try:return self.send(200,current_series(ROOT/'current-recordings'))
            except (OSError,ValueError) as error:return self.send(200,{'ok':False,'samples':[],'error':str(error)})
        if path=='/api/bench/current-log.csv':
            try:return self.send(200,export_csv(ROOT/'current-recordings'),'text/csv; charset=utf-8')
            except (OSError,ValueError) as error:return self.send(404,{'detail':str(error)})
        if path in ('/api/health','/api/bench/status','/api/bench/calibration'):
            try:
                state=self.server.reader.get()
                self.observe_factory(state)
                if path=='/api/bench/calibration':
                    return self.send(200,{'maintenance':self.server.maintenance.snapshot(),
                        'calibration':self.server.calibration.snapshot()})
                return self.send(200,{'ok':True,'revision':REVISION,'connected':True,'wireless_transport_alive':wireless_transport_alive(self.server.tunnel),'startup_read_only':True,
                    'runtime':state,'runtime_frames':self.received_frames(state),'baseline':self.server.baseline,
                    'status_stream':self.server.reader.diagnostics() if hasattr(self.server.reader,'diagnostics') else None,
                    'runtime_basis_matches':pose_basis_matches(self.server.baseline,state),
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
