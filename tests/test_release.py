import ast
import contextlib
import hashlib
import http.client
import json
import io
from pathlib import Path
import subprocess
import shlex
import sys
import threading
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1];PC=ROOT/'pc';sys.path[:0]=[str(PC),str(PC/'tools')]
import release_runtime
import factory_workbench


class ReleaseTests(unittest.TestCase):
    def test_configured_serial_bootstrap_restores_payload_without_machine_identity(self):
        import model_transport as transport
        self.assertTrue(hasattr(transport,'DEVICE_SERIAL'))
        configuration=({'serial':'OWN_TEST_BOARD','known_hosts_path':Path('test-hosts'),'host_key_alias':'own-test','key_path':Path('test-key')},'192.168.1.2',Path('ssh'),'NUL')
        source="import json,sys\nprint(json.dumps({'ok':True,'received':json.load(sys.stdin)}))"
        with patch.object(transport,'_configuration',return_value=configuration),patch.object(transport.subprocess,'run',return_value=subprocess.CompletedProcess([],0,b'{}',b'')) as run:
            transport.execute(PC,source,{'value':'own-config'})
        args=run.call_args.args[0];bootstrap=shlex.split(args[-1])[-1]
        self.assertIn('OWN_TEST_BOARD',bootstrap);self.assertLess(len(args[-1]),1000)
        output=io.StringIO()
        with patch.object(sys,'stdin',io.StringIO(run.call_args.kwargs['input'].decode())),patch.object(Path,'read_text',return_value='OWN_TEST_BOARD\n'),contextlib.redirect_stdout(output):
            exec(compile(bootstrap,'publication-offline-bootstrap','exec'),{'__name__':'__main__'})
        self.assertEqual(json.loads(output.getvalue()),{'ok':True,'received':{'value':'own-config'}})

    def test_optional_interpreters_and_legacy_sources_are_bound_without_native_bundles(self):
        import simulation_service
        import bench_calibration_rpc
        import bench_maintenance
        self.assertEqual(simulation_service.PYTHON,Path(sys.executable))
        self.assertTrue(bench_calibration_rpc.REMOTE.startswith('DEVICE_SERIAL = '))
        self.assertTrue(bench_maintenance.REMOTE.startswith('DEVICE_SERIAL = '))

    def test_relative_manifest_and_original_graphs(self):
        manifest=release_runtime.verified_sources(PC)
        expected=json.loads((ROOT/'RELEASE_RUNTIME.json').read_text('utf-8'))
        self.assertEqual(len(manifest['new_source_manifest']),len(expected['runtime_sources'])+len(expected['external_models']))
        self.assertIsNone(manifest['zero_sha256'])
        self.assertNotIn('python/xgoduck_fast_forward.onnx',manifest['new_source_manifest'])
        for name in manifest['new_source_manifest']:
            release_runtime.runtime_source(PC,name,manifest).relative_to(ROOT)

    def test_published_source_provenance_matches(self):
        manifest=json.loads((ROOT/'SOURCE_MANIFEST.json').read_text('utf-8'))
        for name,item in manifest['files'].items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),item['published_sha256'],name)
        self.assertFalse(manifest['actual_device_data_included'])

    def test_disconnected_start_and_gets_never_spawn_hardware_process(self):
        class Tunnel:
            port=None;official=False;error='not_configured'
        with patch.object(subprocess,'Popen',side_effect=AssertionError('Offline test must not launch SSH/worker')):
            server=factory_workbench.FactoryWorkbenchServer(('127.0.0.1',0),Tunnel())
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                for path in ('/','/api/health','/api/bench/status','/assets/official-workbench.js'):
                    c=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=3);c.request('GET',path);reply=c.getresponse();data=reply.read();c.close();self.assertEqual(reply.status,200,path)
                    if path.startswith('/api/') and 'geometry' not in path:
                        state=json.loads(data);self.assertFalse(state['connected']);self.assertTrue(state['startup_read_only']);self.assertTrue(state['baseline']['basic_keyboard_policy']['active']);self.assertFalse(state['baseline']['manual_pick']['active']);self.assertEqual(state['baseline']['release_profile']['schema'],'xgoduck-basic-keyboard-publication-v2')
                self.assertEqual(server.events,[])
            finally:server.shutdown();server.calibration.close();server.server_close();thread.join(timeout=2)

    def test_no_real_device_configuration_or_native_binaries(self):
        for path in PC.rglob('*'):
            if not path.is_file():continue
            self.assertNotIn('.private',path.parts)
            self.assertNotEqual(path.name,'device.local.json')
            self.assertNotIn(path.suffix,('.pyd','.dll','.exe','.bin','.elf','.pt','.log','.pid'))
            if path.suffix=='.py':ast.parse(path.read_text('utf-8-sig'))

    def test_publication_http_guard_and_basic_six_points(self):
        import release_profile as profile
        from official_control_proxy import OfficialServer
        class Tunnel:port=None;official=False;error='not_configured'
        server=OfficialServer(('127.0.0.1',0),Tunnel());thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            for endpoint,body,expected in [('/api/command',{'twist':[.2,0,0]},503),('/api/command',{'twist':[.1,.1,0]},403),('/api/command',{'manual_pick_depth':.05},403),('/api/pick',{},403),('/api/roulade',{},403),('/api/command',{'head':[.2,-.2,.1,0],'mouth':3},403),('/api/mode',{'mode':'off'},503)]:
                connection=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=2)
                connection.request('POST',endpoint,json.dumps(body),{'Content-Type':'application/json'});response=connection.getresponse();response.read();connection.close();self.assertEqual(response.status,expected,(endpoint,body))
            body={'twist':[-.3,0,0]}
            result=profile.constrain_command(body)
            self.assertEqual(result['head'],[0,0,0,0]);self.assertEqual(result['mouth'],0);self.assertEqual(result['twist'],[-.3,0,0])
        finally:server.shutdown();server.server_close();thread.join(timeout=2)

    def test_actual_host_http_functions_reject_motion_before_controller(self):
        from unittest.mock import Mock
        import release_profile
        tree=ast.parse((ROOT/'robot-app/python/main.py').read_text('utf-8'))
        functions=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in ('command','pick','roulade')]
        for node in functions:node.decorator_list=[]
        class HTTPError(Exception):pass
        controller=Mock();namespace={'controller':controller,'HTTPException':HTTPError,'constrain_command':release_profile.constrain_command,'validate_release_action':release_profile.validate_release_action}
        exec(compile(ast.fix_missing_locations(ast.Module(body=functions,type_ignores=[])),'published-host-endpoints','exec'),namespace)
        for endpoint,body in [('command',{'twist':[.2,0,1]}),('command',{'head':[0,0,.1,0]}),('pick',{}),('roulade',{})]:
            with self.assertRaises(HTTPError):namespace[endpoint](body)
        self.assertEqual(controller.mock_calls,[])
        namespace['command']({'twist':[0,0,1]});controller.command.assert_called_once()
        self.assertEqual(controller.command.call_args.kwargs['twist'],[0,0,1])

    def test_passive_current_recorder_uses_own_pinned_serial(self):
        import servo_current_recorder as recorder
        configuration=({'serial':'OWN_TEST_BOARD','known_hosts_path':Path('test-hosts'),'host_key_alias':'own-test','key_path':Path('test-key')},'192.168.1.2',Path('ssh'),'NUL')
        instance=recorder.CurrentRecorder(PC,start=False)
        with patch.object(recorder,'_configuration',return_value=configuration):args=instance._args()
        self.assertIn('OWN_TEST_BOARD',args[-1]);self.assertNotIn('__PORTABLE_DEVICE_SERIAL__',args[-1])


if __name__=='__main__':unittest.main(verbosity=2)
