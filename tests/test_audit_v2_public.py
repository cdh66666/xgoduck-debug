"""Self-contained published V2 contracts; private binaries are deliberately not dependencies.

This checks packaged data/render boundaries, not original binary-source reconstruction.
The separate private-source review summary states its different verification scope.
"""
import json
import hashlib
import http.client
from pathlib import Path
import re
import struct
import subprocess
import sys
import threading
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
PC=ROOT/'pc'
DATA=json.loads((PC/'assets/standing-audit-v2-data.json').read_text('utf-8'))
E=DATA['evidence']
JS=(PC/'assets/standing-audit-v2.js').read_text('utf-8')
HTML=(PC/'assets/standing-system-audit.html').read_text('utf-8')

class PublishedAuditV2Tests(unittest.TestCase):
    def test_browser_static_assets_serve_without_device_processes(self):
        sys.path[:0]=[str(PC),str(PC/'tools')]
        import factory_workbench
        import servo_current_recorder as recorder
        class Disconnected:
            port=None;official=False;error='not_configured'
            def close(self):pass
        names=['assets/standing-system-audit.html','assets/standing-audit-v2.js','assets/standing-audit-v2.css',
               'assets/standing-audit-v2-data.json','model-assets/vendor/d3/d3.min.js']
        with patch.object(recorder,'_configuration',side_effect=ValueError('offline test has no configured device')),patch.object(subprocess,'Popen',side_effect=AssertionError('No SSH/device/worker process allowed')):
            server=factory_workbench.FactoryWorkbenchServer(('127.0.0.1',0),Disconnected())
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                for name in names:
                    connection=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=3)
                    connection.request('GET','/'+name);response=connection.getresponse();body=response.read();connection.close()
                    self.assertEqual(response.status,200,name)
                    self.assertEqual(hashlib.sha256(body).hexdigest(),hashlib.sha256((PC/name).read_bytes()).hexdigest())
                self.assertEqual(server.events,[])
            finally:
                server.shutdown();server.calibration.close();server.server_close();thread.join(timeout=2)

    def test_packaged_dom_dependencies_and_javascript_syntax(self):
        ids=re.findall(r'\bid="([^"]+)"',HTML)
        self.assertIn('场景诊断',HTML)
        self.assertNotIn('场景单变量',HTML)
        self.assertEqual(len(ids),len(set(ids)))
        self.assertFalse(set(re.findall(r"\$\('([^']+)'\)",JS))-set(ids))
        for name in ['assets/standing-audit-v2.css','assets/standing-audit-v2.js',
                     'assets/standing-audit-v2-data.json','model-assets/vendor/d3/d3.min.js',
                     'model-assets/vendor/d3/LICENSE']:
            self.assertTrue((PC/name).is_file(),name)
        result=subprocess.run(['node','--check',str(PC/'assets/standing-audit-v2.js')],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_only_local_static_data_is_requested(self):
        urls=re.findall(r"\bfetch\(['\"]([^'\"]+)",JS)
        self.assertEqual(len(urls),1)
        self.assertEqual(urls[0].split('?')[0],'/assets/standing-audit-v2-data.json')
        self.assertNotRegex(JS,r'WebSocket|XMLHttpRequest|sendBeacon|/api/|method\s*:\s*[\'"]POST')
        self.assertTrue(DATA['readOnly'])
        self.assertFalse(DATA['deviceRequests'])
        self.assertEqual(E['deviceRequests'],0)
        self.assertEqual(E['newExperiments'],0)

    def test_module_edges_interfaces_and_protocol_layout(self):
        ids={m['id'] for m in DATA['modules']}
        self.assertEqual(len(ids),12)
        self.assertEqual(ids,{m['id'] for m in DATA['contracts']['modules']})
        for edge in DATA['edges']:
            self.assertIn(edge['source'],ids);self.assertIn(edge['target'],ids)
        for module in DATA['contracts']['modules']:
            for direction in ('inputs','outputs'):
                self.assertTrue(module[direction])
                for field in module[direction]:
                    self.assertTrue(all(field[k] for k in ('name','unit','shape','source','freshness')))
        for packet in DATA['contracts']['protocols']:
            self.assertEqual(struct.calcsize(packet['format']),packet['bytes'])
            offset=0
            for field in packet['fields']:
                self.assertEqual(field['offset'],offset)
                self.assertEqual(struct.calcsize('<'+field['format']),field['bytes'])
                offset+=field['bytes']
            self.assertEqual(offset,packet['bytes'])

    def test_history_15_axes_enable_state_and_explicit_missing_values(self):
        self.assertEqual([r['id'] for r in E['axes']],list(range(10,15))+list(range(20,25))+list(range(30,35)))
        rows=E['history']['records'];self.assertLessEqual(len(rows),E['history']['maxSamples'])
        self.assertFalse(E['history']['interpolated'])
        missing=negative=disabled=0
        for row in rows:
            self.assertLess(abs(row['t']),86400)
            for name in ('actualDeg','reportedTargetDeg','currentRaw'):self.assertEqual(len(row[name]),15)
            self.assertIsNone(row['appliedGoalDeg']);self.assertIsNone(row['phase'])
            missing+=sum(value is None for value in row['currentRaw'])
            negative+=sum(value is not None and value<0 for value in row['currentRaw'])
            if not row['enabled']:
                disabled+=1;self.assertIsNone(row['activeSoftwareTargetDeg']);self.assertIsNone(row['softwareErrorDeg'])
            else:
                self.assertEqual(row['activeSoftwareTargetDeg'],row['reportedTargetDeg'])
                for actual,target,error in zip(row['actualDeg'],row['reportedTargetDeg'],row['softwareErrorDeg']):
                    self.assertAlmostEqual(error,actual-target,delta=.000501)
        self.assertGreater(missing,0);self.assertGreater(negative,0);self.assertGreater(disabled,0)

    def test_history_gaps_remain_unobserved(self):
        rows=E['history']['records']
        for gap in E['recordingGaps']:
            self.assertGreater(gap['endT'],gap['startT'])
            self.assertFalse(any(gap['startT']<row['t']<gap['endT'] for row in rows))
            self.assertIn('do not infer',gap['meaning'])
        self.assertIn('v==null||!Number.isFinite(v)',JS)
        self.assertRegex(JS,r'r\.t-rows\[i-1\]\.t>\.3')

    def test_simulation_versions_and_acceptance_remain_separate(self):
        scope=E['simulation']['scopeVerified'];integrated=E['simulation']['integrationR1']
        self.assertEqual(len(scope['rows']),6)
        self.assertNotEqual(scope['workerSha256'],integrated['afterWorkerSha256'])
        self.assertEqual(scope['workerSha256'],integrated['beforeWorkerSha256'])
        self.assertTrue(integrated['compiledModelArraysIdentical'])
        self.assertEqual(integrated['pushedRecoveryCasesAcceptedAfter'],0)
        terminal_only=0
        scenarios=scope['rows']+[version for case in integrated['cases'] for version in case['versions'].values()]
        for scenario in scenarios:
            self.assertEqual(scenario['acceptedUnderDeclaredCriterion'],scenario['last2sContinuouslySteady'])
            self.assertFalse(scenario['physicalAccepted'])
            terminal_only+=scenario['terminalSteady'] and not scenario['last2sContinuouslySteady']
        self.assertGreater(terminal_only,0)
        self.assertFalse(integrated['physicalTrainingContractVerified'])

    def test_anonymous_data_and_relative_provenance_metadata(self):
        text=json.dumps(DATA,ensure_ascii=False)
        self.assertNotRegex(text,r'\b[A-Za-z]:[\\/]|/Users/')
        self.assertNotRegex(text,r'\b20\d{2}-\d\d-\d\dT\d\d:\d\d:\d\d')
        self.assertNotIn('\ufffd',text+HTML+JS)
        self.assertFalse(E['timebase']['containsWallClock'])
        self.assertEqual(E['evidenceScope']['physicalStability'],'NOT_PASSED')
        for source in E['provenance']:
            self.assertFalse(Path(source['path']).is_absolute())
            self.assertNotIn('..',Path(source['path']).parts)
            self.assertRegex(source['sha256'],r'^[a-f0-9]{64}$')
        # Private original files are not uploaded or required by public tests.

if __name__=='__main__':unittest.main(verbosity=2)
