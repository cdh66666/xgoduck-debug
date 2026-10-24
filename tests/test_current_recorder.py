"""Exercise the actual binary copy and CSV export without network access."""
import base64
import csv
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import http.client
import json
import threading
import subprocess
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/'pc'))
sys.path.insert(0, str(HERE))
import servo_current_recorder as pc
import importlib.util
spec=importlib.util.spec_from_file_location('release_wire',HERE.parent/'robot-app/python/wire.py')
wire=importlib.util.module_from_spec(spec);sys.modules[spec.name]=wire;spec.loader.exec_module(wire)
def frame(seq=1,start=10000,*,mask=32767,raw=None,boot=7,attempted=True,skipped=0):
    base=wire.STATE.pack(b'DQS1',1,1,32767,seq,start,start,0,0,10000,6000,0,0,0,100,-100,16384,32,-32,0,*[i/10 for i in range(45)])
    tail=wire.CURRENT.pack(b'DQI1',1,int(attempted),mask,boot&0xffffffff,boot>>32,seq,start,(start+1200)&0xffffffff,seq,skipped,*(raw or [100]*15),0,(start-100)&0xffffffff)
    return base+tail


class CopyTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.recorder = pc.CurrentRecorder(self.temporary.name, start=False)

    def packet(self, *, generation=100, mask=32767, seq=1, offset=32):
        header = pc.HEADER.pack(b'DQICLOG1', 1, 32, 336, generation, generation)
        payload = frame(seq=seq, mask=mask, raw=[0x8001, 2] + [0] * 13)
        raw = pc.RECORD.pack(b'DQIR', 1, 0, 123000000000, 234000000000, payload)
        return dict(slot=0, header=header.hex(), offset=offset, data=base64.b64encode(raw).decode())

    def test_reconnect_replay_preserves_original_bytes_without_growing_file(self):
        packet = self.packet()
        self.recorder.accept(packet)
        path = next(self.recorder.folder.glob('*.bin'))
        before = path.read_bytes()
        self.recorder.accept(packet)
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(len(before), 368)
        self.assertTrue(self.recorder.snapshot()['recording'])

    def test_new_generation_kept_and_missing_prefix_rejected(self):
        self.recorder.accept(self.packet())
        self.recorder.accept(self.packet(generation=200))
        self.assertEqual(len(list(self.recorder.folder.glob('*.bin'))), 2)
        with self.assertRaises(ValueError):
            self.recorder.accept(self.packet(generation=300, offset=368))

    def test_partial_csv_has_blank_sum_and_blank_missing_axes(self):
        self.recorder.accept(self.packet(mask=1))
        rows = list(csv.DictReader(io.StringIO(pc.export_csv(self.recorder.folder).decode('utf-8-sig'))))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['complete_15'], '0')
        self.assertEqual(rows[0]['servo_sum_abs_a_not_battery_total'], '')
        self.assertEqual(rows[0]['id_10_a'], '-0.0065')
        self.assertEqual(rows[0]['id_11_a'], '')

    def test_full_csv_magnitudes_not_cancelled_and_raw_kept(self):
        self.recorder.accept(self.packet())
        row = next(csv.DictReader(io.StringIO(pc.export_csv(self.recorder.folder).decode('utf-8-sig'))))
        self.assertAlmostEqual(float(row['servo_sum_abs_a_not_battery_total']), .0195)
        self.assertAlmostEqual(float(row['servo_sum_signed_a']), .0065)
        self.assertEqual(row['id_10_raw'], '32769')
        self.assertEqual(row['host_unix_s'], '234.0')

    def test_malformed_block_does_not_create_log(self):
        packet = self.packet()
        packet['data'] = base64.b64encode(b'bad').decode()
        with self.assertRaises(ValueError):
            self.recorder.accept(packet)
        self.assertFalse(self.recorder.folder.exists())

    def test_series_preserves_partial_sample_as_gap_and_source_rate(self):
        header = pc.HEADER.pack(b'DQICLOG1', 1, 32, 336, 100, 100)
        records = []
        for index, mask in enumerate((32767, 1, 32767)):
            payload = frame(seq=index+1, start=10000*(index+1), mask=mask)
            records.append(pc.RECORD.pack(b'DQIR', 1, 0, 1000000000+index*10000000,
                234000000000+index*10000000, payload))
        self.recorder.accept(dict(slot=0, header=header.hex(), offset=32,
            data=base64.b64encode(b''.join(records)).decode()))
        result = pc.current_series(self.recorder.folder)
        self.assertEqual(len(result['samples']), 3)
        self.assertIsNone(result['samples'][1]['sum_abs_a'])
        self.assertIsNone(result['samples'][1]['values'][1])
        self.assertEqual(result['full_samples'], 2)
        self.assertEqual(result['partial_samples'], 1)
        self.assertAlmostEqual(result['sample_hz'], 50)
        self.assertTrue(result['stale'])
        self.assertFalse(result['battery_total'])

    def test_actual_http_curve_and_csv_use_only_local_synthetic_binary_log(self):
        import factory_workbench
        self.recorder.accept(self.packet())
        class Tunnel:port=None;official=False;error='not_configured'
        with (patch.object(factory_workbench,'CurrentRecorder',return_value=self.recorder),
              patch.object(subprocess,'Popen',side_effect=AssertionError('No SSH or worker in offline curve test'))):
            server=factory_workbench.FactoryWorkbenchServer(('127.0.0.1',0),Tunnel())
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                with patch.object(factory_workbench,'ROOT',Path(self.temporary.name)):
                    for endpoint in ('/api/bench/current-series','/api/bench/current-log.csv'):
                        connection=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=2);connection.request('GET',endpoint)
                        response=connection.getresponse();body=response.read();connection.close();self.assertEqual(response.status,200)
                        if endpoint.endswith('series'):
                            value=json.loads(body);self.assertEqual(len(value['samples']),1);self.assertFalse(value['battery_total']);self.assertAlmostEqual(value['samples'][0]['sum_abs_a'],.0195)
                        else:self.assertEqual(body,pc.export_csv(self.recorder.folder))
            finally:server.shutdown();server.calibration.close();server.server_close();thread.join(timeout=2)

    def test_resume_positions_use_existing_complete_prefix(self):
        packet=self.packet();self.recorder.accept(packet)
        self.assertEqual(self.recorder.resume_positions(),[dict(slot=0,header=packet['header'],offset=368)])
        path=next(self.recorder.folder.glob('*.bin'))
        with path.open('ab') as stream:stream.write(b'partial')
        self.assertEqual(self.recorder.resume_positions()[0]['offset'],368)

    def test_resume_keeps_rotated_generations_distinct(self):
        self.recorder.accept(self.packet(generation=100));self.recorder.accept(self.packet(generation=200))
        rows=self.recorder.resume_positions()
        self.assertEqual(len(rows),2);self.assertNotEqual(rows[0]['header'],rows[1]['header'])
        self.assertTrue(all(row['offset']==368 for row in rows))


if __name__ == '__main__':
    unittest.main(verbosity=2)
