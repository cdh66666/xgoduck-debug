import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
import threading
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pc'))
from firmware_manager import FirmwareManager

class FirmwareManagerTests(unittest.TestCase):
    def test_stale_poll_cannot_unlock_a_new_update(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'firmware-release.json').write_text(json.dumps({'version':'v1'}))
            (p/'firmware_install_remote.py').write_text('# fixed installer')
            entered=threading.Event();resume=threading.Event()
            def remote(root,source,payload,**kw):
                if payload['operation']=='read':
                    entered.set();self.assertTrue(resume.wait(3))
                    return {'connected':True,'job':None}
                return {'phase':'queued','job':payload['job']}
            m=FirmwareManager(p,remote)
            m.value={'connected':True,'checked_at':time.time(),'candidate':{'ready':True},'state':{'mode':'off','enabled':False}}
            poll=threading.Thread(target=m.refresh);poll.start();self.assertTrue(entered.wait(3))
            try:m.update('v1')
            finally:resume.set();poll.join(3)
            self.assertFalse(poll.is_alive());self.assertTrue(m.snapshot()['busy'])
            with self.assertRaisesRegex(ValueError,'进行'):m.update('v1')

    def test_unknown_job_remains_locked_after_reconnect(self):
        with tempfile.TemporaryDirectory() as d:
            m=FirmwareManager(d,lambda *args,**kwargs:{'connected':True,'job':None})
            m.job='pending-job';result=m.refresh()
            self.assertTrue(result['busy']);self.assertIn('未确认',result['error'])

    def test_refuses_motion_stale_state_and_changed_candidate(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'firmware-release.json').write_text(json.dumps({'version':'v1'}))
            calls=[];m=FirmwareManager(p,lambda *a,**k:calls.append(a))
            m.value={'connected':True,'checked_at':time.time(),'candidate':{'ready':True},'state':{'mode':'policy','enabled':True}}
            with self.assertRaisesRegex(ValueError,'卸力'):m.update('v1')
            m.value['state']={'mode':'off','enabled':False};m.value['checked_at']=0
            with self.assertRaisesRegex(ValueError,'读取'):m.update('v1')
            with self.assertRaisesRegex(ValueError,'变化'):m.update('old')
            self.assertEqual(calls,[])

    def test_lost_reply_keeps_job_and_is_not_replayed(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'firmware-release.json').write_text(json.dumps({'version':'v1'}));(p/'firmware_install_remote.py').write_text('# fixed installer')
            calls=[]
            def remote(root,source,payload,**kw):
                calls.append(payload)
                if payload['operation']=='update':raise TimeoutError('unknown outcome')
                return {'connected':True,'job':{'phase':'starting_disabled'}}
            m=FirmwareManager(p,remote);m.value={'connected':True,'checked_at':time.time(),'candidate':{'ready':True},'state':{'mode':'off','enabled':False}}
            with self.assertRaises(TimeoutError):m.update('v1')
            saved=json.loads(m.job_file.read_text());self.assertEqual(saved['job'],m.job)
            m.refresh()
            with self.assertRaisesRegex(ValueError,'进行'):m.update('v1')
            self.assertEqual([x['operation'] for x in calls],['update','read'])
            self.assertEqual(calls[0]['job'],calls[1]['job'])

if __name__=='__main__':unittest.main()
