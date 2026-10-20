import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"pc"))
"""Offline transaction and cancellation tests; fake transport only."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from bench_calibration import Calibration, IDS
from bench_calibration_rpc import CLIENT, REMOTE


class MaintenanceStub:
    def __init__(self,root):
        self.session_dir=root/'session';self.session_dir.mkdir()
        self.state={'phase':'maintenance','active':True,'session_id':'test-session','image_id':'test-image'}
    def snapshot(self):return copy.deepcopy(self.state)
    def rpc_ready(self):return self.state['phase']=='maintenance'
    def rpc_payload(self):return {'image':'test-image','session_id':'test-session','stage':'test-stage'}
    def update_calibration_state(self,state):
        (self.session_dir/'calibration-state.json').write_text(json.dumps(state),encoding='utf-8')
    def restore(self,state):
        if state['unknown']:raise RuntimeError('unknown ledger')
        self.state.update(phase='restored',active=False)
        return self.snapshot()


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.maintenance=MaintenanceStub(self.root);self.calls=[];self.held=0;self.failure=None
        self.cal=Calibration(self.root,self.maintenance,self.transport)
    def tearDown(self):
        if self.cal.worker:self.cal.worker.join(2)
        self.temp.cleanup()
    def transport(self,root,remote,payload,timeout):
        a=payload['action'];i=payload['id'];self.calls.append((a,i))
        if self.failure and self.failure==(a,i):return {'ok':False,'error':'lost receipt','unknown':True}
        if a=='preflight':return {'ok':True,'held_mask':self.held,'rows':[{'id':i,'torque':0,'mode':4,'lock':1} for i in IDS]}
        if a=='calibrate':self.held|=1<<IDS.index(i)
        if a=='release':self.held&=~(1<<IDS.index(i))
        if a=='stop':self.held=0
        return {'ok':True,'id':i,'position':2048,'target':2048,'torque':int(bool(self.held&(1<<IDS.index(i)))),
                'lock':1,'mode':4,'held_mask':self.held,'saved_zero':2048}
    def run_action(self,action,joint=10):
        self.cal.request(action,joint,True);self.cal.worker.join(3)
        self.assertFalse(self.cal.snapshot()['active'])
        return self.cal.snapshot()
    def test_construction_and_snapshot_do_not_contact_controller(self):
        self.cal.snapshot();self.assertEqual(self.calls,[])
    def test_all_preflight_then_each_axis_saved_once(self):
        state=self.run_action('all')
        self.assertEqual(self.calls,[('preflight',10)]+[('calibrate',i) for i in IDS])
        self.assertEqual(state['completed'],IDS);self.assertFalse(state['unknown'])
        self.assertEqual(state['held_mask'],32767)
        self.assertEqual(len(json.loads((self.maintenance.session_dir/'calibration-state.json').read_text())['saved']),15)
    def test_single_axis_keeps_existing_hold_and_release_affects_only_one(self):
        self.run_action('calibrate',10);self.run_action('calibrate',11)
        state=self.run_action('release',11)
        self.assertEqual(state['held_mask'],1)
        self.assertEqual(self.calls,[('calibrate',10),('calibrate',11),('release',11)])
    def test_lost_receipt_stops_batch_and_restart_preserves_unknown(self):
        self.failure=('calibrate',12);state=self.run_action('all')
        self.assertEqual(state['completed'],[10,11]);self.assertIn('12',state['unknown'])
        self.assertNotIn(('calibrate',13),self.calls)
        restarted=Calibration(self.root,self.maintenance,self.transport)
        self.assertIn('12',restarted.snapshot()['unknown'])
        with self.assertRaises(RuntimeError):restarted.request('all',confirmed=True)
        self.cal=restarted;state=self.run_action('restore')
        self.assertIn('unknown',state['message']);self.assertIn('12',state['unknown'])
    def test_stop_cancels_future_axes_and_unloads_after_inflight_transaction(self):
        entered=threading.Event();finish=threading.Event();original=self.transport
        def delayed(root,remote,payload,timeout):
            if payload['action']=='calibrate' and payload['id']==10:
                entered.set();self.assertTrue(finish.wait(2))
            return original(root,remote,payload,timeout)
        self.cal.transport=delayed;self.cal.request('all',confirmed=True)
        self.assertTrue(entered.wait(2));state=self.cal.request('stop')
        self.assertTrue(state['active']);self.assertIn('取消',state['message'])
        finish.set();self.cal.worker.join(3)
        self.assertEqual([x for x in self.calls if x[0]=='calibrate'],[('calibrate',10)])
        self.assertEqual(self.calls[-1],('stop',10));self.assertEqual(self.held,0)
    def test_all_requires_release_does_not_silently_release(self):
        self.run_action('calibrate',10)
        with self.assertRaises(RuntimeError):self.cal.request('all',confirmed=True)
        self.assertEqual(self.calls,[('calibrate',10)])
    def test_preflight_failure_does_not_write_any_calibration(self):
        self.failure=('preflight',10);self.run_action('all')
        self.assertFalse(any(a=='calibrate' for a,i in self.calls))
    def test_confirmation_and_maintenance_required(self):
        for action in ('enter','restore','all','calibrate'):
            with self.assertRaises(ValueError):self.cal.request(action)
        self.maintenance.state['phase']='official'
        with self.assertRaises(RuntimeError):self.cal.request('calibrate',confirmed=True)


class RemoteSourceTests(unittest.TestCase):
    def test_reused_remote_and_client_compile_offline(self):
        compile(CLIENT,'<client>','exec');compile(REMOTE,'<remote>','exec')
        self.assertIn('ft_calibrate_hold',CLIENT)


if __name__=='__main__':unittest.main()
