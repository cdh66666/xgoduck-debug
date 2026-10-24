import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"pc"))
import json
from pathlib import Path
import tempfile
import unittest

from bench_calibration import FactoryCalibration, FactoryMaintenance, IDS


class FactoryCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.calls = []
        self.m = FactoryMaintenance(self.root)
        self.c = FactoryCalibration(self.root,self.m,self.call)

    def tearDown(self):
        self.temporary.cleanup()

    def call(self, action, joint):
        self.calls.append((action,joint))
        value = dict(ok=True,position=2048,zero_pos=2048,torque=0,
                    locked=1,mode=4,held_mask=0,host_zero_saved=True)
        if action == 'read_all':value['servos']=[dict(value,id=i) for i in IDS]
        return value

    def run_action(self, action, joint=10):
        self.c.request(action,joint,True)
        self.c.worker.join(2)
        self.assertFalse(self.c.worker.is_alive())

    def test_opening_never_commands_or_restores_historical_holds(self):
        self.assertEqual(self.calls,[])
        self.assertFalse(self.m.snapshot()['active'])
        self.assertFalse(self.m.snapshot()['firmware_switch'])

    def test_single_axis_does_not_command_other_axes(self):
        self.run_action('calibrate',22)
        self.assertEqual(self.calls,[('calibrate',22)])
        self.assertEqual(self.c.snapshot()['saved']['22']['zero'],2048)

    def test_all_checks_every_axis_before_first_write(self):
        self.run_action('all')
        self.assertEqual(self.calls[0],('read_all',10))
        self.assertEqual(self.calls[1:],[('calibrate',i) for i in IDS])
        self.assertEqual(self.c.snapshot()['completed'],IDS)

    def test_failed_write_stays_unknown_across_restart(self):
        def fail(action,joint):
            self.assertIn(str(joint),json.loads(self.c.path.read_text('utf-8'))['unknown'])
            raise TimeoutError('reply lost')
        self.c.call = fail
        self.run_action('calibrate',11)
        fresh = FactoryCalibration(self.root,self.m,self.call)
        self.assertIn('11',fresh.snapshot()['unknown'])
        with self.assertRaises(RuntimeError):fresh.request('calibrate',11,True)

    def test_cancel_batch_stops_later_axes_and_releases_once(self):
        def cancel(action,joint):
            value = self.call(action,joint)
            if action == 'calibrate':self.c.cancel.set()
            return value
        self.c.call = cancel
        self.run_action('all')
        self.assertEqual([r for r in self.calls if r[0]=='calibrate'],[('calibrate',10)])
        self.assertEqual(self.calls[-1],('off',10))

    def test_restart_does_not_infer_current_holds_from_receipt(self):
        self.run_action('calibrate')
        value=json.loads(self.c.path.read_text('utf-8'));value['held_mask']=32767
        self.c.path.write_text(json.dumps(value),encoding='utf-8')
        fresh=FactoryCalibration(self.root,FactoryMaintenance(self.root),self.call)
        self.assertEqual(fresh.snapshot()['held_mask'],0)
        self.assertFalse(fresh.maintenance.active)

    def test_release_preserves_completed_batch_and_its_timing(self):
        self.run_action('all')
        original=self.c.snapshot()['last_batch']
        self.run_action('release',34)
        self.assertEqual(self.c.snapshot()['last_batch'],original)
        fresh=FactoryCalibration(self.root,FactoryMaintenance(self.root),self.call)
        self.assertEqual(fresh.snapshot()['last_batch']['completed'],IDS)

    def test_stored_receipts_remain_history_after_entering(self):
        self.run_action('all')
        fresh = FactoryCalibration(self.root,FactoryMaintenance(self.root),self.call)
        self.assertEqual(fresh.snapshot()['prior_saved_count'],15)
        self.assertEqual(fresh.snapshot()['current_saved_ids'],[])
        self.assertIsNone(fresh.snapshot()['current_batch_completed'])
        fresh.request('enter',10,True)
        fresh.worker.join(2)
        self.assertEqual(fresh.snapshot()['current_saved_ids'],[])
        self.assertEqual(len(fresh.snapshot()['saved']),15)
        self.assertIsNone(fresh.snapshot()['current_batch_completed'])

    def test_only_new_confirmed_successes_count_and_survive_read(self):
        self.run_action('calibrate',22)
        self.run_action('read',22)
        self.assertEqual(self.c.snapshot()['current_saved_ids'],[22])
        self.assertEqual(self.c.snapshot()['prior_saved_count'],0)
        fresh = FactoryCalibration(self.root,self.m,self.call)
        self.assertEqual(fresh.snapshot()['current_saved_ids'],[])
        self.assertEqual(fresh.snapshot()['prior_saved_count'],1)

    def test_unconfirmed_host_save_does_not_count(self):
        self.c.call = lambda action,joint: dict(ok=True,position=2048,host_zero_saved=False)
        self.run_action('calibrate',22)
        self.assertEqual(self.c.snapshot()['current_saved_ids'],[])
        self.assertIn('22',self.c.snapshot()['unknown'])

    def test_new_batch_count_is_separate_from_durable_history(self):
        self.assertIsNone(self.c.snapshot()['current_batch_completed'])
        self.run_action('all')
        self.assertEqual(self.c.snapshot()['current_batch_completed'],IDS)
        self.assertEqual(self.c.snapshot()['current_saved_ids'],IDS)
        self.run_action('release',34)
        self.assertEqual(self.c.snapshot()['current_batch_completed'],IDS)


if __name__ == '__main__':unittest.main()
