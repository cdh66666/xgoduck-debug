import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'robot-app/python'))
from controller import Controller
from factory_calibration import FactoryCalibrationMixin
from tray_guard import is_tray_pose, require_motion_clear, TRAY_REASON


def feedback(**extra):
    # Real unloaded fixture sample, 2026-10-24 18:04:40.
    values = dict(imu_ok=True, mask=0x7fff, enabled=False,
        acc=(-10.230664, .012561, .149536),
        angles=(-.7033,.1758,-.2637,.2637,-.5275,-.3516,1.4066,-1.6703,-2.2857,1.0549,.1758,-.2637,.0879,.4396,0.))
    values.update(extra)
    return SimpleNamespace(**values)


class TrayGuardTests(unittest.TestCase):
    def test_measured_tray_and_nearby_pose_block(self):
        for fb in (feedback(), feedback(angles=(5.,)*15, acc=(-9.8,1.,1.))):
            self.assertTrue(is_tray_pose(fb))
            with self.assertRaisesRegex(ValueError, '托盘'):
                require_motion_clear(fb)

    def test_upright_or_folded_floor_pose_is_different(self):
        self.assertFalse(is_tray_pose(feedback(acc=(0.,0.,-9.8))))
        self.assertFalse(is_tray_pose(feedback(angles=(0.,0.,-50.,0.,50.)*3)))

    def test_reinstalled_joint_and_one_missing_servo_do_not_hide_tray(self):
        # Current 2026-10-25 tray: ID10 absent, ID23 at -136.5 deg after reassembly.
        angles=list(feedback().angles);angles[8]=-136.5275
        self.assertTrue(is_tray_pose(feedback(mask=0x7ffe,angles=angles)))
        with self.assertRaisesRegex(ValueError,'托盘'):
            require_motion_clear(feedback(mask=0x7ffe,angles=angles))

    def test_invalid_samples_are_not_claimed_as_detected_tray(self):
        for fb in (None,feedback(mask=0),feedback(imu_ok=False),feedback(acc=(0,0,0)),feedback(angles=(float('nan'),)*15)):
            self.assertFalse(is_tray_pose(fb))

    def test_start_is_rejected_before_any_bridge_or_model_call(self):
        c=Controller.__new__(Controller)
        c.lock=threading.RLock();c.feedback=feedback();c.mode='off'
        for mode in ('policy','hold'):
            with self.assertRaisesRegex(ValueError,'托盘'):
                c.set_mode(mode)
        self.assertEqual(c.mode,'off')

    def test_detecting_tray_during_motion_invalidates_inflight_action_and_disarms(self):
        calls=[];c=Controller.__new__(Controller)
        c.feedback=feedback(enabled=True);c.mode='policy';c.generation=42
        c.recovery=SimpleNamespace(reset=lambda:calls.append('recovery_reset'))
        c._stop_pick_locked=lambda:calls.append('pick_stop')
        c._stop_roulade_locked=lambda:calls.append('roulade_stop')
        c._keyboard_cancel_locked=lambda:calls.append('keyboard_stop')
        c._disarm_now=lambda:calls.append('DISARM')
        self.assertTrue(c._stop_for_tray_locked())
        self.assertEqual(c.mode,'off');self.assertEqual(c.generation,43)
        self.assertEqual(calls[-1],'DISARM');self.assertEqual(c.last_error,TRAY_REASON)

    def test_factory_calibration_that_would_enable_torque_is_blocked(self):
        c=FactoryCalibrationMixin()
        c.servo_op_lock=threading.Lock();c.feedback=feedback()
        c.mode='servo_debug';c.factory_active=True;c.factory_caps=31
        calls=[];c._factory_request=lambda *args:calls.append(args) or {'ok':True}
        with self.assertRaisesRegex(ValueError,'托盘'):
            c.factory_action('calibrate',22)
        self.assertEqual(calls,[])
        self.assertTrue(c.factory_action('read',22)['ok'])
        self.assertEqual(calls,[('read',22)])


if __name__=='__main__':unittest.main()
