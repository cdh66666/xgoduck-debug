"""Regression for the measured 32/33/34 dropout during live get-up."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robot-app/python'))
import controller
from wire import ALL_MASK, CURRENT, STATE, COMMAND, DISARM, HOLD


def frame(seq,mask=ALL_MASK,boot=7):
    sample=seq*10000
    raw=STATE.pack(b'DQS1',1,3,mask,seq,sample,sample,0,0,10000,1000,0,0,0,
                   0,16384,0,0,0,0,*([0.]*45))
    return raw+CURRENT.pack(b'DQI1',1,0,0,boot,0,seq,sample,sample,0,0,*([0]*15),0,0)


class JointFeedbackTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        self.bridge=Mock();telemetry=Mock()
        with (patch.object(controller.Policy,'load',return_value=(True,'offline')),
              patch.object(controller,'ZERO_PATH',Path(temporary.name)/'zero.json'),
              patch.object(controller,'CurrentTelemetry',return_value=telemetry)):
            self.c=controller.Controller(self.bridge)
        inference=patch.object(controller.Policy,'infer',return_value=[0.]*14)
        inference.start();self.addCleanup(inference.stop)
        self.c.mode='off'
        self.receive(1,10.)
        self.c._zeros_pushed=True;self.c._mcu_zero_pos=list(self.c.zero_pos)

    def receive(self,seq,now,mask=ALL_MASK,boot=7):
        with patch.object(controller.time,'monotonic',return_value=now):
            self.c.receive(frame(seq,mask,boot))

    def step(self,now):
        self.c.web_time=now
        with patch.object(controller.time,'monotonic',return_value=now):self.c.step()

    def test_start_requires_all_15_even_with_live_imu_and_matching_zeros(self):
        self.receive(2,10.01,0xfff)
        with patch.object(controller.time,'monotonic',return_value=10.01):
            with self.assertRaisesRegex(ValueError,'required servo feedback'):
                self.c.set_mode('hold')
        self.assertEqual(self.c.mode,'off')
        self.bridge.notify.assert_not_called()

    def test_short_partial_packet_does_not_abort_healthy_control(self):
        self.c.mode='hold'
        self.receive(2,10.05,0xfff);self.step(10.14)
        self.assertEqual(self.c.mode,'hold')
        self.assertEqual(COMMAND.unpack(self.bridge.notify.call_args.args[1])[4],HOLD)

    def test_sustained_tail_dropout_disarms_despite_live_packets_and_imu(self):
        self.c.mode='hold'
        for seq in range(2,18):self.receive(seq,10.+seq*.01,0xfff)
        self.step(10.18)
        self.assertEqual(self.c.mode,'shadow')
        self.assertEqual(self.c.last_error,'servo feedback lost: 32,33,34; disarmed')
        self.assertEqual(COMMAND.unpack(self.bridge.notify.call_args.args[1])[4],DISARM)
        with patch.object(controller.time,'monotonic',return_value=10.18):s=self.c.status()
        self.assertFalse(s['control_ready']);self.assertEqual(s['stale_joint_ids'],[32,33,34])

    def test_joint_return_never_rearms_the_controller(self):
        self.c.mode='hold'
        self.receive(2,10.14,0xfff);self.step(10.16)
        self.receive(3,10.17);self.step(10.18)
        self.assertEqual(self.c.mode,'shadow')
        self.assertEqual(COMMAND.unpack(self.bridge.notify.call_args.args[1])[4],DISARM)

    def test_new_boot_cannot_inherit_old_joint_receipt_times(self):
        self.receive(1,10.01,0xfff,boot=8)
        self.assertEqual(self.c._stale_joint_ids(10.01),[32,33,34])

    def test_dropout_arriving_during_inference_cannot_send_an_action(self):
        self.c.mode='policy';self.c.policy_needs_reset=False
        self.receive(2,10.14,0xfff)
        def inference(*args):
            self.c._joint_rx_times[12:]=[9.]*3
            return [0.]*14
        with patch.object(self.c.policy,'infer',side_effect=inference):self.step(10.14)
        self.assertEqual(COMMAND.unpack(self.bridge.notify.call_args.args[1])[4],DISARM)
        self.assertEqual(self.c.mode,'shadow')
        self.receive(3,10.17);self.step(10.18)
        self.assertEqual(self.c.mode,'shadow')
        self.assertEqual(COMMAND.unpack(self.bridge.notify.call_args.args[1])[4],DISARM)

    def test_recorded_getup_yaw_outputs_cannot_turn_behind_the_head(self):
        self.c.mode='policy';self.c.policy_needs_reset=False
        for requested in [-144.3607178,94.8172836]:
            target=[0.]*15;target[12]=requested;target[3]=22.
            with patch.object(self.c.policy,'action_to_deg',return_value=target):self.step(10.01)
            sent=COMMAND.unpack(self.bridge.notify.call_args.args[1])[7:]
            self.assertLessEqual(abs(sent[12]),90.)
            self.assertEqual(sent[3],22.)
            self.assertEqual(self.c.last_action[12],sent[12])

    def test_sit_with_one_invalid_fusion_update_holds_last_target(self):
        self.c.mode='policy';self.c.policy_needs_reset=False;self.c.sit_active=True
        goal=[0.,0.,54.,83.,29.,0.,0.,-54.,-83.,-29.,20.,20.,0.,0.,0.]
        self.c.last_action=list(goal);self.c.filtered['imu_fusion_updated']=False
        self.step(10.01)
        sent=COMMAND.unpack(self.bridge.notify.call_args.args[1])[7:]
        self.assertEqual(tuple(sent),tuple(goal))
        self.assertEqual(self.c.mode,'policy')


if __name__=='__main__':unittest.main(verbosity=2)
