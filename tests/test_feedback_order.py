"""Actual receive/filter behavior for the observed same-boot reordered batch."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'robot-app/python'))
import controller
from wire import ALL_MASK, CURRENT, STATE, decode_state
from current_telemetry import CurrentTelemetry
from display_telemetry import DISPLAY


def frame(seq, *, boot=7, sample_us=None):
    sample_us = seq*10000 if sample_us is None else sample_us
    raw = STATE.pack(b'DQS1',1,1,ALL_MASK,seq,sample_us,sample_us,0,0,10000,1000,0,0,0,
                     0,0,16384,0,0,0,*([0.0]*45))
    return raw+CURRENT.pack(b'DQI1',1,0,0,boot&0xffffffff,boot>>32,seq,sample_us,sample_us,
                            0,0,*([0]*15),0,0)


def display_frame(seq,boot=7):
    return DISPLAY.pack(b'DQD1',2,59,10,82,boot&0xffffffff,boot>>32,seq,seq*10000,seq*10000,1,
                        *([0,0,0,1]*2),0,0,-1,0,seq*10000,*([0]*7),0,0,0,201)


class FeedbackOrderTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.bridge = Mock()
        telemetry = CurrentTelemetry(self.temporary.name, start_writer=False)
        self.addCleanup(telemetry.close)
        with (patch.object(controller.Policy,'load',return_value=(True,'offline test')),
             patch.object(controller,'ZERO_PATH',Path(self.temporary.name)/'zero.json'),
             patch.object(controller,'CurrentTelemetry',return_value=telemetry)):
            self.c = controller.Controller(self.bridge)
        self.c.mode = 'off'

    def test_reordered_batch_does_not_reset_filter_or_stationary_fit(self):
        for seq in range(1,260):self.c.receive(frame(seq))
        self.assertTrue(self.c.gyro_bias_control.ready)
        before=(self.c.received,self.c.rx_time,self.c._feedback_clock_s,self.c.generation)
        for seq in range(233,259):self.c.receive(frame(seq))
        self.assertEqual((self.c.received,self.c.rx_time,self.c._feedback_clock_s,self.c.generation),before)
        self.assertEqual(self.c.mode,'off')
        self.assertTrue(self.c.gyro_bias_control.ready)
        self.assertEqual(self.c.out_of_order_frames,26)
        self.c.receive(frame(260))
        self.assertTrue(self.c.gyro_bias_control.ready)
        self.assertEqual(self.c.gaps,0)
        self.bridge.notify.assert_not_called()

    def test_real_boot_change_still_resets_and_old_boot_cannot_replace_it(self):
        self.c.receive(frame(259))
        self.c.mode='policy';self.c.arm_pending=True
        self.c.receive(frame(1,boot=8))
        self.assertEqual(self.c.mode,'shadow')
        self.assertFalse(self.c.arm_pending)
        self.assertFalse(self.c._zero_sync_confirmed())
        self.assertFalse(self.c.gyro_bias_control.ready)
        self.assertEqual(self.c.feedback.boot,8)
        self.c.receive(frame(260,boot=7))
        self.assertEqual(self.c.feedback.boot,8)
        self.assertEqual(self.c.out_of_order_frames,1)
        self.bridge.notify.assert_not_called()

    def test_sequence_wrap_keeps_same_boot_and_forward_clock(self):
        self.c.receive(frame(0xffffffff,sample_us=10000))
        self.c.receive(frame(0,sample_us=20000))
        self.assertEqual(self.c.mode,'off')
        self.assertEqual(self.c.generation,0)
        self.assertEqual(self.c.gaps,0)
        self.assertAlmostEqual(self.c._feedback_clock_s,.01)

    def test_reused_current_boot_marker_recovers_after_fresh_progress(self):
        with patch.object(controller.time,'monotonic',return_value=10):
            self.c.receive(frame(1000))
        self.c.mode='policy';self.c.arm_pending=True
        for seq in range(1,19):
            with patch.object(controller.time,'monotonic',return_value=10.01+seq*.01):
                self.c.receive(frame(seq))
        self.assertEqual(self.c.feedback.seq,18)
        self.assertEqual(self.c.mode,'shadow')
        self.assertFalse(self.c.arm_pending)
        self.assertFalse(self.c.gyro_bias_control.ready)
        self.assertFalse(self.c._zero_sync_confirmed())
        self.bridge.notify.assert_not_called()

    def test_retired_marker_can_be_reused_by_an_actual_advancing_restart(self):
        with patch.object(controller.time,'monotonic',return_value=10):
            self.c.receive(frame(1000,boot=7))
            self.c.receive(frame(1,boot=8))
            self.c.receive(frame(1000,boot=8))
        generation=self.c.generation
        for seq in range(1,19):
            with patch.object(controller.time,'monotonic',return_value=10.01+seq*.01):
                self.c.receive(frame(seq,boot=7))
        self.assertEqual(self.c.feedback.boot,7)
        self.assertEqual(self.c.feedback.seq,18)
        self.assertEqual(self.c.generation,generation+1)
        self.assertNotIn(7,self.c._retired_feedback_boots)

    def test_compressed_old_batch_cannot_claim_restart(self):
        with patch.object(controller.time,'monotonic',return_value=10):
            self.c.receive(frame(1000))
        for seq in range(1,101):
            with patch.object(controller.time,'monotonic',return_value=10.2+seq*.00001):
                self.c.receive(frame(seq))
        self.assertEqual(self.c.feedback.seq,1000)
        self.assertEqual(self.c.mode,'off')
        self.assertEqual(self.c.generation,0)

    def test_forward_sequence_after_disconnect_cannot_reuse_frozen_fit_or_zeros(self):
        with patch.object(controller.time,'monotonic',return_value=10):
            for seq in range(1,260):self.c.receive(frame(seq))
        self.c.gyro_bias_control.freeze(epoch=self.c._gyro_control_epoch,now=10)
        self.c._zeros_pushed=True;self.c._mcu_zero_pos=list(self.c.zero_pos);self.c._zero_push_at=10
        self.c.mode='policy';self.c.arm_pending=True
        with patch.object(controller.time,'monotonic',return_value=10.2):self.c.step()
        generation=self.c.generation
        with patch.object(controller.time,'monotonic',return_value=14):self.c.receive(frame(300))
        self.assertEqual(self.c.feedback.seq,300)
        self.assertEqual(self.c.mode,'shadow')
        self.assertEqual(self.c.generation,generation+1)
        self.assertIsNone(self.c.gyro_bias_control.frozen)
        self.assertFalse(self.c.gyro_bias_control.ready)
        self.assertFalse(self.c._zero_sync_confirmed())

    def test_forward_gap_preserves_explicit_disabled_off_and_recollects(self):
        with patch.object(controller.time,'monotonic',return_value=10):
            for seq in range(1,260):self.c.receive(frame(seq))
        self.c.gyro_bias_control.freeze(epoch=self.c._gyro_control_epoch,now=10)
        with patch.object(controller.time,'monotonic',return_value=14):self.c.receive(frame(300))
        self.assertEqual(self.c.mode,'off')
        self.assertIsNone(self.c.gyro_bias_control.frozen)
        self.assertFalse(self.c._zero_sync_confirmed())
        self.assertFalse(self.c.gyro_bias_control.ready)
        for seq in range(301,560):
            with patch.object(controller.time,'monotonic',return_value=14+(seq-300)*.01):self.c.receive(frame(seq))
        self.assertTrue(self.c.gyro_bias_control.ready)
        self.assertIsNone(self.c.gyro_bias_control.frozen)
        self.bridge.notify.assert_not_called()

    def test_reused_marker_resync_preserves_explicit_off(self):
        with patch.object(controller.time,'monotonic',return_value=10):
            self.c.receive(frame(1000))
        for seq in range(1,19):
            with patch.object(controller.time,'monotonic',return_value=10.01+seq*.01):
                self.c.receive(frame(seq))
        self.assertEqual(self.c.feedback.seq,18)
        self.assertEqual(self.c.mode,'off')
        self.assertFalse(self.c.arm_pending)
        self.assertFalse(self.c._zero_sync_confirmed())
        for seq in range(19,280):
            with patch.object(controller.time,'monotonic',return_value=10.01+seq*.01):
                self.c.receive(frame(seq))
        self.assertTrue(self.c.gyro_bias_control.ready)
        self.bridge.notify.assert_not_called()

    def test_new_boot_preserves_off_but_withdraws_old_qualification(self):
        with patch.object(controller.time,'monotonic',return_value=10):
            for seq in range(1,260):self.c.receive(frame(seq))
            self.c.gyro_bias_control.freeze(epoch=self.c._gyro_control_epoch,now=10)
            self.c._zeros_pushed=True;self.c._mcu_zero_pos=list(self.c.zero_pos)
            self.c.receive(frame(1,boot=8))
        self.assertEqual(self.c.mode,'off')
        self.assertIsNone(self.c.gyro_bias_control.frozen)
        self.assertFalse(self.c.gyro_bias_control.ready)
        self.assertFalse(self.c._zero_sync_confirmed())
        self.assertFalse(self.c.arm_pending)
        self.bridge.notify.assert_not_called()

    def test_current_suffix_boot_is_validated_and_legacy_stays_supported(self):
        raw=frame(7,boot=0x123456789abcdef0)
        self.assertEqual(decode_state(raw).boot,0x123456789abcdef0)
        self.assertIsNone(decode_state(raw[:STATE.size]).boot)
        with self.assertRaises(ValueError):decode_state(raw[:-1])

    def _check_consumer_recovery(self,retire):
        telemetry=self.c.current_telemetry
        def drain():
            while not telemetry.queue.empty():
                telemetry._consume(telemetry.queue.get_nowait());telemetry.queue.task_done()
        with patch.object(controller.time,'monotonic',return_value=10):
            self.c.receive(frame(1000));self.c.receive_display(display_frame(1000))
            drain();old_item=(frame(1001),1001,10.,0,0)
            if retire:
                self.c.receive(frame(1,boot=8));self.c.receive(frame(1000,boot=8))
                self.c.receive_display(display_frame(1000,boot=8));drain()
        for seq in range(1,19):
            with patch.object(controller.time,'monotonic',return_value=10.01+seq*.01):self.c.receive(frame(seq))
        drain()
        self.assertEqual(telemetry._seq,18)
        self.assertEqual(telemetry._boot,7)
        telemetry._consume(old_item)
        self.assertEqual(telemetry._seq,18)  # A late old queued epoch cannot pollute it.
        self.c.receive_display(display_frame(1001,boot=8 if retire else 7))
        self.assertIsNone(self.c.display_telemetry.value)
        self.c.receive_display(display_frame(18))
        self.assertEqual(self.c.display_telemetry.value['source_seq'],18)
        self.assertEqual(self.c.display_telemetry.value['boot_epoch'],'0000000000000007')
        self.assertTrue(self.c.display_telemetry.snapshot(self.c.feedback,self.c.rx_time,self.c.rx_time)['display_heading']['available'])
        self.bridge.notify.assert_not_called()

    def test_reused_marker_recovery_reaches_current_and_heading_consumers(self):
        self._check_consumer_recovery(False)

    def test_retired_marker_recovery_reaches_current_and_heading_consumers(self):
        self._check_consumer_recovery(True)


if __name__=='__main__':unittest.main(verbosity=2)
