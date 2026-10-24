"""Exercise actual board dispatch: coalescing, stop ordering and expiry."""
import unittest
from unittest.mock import patch
import test_remote_disconnect as fixtures
import controller


class ControlMailboxTests(unittest.TestCase):
    receive=fixtures.RemoteDisconnectTests.receive
    step=fixtures.RemoteDisconnectTests.step

    def setUp(self):
        fixtures.RemoteDisconnectTests.setUp(self)
        self.c.web_time=11.01
        self.c.twist=[0.,0.,0.]

    def submit(self,twist,now=11.015):
        with patch.object(controller.time,'monotonic',return_value=now):
            return self.c.submit_command(twist=twist)

    def test_latest_release_replaces_movement_without_running_in_http_caller(self):
        before=self.bridge.notify.call_count
        first=self.submit([.2,0,0])
        last=self.submit([0,0,0])
        self.assertFalse(first['applied'])
        self.assertEqual(self.c.twist,[0,0,0])
        self.assertEqual(self.bridge.notify.call_count,before)
        self.step(11.02)
        self.assertEqual(self.infer.call_args.args[4],[0,0,0])
        self.assertEqual(self.c.active_intent_seq,last['accepted_seq'])
        self.assertEqual(self.c.intent_overwritten,1)
        self.assertAlmostEqual(self.c.intent_result['wait_ms'],5.)

    def test_command_enters_next_policy_tick_and_uses_receipt_time(self):
        reply=self.submit([.2,0,0])
        self.assertEqual(self.c.twist,[0,0,0])
        self.step(11.02)
        self.assertEqual(self.infer.call_args.args[4],[.2,0,0])
        self.assertEqual(self.c.intent_result['sequence'],reply['accepted_seq'])
        self.assertEqual(self.c.web_time,11.015)

    def test_explicit_off_cancels_unconsumed_movement(self):
        reply=self.submit([.2,0,0])
        with patch.object(controller.time,'monotonic',return_value=11.017):self.c.set_mode('off')
        self.assertIsNone(self.c.pending_intent)
        self.step(11.02)
        self.assertEqual(self.c.mode,'off')
        self.assertEqual(self.c.twist,[0,0,0])
        self.assertEqual(self.c.intent_result,dict(sequence=reply['accepted_seq'],applied=False,error='mode_changed'))

    def test_stale_intent_does_not_renew_its_lease(self):
        self.submit([.2,0,0],now=9.)
        self.step(11.02)
        self.assertEqual(self.c.intent_result['error'],'intent_expired')
        self.assertEqual(self.c.web_time,11.01)
        self.assertEqual(self.c.twist,[0,0,0])

    def test_invalid_request_is_rejected_before_occupying_mailbox(self):
        with self.assertRaises(ValueError):self.submit([.2,0,1])
        self.assertIsNone(self.c.pending_intent)
        self.assertEqual(self.c.intent_sequence,0)


if __name__=='__main__':unittest.main()
