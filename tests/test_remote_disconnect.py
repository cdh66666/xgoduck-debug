"""PC intent can expire without interrupting the board's local balance loop."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot-app/python'))
import controller
from wire import ALL_MASK, CURRENT, STATE, COMMAND, DISARM, POLICY
from rl_core import DEFAULT_DEG


def frame(seq, *, enabled=True, mask=ALL_MASK, imu_ok=True):
    sample = seq * 10000
    flags = int(imu_ok) | (2 if enabled else 0)
    raw = STATE.pack(b'DQS1', 1, flags, mask, seq, sample, sample, 0, 0,
                     10000, 1000, 0, 0, 0, 0, 0, -16384, 0, -256, 0,
                     *DEFAULT_DEG, *([0.] * 15), *DEFAULT_DEG)
    return raw + CURRENT.pack(b'DQI1', 1, 0, 0, 7, 0, seq, sample, sample,
                              0, 0, *([0] * 15), 0, 0)


class RemoteDisconnectTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.bridge = Mock()
        with (patch.object(controller.Policy, 'load', return_value=(True, 'offline')),
              patch.object(controller, 'ZERO_PATH', Path(temporary.name) / 'zero.json'),
              patch.object(controller, 'CurrentTelemetry', return_value=Mock())):
            self.c = controller.Controller(self.bridge)
        self.c.mode = 'policy'
        self.c.policy_needs_reset = False
        self.c.web_time = 10.
        self.c.twist = [.2, 0., 0.]
        self.receive(1, 11.01)
        self.c._zeros_pushed = True
        self.c._mcu_zero_pos = list(self.c.zero_pos)
        inference = patch.object(controller.Policy, 'infer', return_value=[0.] * 14)
        self.infer = inference.start()
        self.addCleanup(inference.stop)

    def receive(self, seq, now, **values):
        with patch.object(controller.time, 'monotonic', return_value=now):
            self.c.receive(frame(seq, **values))

    def step(self, now=11.01):
        with patch.object(controller.time, 'monotonic', return_value=now):
            self.c.step()
        return COMMAND.unpack(self.bridge.notify.call_args.args[1])

    def command(self, twist, now=11.02):
        with patch.object(controller.time, 'monotonic', return_value=now):
            return self.c.command(twist=twist)

    def test_expiry_keeps_policy_history_and_continuous_zero_speed_inference(self):
        self.c.policy.prev_action[:] = .7
        self.c.policy._act_filt[:] = .4
        phase = self.c.recovery.phase
        with patch.object(self.c.recovery, 'reset') as reset:
            self.assertEqual(self.step()[4], POLICY)
            self.receive(2, 11.03)
            self.assertEqual(self.step(11.03)[4], POLICY)
            reset.assert_not_called()
        self.assertEqual(self.c.mode, 'policy')
        self.assertEqual(self.c.recovery.phase, phase)
        self.assertTrue(self.c.remote_command_timed_out)
        self.assertTrue(self.c._keyboard_recovery_latched)
        self.assertEqual(self.c.twist, [0.] * 3)
        self.assertEqual(self.infer.call_args.args[4], [0.] * 3)
        self.assertTrue((self.c.policy.prev_action == .7).all())
        self.assertTrue((self.c.policy._act_filt == .4).all())
        self.assertFalse(self.c.policy_needs_reset)

    def test_late_movement_is_rejected_even_before_the_next_control_tick(self):
        result = self.command([.2, 0., 0.])
        self.assertTrue(result['motion_ignored'])
        self.assertEqual(self.c.twist, [0.] * 3)
        self.assertTrue(self.c.remote_command_timed_out)

    def test_neutral_acknowledgement_then_new_command_can_move(self):
        self.step()
        for now in (11.02, 11.04):
            self.assertTrue(self.command([.2, 0., 0.], now)['motion_ignored'])
        self.assertTrue(self.command([0., 0., 0.], 11.05)['motion_ignored'])
        self.assertFalse(self.c.remote_command_timed_out)
        self.assertEqual(self.c.twist, [0.] * 3)
        self.assertNotIn('motion_ignored', self.command([.2, 0., 0.], 11.06))
        self.assertEqual(self.c.twist, [.2, 0., 0.])

    def test_repeated_policy_mode_does_not_bypass_expired_intent(self):
        with patch.object(controller.time, 'monotonic', return_value=11.01):
            self.c.set_mode('policy')
        self.assertTrue(self.command([.2, 0., 0.])['motion_ignored'])
        self.assertEqual(self.c.twist, [0.] * 3)

    def test_live_heartbeat_keeps_current_intent(self):
        self.c.web_time = 10.5
        self.assertEqual(self.step()[4], POLICY)
        self.assertEqual(self.c.twist, [.2, 0., 0.])
        self.assertFalse(self.c.remote_command_timed_out)

    def test_sensor_loss_still_disarms(self):
        self.assertEqual(self.step(11.2)[4], DISARM)
        self.assertEqual(self.c.mode, 'shadow')
        self.receive(2, 11.21)
        self.assertEqual(self.step(11.21)[4], DISARM)

    def test_joint_or_zero_loss_still_disarms(self):
        for failure in ('joint', 'zero'):
            with self.subTest(failure=failure):
                self.c.mode = 'policy'
                self.c._joint_rx_times = [11.01] * 15
                self.c._mcu_zero_pos = list(self.c.zero_pos)
                if failure == 'joint':
                    self.c._joint_rx_times[-1] = 10.
                else:
                    self.c._mcu_zero_pos[-1] += 1
                self.assertEqual(self.step()[4], DISARM)

    def test_hold_timeout_and_explicit_off_still_disarm(self):
        self.c.mode = 'hold'
        self.assertEqual(self.step()[4], DISARM)
        with patch.object(controller.time, 'monotonic', return_value=11.02):
            self.c.set_mode('off')
        self.assertEqual(self.step(11.02)[4], DISARM)
        self.command([0.] * 3)
        self.assertEqual(self.c.mode, 'off')


if __name__ == '__main__':
    unittest.main(verbosity=2)
