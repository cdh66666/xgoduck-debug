"""Recorded policy input must reproduce the actual ONNX output offline."""
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'robot-app/python'))
import controller
from motion_recording import compact
from wire import ARM, COMMAND, POLICY
from test_remote_disconnect import frame


class PolicyRecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.bridge = Mock()
        with (patch.object(controller, 'ZERO_PATH', Path(cls.temporary.name) / 'zero.json'),
              patch.object(controller, 'CurrentTelemetry', return_value=Mock())):
            cls.c = controller.Controller(cls.bridge)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_armed_step_snapshot_replays_exact_input_and_survives_warming(self):
        c = self.c
        c.mode = 'off'
        for seq in range(1, 206):
            with patch.object(controller.time, 'monotonic', return_value=10. + seq * .01):
                c.receive(frame(seq, enabled=False))
        c._zeros_pushed = True
        c._mcu_zero_pos = list(c.zero_pos)
        with patch.object(controller.time, 'monotonic', return_value=12.05):
            c.set_mode('policy')
            c.step()
        self.assertEqual(COMMAND.unpack(self.bridge.notify.call_args.args[1])[4], ARM)
        self.assertIsNone(c.last_policy_step, 'ARM must not invent a policy inference')
        with patch.object(controller.time, 'monotonic', return_value=12.06):
            c.receive(frame(206))
            c.step()
            state = c.status()
        command = COMMAND.unpack(self.bridge.notify.call_args.args[1])
        row = c.last_policy_step
        self.assertEqual(command[4], POLICY)
        self.assertEqual(row['command_seq'], command[1])
        self.assertEqual(row['command_mode'], POLICY)
        self.assertEqual(row['source_mcu_seq'], 206)
        self.assertEqual(row['source_sample_us'], 2060000)
        self.assertEqual(row['phase'], 'walk')
        self.assertEqual(row['model'], 'xgoduck_walk.onnx')
        self.assertEqual(row['model_sha256'], hashlib.sha256(
            (ROOT / 'robot-app/python/xgoduck_walk.onnx').read_bytes()).hexdigest())
        self.assertEqual(len(row['observation']), 61)
        self.assertEqual(len(row['raw_action']), 14)
        self.assertEqual(len(row['filtered_action']), 14)
        np.testing.assert_array_equal(row['submitted_targets'], command[7:])
        output = c.policy.session.run(c.policy._outputs, {
            c.policy.in_name: np.asarray([row['observation']], dtype=np.float32)})[0][0]
        np.testing.assert_array_equal(output, row['raw_action'])
        saved = compact(state)
        observation = list(row['observation'])
        c.policy.warm(1)
        self.assertEqual(c.last_policy_step['observation'], observation)
        self.assertEqual(saved['last_policy_step']['observation'], observation)
        self.assertIsNot(saved['last_policy_step'], row)
        self.assertEqual(saved['remote_command']['policy_disconnect_action'], 'zero_intent_keep_local_policy')
        # A stop that wins during ONNX must not publish that obsolete trace/action.
        with patch.object(controller.time, 'monotonic', return_value=12.07):
            c.set_mode('off')
        self.assertIsNone(c.last_policy_step)
        c.mode = 'policy'
        c.arm_pending = False
        c.policy_needs_reset = False
        original = c.policy.infer

        def stopping_inference(*args):
            result = original(*args)
            c.set_mode('off')
            return result

        before = self.bridge.notify.call_count
        with (patch.object(controller.time, 'monotonic', return_value=12.08),
              patch.object(c.policy, 'infer', side_effect=stopping_inference)):
            # set_mode takes the policy lock after its generation update. Use
            # a reentrant test lock to exercise that update in the same thread.
            import threading
            c.policy_lock = threading.RLock()
            c.step()
        self.assertEqual(self.bridge.notify.call_count, before)
        self.assertIsNone(c.last_policy_step)


if __name__ == '__main__':
    unittest.main(verbosity=2)
