"""The native official catalog must match its vendor runtime HOME contract."""
from pathlib import Path
import sys
import unittest
import numpy as np

RUNTIME=Path(__file__).resolve().parents[1]/'robot-app/python'
sys.path.insert(0,str(RUNTIME))
from policy import Policy
from rl_core import DEFAULT_DEG, HOME_DEG, POLICY_INDEX


class OfficialPolicyContractTests(unittest.TestCase):
    def test_all_official_roles_use_the_same_runtime_home(self):
        for role in ('walk','getup','pick','roulade'):
            with self.subTest(role=role):
                policy=Policy(reference_home=True)
                ok,message=policy.load(RUNTIME/('xgoduck_'+role+'.onnx'))
                self.assertTrue(ok,message)
                self.assertEqual(policy.home_source,'official_runtime')
                np.testing.assert_array_equal(policy.home_deg,HOME_DEG)
                targets=np.asarray(policy.action_to_deg(np.zeros(14,dtype=np.float32)))
                np.testing.assert_array_equal(targets[POLICY_INDEX],HOME_DEG)
                observation=policy.build_obs([0,0,-1],[0,0,0],DEFAULT_DEG,[0.]*15)
                np.testing.assert_array_equal(observation[6:20],np.zeros(14))

    def test_explicit_metadata_mode_still_uses_the_exported_training_home(self):
        policy=Policy(reference_home=False)
        ok,message=policy.load(RUNTIME/'xgoduck_walk.onnx')
        self.assertTrue(ok,message)
        self.assertEqual(policy.home_source,'onnx.default_joint_pos')
        self.assertFalse(np.array_equal(policy.home_deg,HOME_DEG))
        # The official walk export describes about 24 degrees, while its
        # deployed adapter deliberately uses the shared 22-degree runtime HOME.
        self.assertAlmostEqual(abs(float(policy.home_deg[2])),24.,delta=.02)


if __name__=='__main__':unittest.main()
