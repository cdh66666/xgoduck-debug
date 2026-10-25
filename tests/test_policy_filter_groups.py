import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robot-app/python'))
from policy import Policy


class PolicyFilterGroupsTests(unittest.TestCase):
    def test_head_filter_cannot_change_right_hip_response(self):
        for head_alpha in (.2,.8):
            p=Policy();p.set_action_alphas(leg=.45,head=head_alpha)
            p.session=SimpleNamespace(run=lambda *args:[np.ones((1,14),dtype=np.float32)])
            p._outputs=['actions'];p._feed={}
            filtered=p.infer([0,0,-1],[0]*3,[0]*15,[0]*15)
            np.testing.assert_allclose(filtered[[0,1,2,3,4,9,10,11,12,13]],.55)
            np.testing.assert_allclose(filtered[5:9],1-head_alpha)
            self.assertEqual(float(filtered[0]),float(filtered[9]))


if __name__=='__main__':unittest.main()
