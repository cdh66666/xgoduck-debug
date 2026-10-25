import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pc'))
from factory_workbench import pose_basis_matches


class PoseBasisTests(unittest.TestCase):
    def setUp(self):
        self.baseline={'local_sources_verified':True}
        self.state={'encoder_center':2047,'recovery':{'thresholds':{'default_pose_s':1.0}},
                    'deployment':{'reference':'rl/raspi_deploy','control_hz':50,
                    'home_deg':[0,-5,-22,0,18,20,20,0,0,0,5,22,0,-18],
                    'default_deg':[0,-5,-22,0,18,0,5,22,0,-18,20,20,0,0,0]}}

    def test_current_official_pose_is_not_rejected_for_recovery_delay(self):
        self.assertTrue(pose_basis_matches(self.baseline,self.state))

    def test_changed_pose_clock_or_encoder_is_not_admitted(self):
        for key,value in [('home_deg',[0]*14),('default_deg',[0]*15),('control_hz',25)]:
            state=copy.deepcopy(self.state);state['deployment'][key]=value
            self.assertFalse(pose_basis_matches(self.baseline,state))
        self.state['encoder_center']=2048
        self.assertFalse(pose_basis_matches(self.baseline,self.state))
        self.assertFalse(pose_basis_matches({'local_sources_verified':False},{}))
        self.assertFalse(pose_basis_matches(self.baseline,{}))


if __name__=='__main__':unittest.main()
