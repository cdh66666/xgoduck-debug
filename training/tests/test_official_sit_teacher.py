import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from official_sit_teacher import SitCycleTeacher,HOME


class TeacherEntryTests(unittest.TestCase):
    def observation(self,phase=0,direction=1):
        obs=np.zeros(61,np.float32);obs[5]=-1;obs[55]=phase;obs[56]=direction;return obs

    def test_start_captures_actual_pose_instead_of_standing_destination(self):
        teacher=SitCycleTeacher(HOME);pose=HOME.astype(float).copy();pose[[2,5,11]]+=[4,7,-3]
        action=teacher(self.observation(),pose)
        np.testing.assert_allclose(np.rad2deg(action[[2,5,11]]),[4,7,-3],atol=1e-5)
        pose[5]=70 # Input mutation must not change the captured start.
        np.testing.assert_allclose(teacher.sit_start[5],HOME[5]+7)
        later=teacher(self.observation(.2),pose)
        np.testing.assert_array_equal(later,teacher(self.observation(.2),HOME))
        new=HOME.astype(float).copy();new[5]-=9
        self.assertAlmostEqual(float(np.rad2deg(teacher(self.observation(),new)[5])),-9,places=5)

    def test_rise_returns_to_standing_reference_not_current_seated_angles(self):
        standing=HOME.astype(float).copy();standing[5]+=6;teacher=SitCycleTeacher(standing)
        action=teacher(self.observation(1,-1),np.ones(14)*50)
        np.testing.assert_allclose(np.rad2deg(action),standing-HOME,atol=1e-5)

    def test_mid_phase_cannot_silently_invent_start(self):
        with self.assertRaisesRegex(ValueError,'phase zero'):
            SitCycleTeacher(HOME)(self.observation(.2),HOME)
