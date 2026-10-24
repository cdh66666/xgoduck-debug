import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robot-app/python'))
from gyro_bias_control import FrozenGyroBias


class StationaryQualificationTests(unittest.TestCase):
    def observe(self, fit, seq, **extra):
        data=dict(acc=[0,0,9.8],gyro=[0,-8,0],angles=[0.]*15,velocities=[0.]*15,
                  seq=seq,sample_us=seq*10000,mask=0x7fff,imu_ok=True,eligible=True,epoch='boot',now=seq*.01)
        data.update(extra);return fit.observe(**data)

    def test_actual_quiet_complete_feedback_qualifies_without_changing_limits(self):
        fit=FrozenGyroBias()
        for seq in range(1,260):self.observe(fit,seq)
        self.assertTrue(fit.ready)
        self.assertEqual(fit.MAX_GYRO_STD_DPS,.35)
        self.assertIsNone(fit.frozen)

    def test_missing_feedback_and_reported_joint_motion_never_qualify(self):
        for extra in [dict(mask=0),dict(velocities=[10.]*15),dict(angles=[0.]*14)]:
            fit=FrozenGyroBias()
            for seq in range(1,260):self.observe(fit,seq,**extra)
            self.assertFalse(fit.ready,extra)
            with self.assertRaises(ValueError):fit.freeze(epoch='boot',now=2.59)

    def test_slow_drift_with_zero_reported_speed_and_mouth_motion_are_detected(self):
        for joint in [0,14]:
            fit=FrozenGyroBias()
            for seq in range(1,310):
                angles=[0.]*15;angles[joint]=seq*.01
                self.observe(fit,seq,angles=angles)
            self.assertFalse(fit.ready)

    def test_frozen_correction_remains_fixed_during_policy_motion(self):
        fit=FrozenGyroBias()
        for seq in range(1,260):self.observe(fit,seq)
        bias=fit.freeze(epoch='boot',now=2.59)
        self.observe(fit,260,velocities=[30.]*15,gyro=[1,2,3],eligible=False)
        self.assertEqual(fit.frozen,bias)

    def test_range_includes_both_sides_and_expires_old_extrema(self):
        fit=FrozenGyroBias()
        self.observe(fit,1)
        self.observe(fit,2,angles=[.3]*15)
        self.observe(fit,3,angles=[-.3]*15)
        self.assertEqual(len(fit.samples),1)
        fit=FrozenGyroBias()
        self.observe(fit,1,angles=[-.2]*15)
        for seq in range(2,258):self.observe(fit,seq)
        self.observe(fit,258,angles=[.2]*15)
        self.assertTrue(fit.ready)
        self.assertEqual(len(fit.samples),256)

    def test_rolling_gyro_statistics_match_full_window_across_expiry_and_reset(self):
        import statistics
        fit=FrozenGyroBias()
        for seq in range(1,1600):
            if seq == 800:fit.reset()
            gyro=[(600+(seq*17)%23)/32,(-250+(seq*7)%19)/32,((seq*3)%11)/32]
            self.observe(fit,seq,gyro=gyro)
            if fit.mean is None:continue
            for axis in range(3):
                values=[row[1][axis] for row in fit.samples]
                self.assertAlmostEqual(fit.mean[axis],statistics.mean(values),places=12)
                self.assertAlmostEqual(fit.std[axis],statistics.stdev(values),places=10)


if __name__=='__main__':unittest.main(verbosity=2)
