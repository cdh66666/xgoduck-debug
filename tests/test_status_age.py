from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pc'))
from status_age import aged_snapshot


class SnapshotAgeTests(unittest.TestCase):
    def test_cached_250ms_plus_native_8ms_fails_150ms_gate(self):
        raw=dict(feedback_age_ms=8.,full_robot_ready=True,pc_observed_monotonic_s=10.)
        state=aged_snapshot(raw,10.,10.25)
        self.assertEqual(state['feedback_age_ms'],258.)
        self.assertEqual(state['native_feedback_age_ms'],8.)
        self.assertFalse(state['full_robot_ready'])
        self.assertEqual(raw['feedback_age_ms'],8.)
        self.assertTrue(raw['full_robot_ready'])
        self.assertAlmostEqual(aged_snapshot(raw,10.,10.01)['feedback_age_ms'],18.)

    def test_joint_expiry_and_global_expiry_keep_cached_readiness_consistent(self):
        raw=dict(feedback_age_ms=3.,full_robot_ready=True,control_ready=True,
                 joint_feedback_age_ms=[149.]+[3.]*14,stale_joint_ids=[])
        partial=aged_snapshot(raw,10.,10.02)
        self.assertFalse(partial['control_ready'])
        self.assertEqual(partial['stale_joint_ids'],[10])
        old=aged_snapshot(raw,10.,10.2)
        self.assertFalse(old['control_ready'])
        self.assertEqual(len(old['stale_joint_ids']),15)
        self.assertEqual(raw['stale_joint_ids'],[])

    def test_known_delivery_backlog_does_not_double_count_native_cache(self):
        raw=dict(feedback_age_ms=70.,telemetry_cache_age_ms=60.,
                 pc_source_progress={'relative_backlog_lower_bound_s':.2},
                 full_robot_ready=True,control_ready=True,joint_feedback_age_ms=[70.]*15)
        state=aged_snapshot(raw,10.,10.02)
        self.assertAlmostEqual(state['pc_delivery_age_lower_bound_ms'],140.)
        self.assertAlmostEqual(state['feedback_age_ms'],230.)
        self.assertAlmostEqual(state['joint_feedback_age_ms'][0],230.)
        self.assertFalse(state['full_robot_ready'])
        self.assertFalse(state['control_ready'])
        self.assertTrue(state['native_full_robot_ready'])
        self.assertEqual(state['native_feedback_age_ms'],70.)
        self.assertEqual(raw['feedback_age_ms'],70.)
        raw['pc_source_progress']['relative_backlog_lower_bound_s']=.04
        self.assertEqual(aged_snapshot(raw,10.,10.)['feedback_age_ms'],70.)

    def test_recorded_burst_endpoint_is_stale_even_on_instant_pc_delivery(self):
        # wlan-media-comparison-20261023, observation 13012. This old stream
        # did not report cache age: use max(native age, known source backlog).
        raw=dict(feedback_age_ms=19.53480399970431,
                 pc_source_progress={'relative_backlog_lower_bound_s':.3248473830171861},
                 full_robot_ready=True,control_ready=True)
        state=aged_snapshot(raw,10.,10.)
        self.assertAlmostEqual(state['feedback_age_ms'],324.8473830171861)
        self.assertFalse(state['full_robot_ready'])
        self.assertFalse(state['control_ready'])

    def test_joint_only_expiry_includes_delivery_without_rejecting_sensor_history(self):
        raw=dict(feedback_age_ms=1.,telemetry_cache_age_ms=0.,
                 pc_source_progress={'relative_backlog_lower_bound_s':.1},
                 full_robot_ready=True,control_ready=True,
                 joint_feedback_age_ms=[80.]+[1.]*14)
        state=aged_snapshot(raw,10.,10.)
        self.assertEqual(state['feedback_age_ms'],101.)
        self.assertEqual(state['stale_joint_ids'],[10])
        self.assertFalse(state['control_ready'])
        self.assertTrue(state['native_full_robot_ready'])


if __name__=='__main__':unittest.main()
