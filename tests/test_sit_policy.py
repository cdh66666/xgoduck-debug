import sys,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'robot-app/python'))
from sit_policy import SitPolicy,SIT_HOME,SIT_SECONDS
from sit_runtime import SitRuntimeMixin
from rl_core import POLICY_INDEX

class PolicyContractTests(unittest.TestCase):
    @unittest.skipUnless((ROOT/'robot-app/python/xgoduck_sit.onnx').exists(), 'Optional sit candidate is not included in public checkout')
    def test_onnx_inputs_and_raw_action_match_deployed_adapter(self):
        policy=SitPolicy();ok,message=policy.load();self.assertTrue(ok,message)
        targets=np.zeros(15,dtype=np.float32);targets[POLICY_INDEX]=SIT_HOME
        policy.seed_targets(targets);policy.phase=.6;policy.direction=1.
        policy.infer([.05,0,-.99875],[0,0,0],targets,[0]*15)
        obs=np.asarray(policy.last_inference['observation'],dtype=np.float32)
        self.assertAlmostEqual(float(obs[55]),.6,places=6);self.assertEqual(obs[56],1)
        np.testing.assert_allclose(obs[6:20],0,atol=1e-6)
        np.testing.assert_allclose(obs[34:48],0,atol=1e-6)
        raw=policy.session.run(None,{'observation':obs.reshape(1,61)})[0][0]
        np.testing.assert_allclose(raw,policy.last_inference['raw_action'],atol=1e-6)
        changed=obs.copy();changed[20:34]=100
        np.testing.assert_array_equal(raw,policy.session.run(None,{'observation':changed.reshape(1,61)})[0][0])

    def test_seed_keeps_real_last_targets_in_policy_order(self):
        policy=SitPolicy();targets=np.arange(15,dtype=np.float32)
        policy.seed_targets(targets)
        np.testing.assert_allclose(policy.action_to_deg(policy._act_filt)[0:14],targets[:14],atol=1e-5)

class Owner(SitRuntimeMixin):
    def __init__(self):
        self.lock=threading.RLock();self._sit_init();self.mode='policy';self.generation=0
        self.feedback=SimpleNamespace(enabled=True,imu_ok=True,mask=32767,acc=[0,0,-9.81],angles=[0.]*15)
        self.filtered={'grav':[0,0,-1]};self.rx_time=time.monotonic();self.twist=[0]*3;self.head=[0]*4;self.mouth=0
        self.pick_active=False;self.roulade_until=0;self.recovery=SimpleNamespace(phase='walk',reset=lambda:None)
        self.last_action=[0.]*15;self.sit_policy=SitPolicy();self.policy_needs_reset=False
    def _require_feedback(self,mask):return self.feedback

class SitOwnershipTests(unittest.TestCase):
    def test_missing_optional_model_does_not_prevent_unloaded_startup(self):
        owner=Owner();owner.mode='off';owner.feedback.enabled=False;owner.sit_policy=None
        result=owner.prepare_sit(ROOT/'does-not-exist-sit.onnx')
        self.assertFalse(result['ok']);self.assertFalse(result['available'])
        status=owner._sit_status_locked(time.monotonic())
        self.assertFalse(status['available']);self.assertTrue(status['error'])
        self.assertEqual(owner.mode,'off');self.assertFalse(owner.feedback.enabled)

    def test_start_and_completed_seat_keep_policy_enabled(self):
        owner=Owner();r=owner.start_sit();self.assertEqual(r['phase'],'lowering')
        self.assertEqual(owner.mode,'policy');self.assertTrue(owner.feedback.enabled)
        self.assertFalse(owner._sit_status_locked(owner.sit_started+SIT_SECONDS)['pose_ready'])
        owner.sit_started=time.monotonic()-SIT_SECONDS;owner.feedback.angles[2:4]=[54.,83.]
        owner.feedback.angles[7:9]=[-54.,-83.]
        self.assertEqual(owner._sit_status_locked(time.monotonic())['phase'],'seated')
        with self.assertRaises(ValueError):owner._sit_command_locked([.2,0,0],None,None)
        with self.assertRaises(ValueError):owner._sit_mode_locked('policy')
        owner._sit_mode_locked('off');self.assertFalse(owner.sit_active)

    def test_travel_and_tray_cannot_start_sitting(self):
        owner=Owner();owner.twist=[.2,0,0]
        with self.assertRaises(ValueError):owner.start_sit()
        owner.twist=[0]*3;owner.feedback.acc=[-9.81,0,0]
        with self.assertRaises(ValueError):owner.start_sit()

    def test_excessive_tracking_pauses_phase_without_disarming(self):
        owner=Owner();owner.start_sit();owner.feedback.angles[3]=30.
        owner._sit_select_locked(owner.sit_started+2,[0,0,-1],owner.feedback)
        phase=owner.sit_paused_phase
        owner._sit_select_locked(owner.sit_started+4,[0,0,-1],owner.feedback)
        self.assertEqual(owner.sit_policy.phase,phase);self.assertTrue(owner.sit_error)
        self.assertEqual(owner.mode,'policy');self.assertTrue(owner.feedback.enabled)

    def test_shallow_trial_cap_and_resume_do_not_jump_phase(self):
        owner=Owner();owner.start_sit(.35);owner.sit_started=time.monotonic()-4
        self.assertEqual(owner._sit_phase_locked(time.monotonic()),.35)
        owner.start_sit(1.)
        self.assertAlmostEqual(owner._sit_phase_locked(time.monotonic()),.35,places=3)

if __name__=='__main__':unittest.main()
