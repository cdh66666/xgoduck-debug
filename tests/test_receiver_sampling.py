import ast
import copy
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent/'pc'
ACTIVE=True
TEST_SOURCE=ROOT/'simulation/worker.py'
sys.path.insert(0,str(ROOT/'simulation'))
import worker
import numpy as np
from simulation_catalog import resolve_model
binding=resolve_model(ROOT,'official-getup')
sys.path.insert(0,str(Path(binding['sources']['policy.py']['path']).parent))

module=types.ModuleType('qa_receiver_candidate');module.__file__=str(ROOT/'simulation/worker.py')
exec(compile(TEST_SOURCE.read_text('utf-8'),'<tested receiver source>','exec'),module.__dict__)


class ReceiverTests(unittest.TestCase):
    def test_in_between_pulse_matches_actual_signal_filter_at_100hz(self):
        from policy import Policy
        from proc import SignalFilter
        from rl_core import POLICY_INDEX
        host=SignalFilter();sim=module.Simulation()
        sim.data=types.SimpleNamespace(time=0.,qvel=np.zeros(14));sim.model=object();sim.root_id=0;sim.vadr=np.arange(14)
        sim._feedback_filter_time=None;sim.filtered_gyro=sim.filtered_velocity=None
        sample=[0.]
        def object_velocity(model,data,kind,root,out,local):out[:3]=[sample[0],0,0]
        with patch.object(module.mujoco,'mj_objectVelocity',object_velocity):
            for t,rate in [(0.,0.),(.01,1.),(.02,0.)]:
                sample[0]=rate;sim.data.time=t;sim.data.qvel[:]=rate
                sim._receive_policy_feedback()
                velocities=np.zeros(15);velocities[POLICY_INDEX]=np.rad2deg(np.full(14,rate))
                result=host.update([0,0,-9.8],np.rad2deg([rate,0,0]).tolist(),[0.]*15,velocities.tolist(),timestamp=1+t)
        np.testing.assert_allclose(sim.filtered_gyro,result['gyro_rad'],atol=1e-12)
        np.testing.assert_allclose(sim.filtered_velocity,np.deg2rad(np.array(result['vels_deg'])[POLICY_INDEX]),atol=1e-12)
        self.assertAlmostEqual(sim.filtered_gyro[0],.25)
        self.assertAlmostEqual(sim.filtered_velocity[0],.24)
        self.assertNotEqual(sim.filtered_gyro[0],0.) # old end-sample-only approximation

    def test_same_timestamp_does_not_double_filter(self):
        sim=module.Simulation();sim.data=types.SimpleNamespace(time=0.);sim._feedback_filter_time=0.
        with patch.object(module.mujoco,'mj_objectVelocity',side_effect=AssertionError('unexpected duplicate sample')):
            sim._receive_policy_feedback()

    def test_real_models_execute_and_getup_handoff_kept(self):
        sim=module.Simulation();bound=resolve_model(ROOT,'official-getup');sim.load('official-getup',bound['sha256']);sim.set_policy(True);sim.step(400)
        self.assertEqual(sim.active_model,'walk');self.assertGreater(sim.inference_counts['getup'],0);self.assertGreater(sim.inference_counts['walk'],0)

    def test_only_filter_sampling_home_binding_and_pause_neutral_changed(self):
        before=ast.parse((HERE/'fixtures/worker-pre-receiver.py').read_text('utf-8'));after=ast.parse(TEST_SOURCE.read_text('utf-8'))
        for tree in (before,after):
            cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Simulation')
            cls.body=[n for n in cls.body if not isinstance(n,ast.FunctionDef) or n.name not in ('reset','step','infer','_receive_policy_feedback','mouth_command_deg')]
            for method in cls.body:
                if not isinstance(method,ast.FunctionDef):continue
                if method.name=='__init__':
                    method.body=[n for n in method.body if not (isinstance(n,ast.Assign) and len(n.targets)==1 and isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr=='feedback_filter_samples')]
                # Official roles now explicitly share runtime HOME. Assert the
                # exact constructor change before normalizing the old fixture.
                if method.name=='load':
                    for call in (n for n in ast.walk(method) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='Policy'):
                        for keyword in call.keywords:
                            self.assertEqual(keyword.arg,'reference_home')
                            self.assertIsInstance(keyword.value,ast.Constant)
                            self.assertIs(keyword.value.value,True)
                        call.keywords=[]
                if method.name=='state':
                    output=next(n for n in method.body if isinstance(n,ast.Return) and isinstance(n.value,ast.Call))
                    output.value.keywords=[k for k in output.value.keywords if k.arg not in ('feedbackFilterSteps','receiverDt','mouthCommandDeg')]
                if tree is after and method.name=='command':
                    paused=next(n for n in ast.walk(method) if isinstance(n,ast.If) and ast.unparse(n.test)=='self.paused')
                    self.assertEqual(ast.unparse(paused.body[0]),'self.twist.fill(0)')
                    paused.body.pop(0)  # Explicit pause clears intent; physics remains frozen.
        self.assertEqual(ast.dump(before,include_attributes=False),ast.dump(after,include_attributes=False))


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ReceiverTests))
    import tempfile
    output=Path(tempfile.gettempdir())/'duck-portable-receiver-tests.json'
    output.write_text(json.dumps(dict(result='PASS' if result.wasSuccessful() else 'FAIL',tests=result.testsRun,tested_active_source=ACTIVE),indent=2)+'\n','utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
