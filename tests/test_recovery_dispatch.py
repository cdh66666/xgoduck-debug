"""Current official model bindings and Recovery dispatch; local CPU only."""
import ast
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pc/simulation'))
from worker import Simulation, HERE
from simulation_catalog import resolve_model
import active_runtime

ROOT=HERE.parent


class RecoveryDispatchTests(unittest.TestCase):
    def make(self,name):
        bound=resolve_model(ROOT,'official-'+name)
        sim=Simulation();sim.load('official-'+name,bound['sha256']);sim.set_policy(True)
        return sim

    def test_all_four_roles_bind_actual_deployed_official_getup_and_walk(self):
        m=active_runtime.verified_sources(ROOT)
        for name in ('walk','getup','pick','roulade'):
            b=resolve_model(ROOT,'official-'+name)
            self.assertEqual(b['models']['getup']['sha256'],m['new_source_manifest']['python/xgoduck_getup.onnx'])
            self.assertEqual(b['models']['walk']['sha256'],m['new_source_manifest']['python/xgoduck_walk.onnx'])
            self.assertEqual(b['recovery_sha256'],m['new_source_manifest']['python/recovery.py'])

    def test_getup_role_hands_back_to_walk_after_one_second_upright(self):
        sim=self.make('getup')
        sim.step(400)
        self.assertEqual(sim.recovery.phase,'walk')
        self.assertEqual(sim.active_model,'walk')
        self.assertGreater(sim.inference_counts['getup'],0)
        self.assertGreater(sim.inference_counts['walk'],0)

    def test_roll_is_not_aborted_by_recovery_before_its_official_window(self):
        sim=self.make('roulade')
        sim.step(360)
        self.assertEqual(sim.active_model,'roulade')
        self.assertEqual(sim.recovery.phase,'walk')
        self.assertEqual(sim.inference_counts['getup'],0)
        # The roll trajectory need not finish fallen. Exercise post-roll
        # recovery with an explicit fallen sensor input, not that assumption.
        update=sim.recovery.update
        with patch.object(sim.recovery,'update',side_effect=lambda gravity,now,valid:update([1.,0.,0.],now,valid)):
            sim.step(100)
        self.assertGreater(sim.inference_counts['getup'],0)
        self.assertFalse(sim.special_active)

    def test_external_force_is_cleared_after_duration_not_held_forever(self):
        sim=self.make('walk');sim.step(200)
        sim.command(dict(op='push',force=[2,0,0],duration=.1))
        sim.step(10)
        self.assertGreater(float(abs(sim.data.qfrc_applied).sum()),0)
        sim.step(40)
        self.assertEqual(float(abs(sim.data.qfrc_applied).sum()),0)
        self.assertEqual(sim.state()['force']['remainingS'],0)

    def test_pausing_keyboard_simulation_clears_the_command_before_resume(self):
        sim=self.make('walk')
        sim.command(dict(op='command',twist=[.3,0,0]))
        sim.command(dict(op='pause',paused=True))
        self.assertEqual(sim.state()['command'],[0,0,0])
        sim.command(dict(op='pause',paused=False))
        self.assertEqual(sim.state()['command'],[0,0,0])

    def test_physics_methods_unchanged_by_dispatch_fix(self):
        old=ast.parse((Path(__file__).resolve().parent/'fixtures/worker-pre-dispatch.py').read_text('utf-8'))
        new=ast.parse((HERE/'worker.py').read_text('utf-8'))
        frame_updates=[n for n in ast.walk(new) if isinstance(n,ast.Attribute) and n.attr=='mjOBJ_XBODY']
        self.assertEqual(len(frame_updates),3)
        # The separately approved bodyframe-v2 correction changes exactly
        # these three velocity sampling constants, not integration or contacts.
        for node in frame_updates:node.attr='mjOBJ_BODY'
        # The official catalog now uses the same fixed HOME as the real runtime.
        policies=[n for n in ast.walk(new) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='Policy']
        self.assertEqual(len(policies),1)
        self.assertEqual([(kw.arg,ast.literal_eval(kw.value)) for kw in policies[0].keywords],[('reference_home',True)])
        policies[0].keywords=[]
        for tree in (old,new):
            cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Simulation')
            cls.body=[n for n in cls.body if not isinstance(n,ast.FunctionDef) or n.name not in ('infer','_receive_policy_feedback','mouth_command_deg')]
            # The later approved receiver-rate fix adds only a filter clock,
            # actual sample counter and one 100 Hz hook before the original
            # physics body. Remove those precise additions for this comparison;
            # all contact, motor, target-delay and integration code stays checked.
            for method in cls.body:
                if not isinstance(method,ast.FunctionDef):continue
                if method.name in ('__init__','reset'):
                    method.body=[n for n in method.body if not (isinstance(n,ast.Assign) and len(n.targets)==1 and
                        isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr in ('_feedback_filter_time','feedback_filter_samples'))]
                if method.name=='step':
                    loop=next(n for n in method.body if isinstance(n,ast.For))
                    if loop.body and isinstance(loop.body[0],ast.If) and ast.unparse(loop.body[0].test)=='self.physics_steps % 2 == 0':
                        self.assertEqual(ast.unparse(loop.body[0].body[0]),'self._receive_policy_feedback()')
                        loop.body.pop(0)
                if method.name=='state':
                    output=next(n for n in method.body if isinstance(n,ast.Return) and isinstance(n.value,ast.Call))
                    output.value.keywords=[k for k in output.value.keywords if k.arg not in ('feedbackFilterSteps','receiverDt','mouthCommandDeg')]
                if tree is new and method.name=='command':
                    paused=next(n for n in ast.walk(method) if isinstance(n,ast.If) and ast.unparse(n.test)=='self.paused')
                    self.assertEqual(ast.unparse(paused.body[0]),'self.twist.fill(0)')
                    paused.body.pop(0)  # UI pause now also clears the virtual keyboard intent.
        self.assertEqual(ast.dump(old,include_attributes=False),ast.dump(new,include_attributes=False))


if __name__=='__main__':unittest.main(verbosity=2)
