"""The separate ID34 Pick command, with the unchanged 14-axis physics."""
import ast
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc/simulation'))
from worker import Simulation
from simulation_catalog import resolve_model


class MouthTests(unittest.TestCase):
    def make(self, name='pick'):
        bound=resolve_model(ROOT/'pc','official-'+name)
        sim=Simulation();sim.load('official-'+name,bound['sha256'])
        return sim

    def test_pick_opens_then_closes_with_actual_controller_timing(self):
        tree=ast.parse((ROOT/'robot-app/python/controller.py').read_text('utf-8'))
        values={node.targets[0].id:ast.literal_eval(node.value) for node in tree.body
                if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name)
                and node.targets[0].id in ('PICK_PERIOD_S','PICK_MOUTH_CLOSE_PHI')}
        sim=self.make();self.assertEqual(sim.state()['mouthCommandDeg'],0)
        sim.set_policy(True);self.assertEqual(sim.state()['mouthCommandDeg'],30)
        self.assertEqual((sim.model.nq,sim.model.nu),(21,14))
        close_time=values['PICK_PERIOD_S']*values['PICK_MOUTH_CLOSE_PHI']
        sim.step(round((close_time-.02)/.005));self.assertEqual(sim.state()['mouthCommandDeg'],30)
        sim.step(8);self.assertEqual(sim.state()['mouthCommandDeg'],0)
        sim.step(500);self.assertEqual(sim.active_model,'walk');self.assertEqual(sim.state()['mouthCommandDeg'],0)

    def test_pause_holds_mouth_reset_and_aborted_pick_close_it(self):
        sim=self.make();sim.set_policy(True);sim.step(20)
        sim.command({'op':'pause','paused':True});self.assertEqual(sim.state()['mouthCommandDeg'],30)
        sim.active_model='getup';self.assertEqual(sim.state()['mouthCommandDeg'],0)
        sim.reset();self.assertEqual(sim.state()['mouthCommandDeg'],0)
        walk=self.make('walk');walk.set_policy(True);self.assertEqual(walk.state()['mouthCommandDeg'],0)


if __name__=='__main__':unittest.main()
