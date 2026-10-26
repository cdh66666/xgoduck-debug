import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from butt_sit import SitPlant
from evaluate_official_sit import cycle


class EntryEvaluationTests(unittest.TestCase):
    def test_entry_keeps_delay_history_and_clock_into_first_inference(self):
        plant=SitPlant();seen=[]
        class Variation:
            sample={'transport_delay_physics_steps':6}
            def before_reset(self):pass
        def entry(current):
            self.assertEqual(len(current.delay),6)
            current.advance(current.filtered+.02)
            seen.append((current.sim.data.time,current.previous.copy(),[q.copy() for q in current.delay]))
        def predict(obs):
            if len(seen)==1:
                self.assertAlmostEqual(plant.sim.data.time,.02)
                np.testing.assert_array_equal(plant.previous,seen[0][1])
                np.testing.assert_array_equal(plant.delay,seen[0][2])
                seen.append(True)
            return plant.previous
        measure=plant.measure
        def end_after_one():
            row=measure();row['head_clearance_mm']=0.;return row
        plant.measure=end_after_one
        try:
            result,rows=cycle(plant,predict,42,Variation(),entry=entry)
            self.assertEqual(len(rows),1)
            self.assertAlmostEqual(rows[0]['t'],.04)
            self.assertAlmostEqual(result['entry_duration_s'],.02)
            self.assertEqual(result['resets_between_skills'],0)
            self.assertEqual(len(seen),2)
        finally:plant.adapter.close()

    def test_reset_inside_entry_is_rejected(self):
        plant=SitPlant()
        try:
            with self.assertRaisesRegex(ValueError,'preserve physical state'):
                cycle(plant,lambda obs:plant.previous,42,entry=lambda current:current.reset(43))
        finally:plant.adapter.close()
