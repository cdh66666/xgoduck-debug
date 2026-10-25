import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from butt_sit import SitPlant
from evaluate_skill_chain import run

class ChainAbortTests(unittest.TestCase):
    def test_physical_failure_stops_all_later_stages_even_with_other_failures(self):
        plant=SitPlant()
        try:
            bad=lambda observation:np.full(14,4.,dtype=np.float32)
            result,trace=run(plant,bad,bad,3.7,4000,True)
            self.assertIsNotNone(result['aborted_stage']);self.assertFalse(result['complete'])
            self.assertEqual(result['stages'][-1]['stage'],result['aborted_stage'])
            self.assertEqual(trace[-1]['stage'],result['aborted_stage'])
            failures=[v.split(':')[0] for v in result['failures'] if v.endswith(':tilt_or_head_clearance')]
            self.assertEqual(failures,[result['aborted_stage']])
            self.assertEqual(result['resets_during_recording'],0)
        finally:plant.adapter.close()

if __name__=='__main__':unittest.main()
