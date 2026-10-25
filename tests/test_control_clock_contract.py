import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc'))
from control_clock_check import check_50hz_loop

class ClockContractTests(unittest.TestCase):
    def test_real_controller_preserves_phase_after_small_overrun(self):
        result=check_50hz_loop((ROOT/'robot-app/python/controller.py').read_text())
        self.assertEqual(result['steps_in_one_second'],50)

    def test_previous_native_skip_and_idle_warm_algorithm_is_rejected(self):
        previous='''class Controller:
 def run(self):
  deadline=time.monotonic()
  while not self.stop_event.is_set():
   self.step();deadline+=CONTROL_PERIOD;now=time.monotonic()
   if deadline<=now:
    missed=int((now-deadline)/CONTROL_PERIOD)+1
    self.loop_missed+=missed;deadline+=missed*CONTROL_PERIOD
   elif deadline-now>.008:self._keep_warm_idle()
   self.stop_event.wait(max(0.,deadline-time.monotonic()))
'''
        with self.assertRaises(ValueError):check_50hz_loop(previous)
