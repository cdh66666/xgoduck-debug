"""Execute the real loop with a clock to check late-step scheduling."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[1]


class ControlScheduleTests(unittest.TestCase):
    def starts(self, durations):
        tree=ast.parse((ROOT/'robot-app/python/controller.py').read_text('utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Controller')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='run')
        clock=[0.];starts=[]
        stop=SimpleNamespace(is_set=lambda:len(starts)==len(durations),wait=lambda seconds:clock.__setitem__(0,clock[0]+seconds))
        def step():
            starts.append(clock[0]);clock[0]+=durations[len(starts)-1]
        control=SimpleNamespace(stop_event=stop,step=step,loop_missed=0)
        scope={'time':SimpleNamespace(monotonic=lambda:clock[0]),'CONTROL_PERIOD':.02}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'control-loop','exec'),scope)
        scope['run'](control)
        return starts,control.loop_missed

    def test_short_steps_keep_50hz(self):
        starts,missed=self.starts([.003]*4)
        for a,b in zip(starts,[0,.02,.04,.06]):self.assertAlmostEqual(a,b)
        self.assertEqual(missed,0)

    def test_one_ms_overrun_does_not_insert_an_extra_19ms_wait(self):
        starts,missed=self.starts([.021,.003,.003])
        for a,b in zip(starts,[0,.021,.04]):self.assertAlmostEqual(a,b)
        self.assertEqual(missed,0)

    def test_long_overrun_rebases_without_catching_up_a_queue(self):
        starts,missed=self.starts([.065,.003,.003])
        for a,b in zip(starts,[0,.065,.085]):self.assertAlmostEqual(a,b)
        self.assertEqual(missed,2)

    def test_repeated_small_overruns_do_not_slow_the_policy_clock(self):
        starts,missed=self.starts([.021,.003]*100)
        self.assertAlmostEqual(starts[-1],3.981)
        self.assertEqual(missed,0)


if __name__=='__main__':unittest.main()
