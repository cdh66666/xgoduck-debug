"""Explicit OFF starts must not treat a partly collapsed body as standing."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robot-app/python'))
from recovery import Recovery

def gravity(degrees):
    angle=math.radians(degrees)
    return [math.sin(angle),0.,-math.cos(angle)]

class RecoveryStartupTests(unittest.TestCase):
    def test_measured_26_degree_restart_uses_getup_immediately(self):
        r=Recovery()
        self.assertTrue(r.start(gravity(26),0))
        self.assertEqual(r.phase,'getup')
        self.assertEqual(r.transitions,1)

    def test_already_upright_start_stays_walk(self):
        r=Recovery()
        self.assertFalse(r.start(gravity(6),0))
        self.assertEqual(r.phase,'walk')

    def test_running_walk_keeps_existing_fall_threshold(self):
        r=Recovery();r.start(gravity(6),0)
        for i in range(30):r.update(gravity(26),i*.02)
        self.assertEqual(r.phase,'walk')
        for i in range(8):r.update(gravity(70),1+i*.02)
        self.assertEqual(r.phase,'getup')

    def test_transient_upright_does_not_complete_getup(self):
        r=Recovery();r.start(gravity(90),0)
        for i in range(5):r.update(gravity(6),i*.02)
        self.assertEqual(r.phase,'getup')
        r.update(gravity(26),.1)
        self.assertEqual(r.stand_elapsed,0)
        for i in range(11):r.update(gravity(6),.2+i*.02)
        self.assertEqual(r.phase,'walk')

if __name__=='__main__':unittest.main()
