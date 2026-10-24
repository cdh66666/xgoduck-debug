"""Exercise the actual HTTP function guards offline; no actuator calls allowed."""
import ast
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]


class Locked(Exception):
    def __init__(self, code, detail):
        self.status_code, self.detail=code,detail


class MotionInhibitTests(unittest.TestCase):
    def test_all_motion_entries_reject_before_controller_when_in_tray(self):
        source=ast.parse((ROOT/'robot-app/python/main.py').read_text('utf-8'))
        names={'require_motion_permission','mode','command','pick','roulade','cal_enter',
               'cal_targets','cal_finish','servo_enter','servo_action','factory_action'}
        functions=ast.Module(body=[n for n in source.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[])
        with tempfile.TemporaryDirectory() as folder:
            lock=Path(folder)/'motion-inhibit';lock.write_text('tray')
            calls=[]
            controller=SimpleNamespace(set_mode=lambda value:calls.append(('mode',value)),
                                       factory_action=lambda *args:calls.append(('factory',args)))
            namespace={'HTTPException':Locked,'motion_inhibit_file':lock,'controller':controller,
                       'motion_recorder':SimpleNamespace(mark_event=lambda *args:None)}
            exec(compile(functions,'main.py','exec'),namespace)
            cases={'mode':{'mode':'policy'},'command':{'twist':[0,0,0]},'pick':{},'roulade':{},
                   'cal_enter':{},'cal_targets':{'targets':[0]*15},'cal_finish':{'index':0},
                   'servo_enter':{},'servo_action':{'action':'goto','target_id':10},
                   'factory_action':{'action':'calibrate','target_id':10}}
            for name,body in cases.items():
                with self.subTest(route=name),self.assertRaises(Locked) as raised:
                    namespace[name](body)
                self.assertEqual(raised.exception.status_code,423)
            self.assertEqual(calls,[])
            namespace['mode']({'mode':'off'})
            namespace['factory_action']({'action':'off','target_id':10})
            self.assertEqual(calls,[('mode','off'),('factory',('off',10))])


if __name__=='__main__':unittest.main()
