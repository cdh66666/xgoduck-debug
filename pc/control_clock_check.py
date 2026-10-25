"""Exercise an actual Controller.run with one small overrun, without hardware."""
import ast
from types import SimpleNamespace

def check_50hz_loop(source):
    tree=ast.parse(source)
    owner=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Controller')
    method=next(n for n in owner.body if isinstance(n,ast.FunctionDef) and n.name=='run')
    clock=SimpleNamespace(now=0.)
    namespace={'time':SimpleNamespace(monotonic=lambda:clock.now),'CONTROL_PERIOD':.02}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'<control-clock-check>','exec'),namespace)
    class Harness:
        def __init__(self):
            self.starts=[];self.loop_missed=0;self.idle_calls=0
            self.stop_event=SimpleNamespace(is_set=lambda:clock.now>=1.-1e-9,wait=self.wait)
        def wait(self,seconds):clock.now+=max(0.,seconds)
        def step(self):
            self.starts.append(clock.now)
            clock.now+=.023 if len(self.starts)==11 else .002
        def _keep_warm_idle(self):self.idle_calls+=1;clock.now+=.030
    harness=Harness();namespace['run'](harness)
    report={'steps_in_one_second':len(harness.starts),'idle_model_calls':harness.idle_calls,
            'missed':harness.loop_missed,'one_step_overrun_ms':23}
    if len(harness.starts)!=50 or harness.idle_calls or harness.loop_missed:
        raise ValueError('50 Hz loop contract failed: '+str(report))
    return report
