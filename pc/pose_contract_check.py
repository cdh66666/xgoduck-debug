"""Execute a native controller's real step() with a fake bridge, never hardware."""
import ast,collections,contextlib,math,runpy,types
from pathlib import Path

def check_native_poses(source,wire_path):
    cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='Controller')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='step')
    wire=runpy.run_path(str(wire_path));scope=dict(wire)
    scope.update(time=types.SimpleNamespace(monotonic=lambda:100.),math=math,EXCLUSIVE_MODES=('calibrate','servo_debug'),
                 CONTROL_PERIOD=.02,MOUTH_INDEX=14,MOUTH_MAX_DEG=30.,PICK_PERIOD_S=4.,PICK_MOUTH_CLOSE_PHI=.4,_trace_push=None)
    exec(compile(ast.Module(body=[method],type_ignores=[]),'actual-native-controller-step','exec'),scope)
    default=[0,-5,-22,0,18,0,5,22,0,-18,20,20,0,0,0]
    results={}
    for mode,arm in [('hold',False),('policy',False),('hold',True)]:
        sent=[];policy=types.SimpleNamespace(default_deg=default,reset_action_filter=lambda:None,warm=lambda n:None,
                   infer=lambda *a:[0.]*14,action_to_deg=lambda a:list(default))
        self=types.SimpleNamespace(lock=contextlib.nullcontext(),policy_lock=contextlib.nullcontext(),
            feedback=types.SimpleNamespace(imu_ok=True,angles=[float(i) for i in range(15)],sample_us=1234),
            filtered={'grav':[0.,0.,-1.],'gyro_rad':[0.]*3,'angles_deg':[0.]*15,'vels_deg':[0.]*15,'imu_fusion_ready':True,'imu_fusion_updated':True},
            generation=1,mode=mode,rx_time=100.,web_time=100.,twist=[0.]*3,head=[0.]*4,mouth=7.,roulade_until=0.,
            _stale_joint_ids=lambda now:[],_roulade_on=lambda now:False,_stop_for_tray_locked=lambda:False,
            recovery=types.SimpleNamespace(phase='walk',update=lambda *a:False),pick_active=False,sit_active=False,
            policy_needs_reset=False,policy=policy,arm_pending=arm,seq=0,sent=0,inferred=0,
            infer_times=collections.deque(),infer_ms=collections.deque(),bridge=types.SimpleNamespace(notify=lambda name,payload:sent.append((name,payload))))
        scope['step'](self)
        assert len(sent)==1 and sent[0][0]=='duck_command'
        packet=wire['COMMAND'].unpack(sent[0][1]);target=list(packet[7:])
        key='initial_enable' if arm else mode
        expected=list(self.feedback.angles) if arm else [0.]*15 if mode=='hold' else default[:-1]+[7.]
        if target!=expected:raise ValueError(f'{key} sent {target}, expected {expected}')
        assert self.inferred==(1 if mode=='policy' else 0)
        results[key]={'wire_mode':packet[4],'targets':target,'inferences':self.inferred}
    return results
