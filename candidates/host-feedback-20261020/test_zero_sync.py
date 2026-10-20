"""Execute unchanged Controller method ASTs with fake clocks/Bridge/models only."""
from pathlib import Path
import ast, collections, json, math, sys, threading, types, importlib.util
ROOT=Path(__file__).resolve().parents[2]/'pc';OUT=Path(__file__).resolve().parent
import atexit,tempfile
_fixture_tmp=tempfile.TemporaryDirectory(prefix='duck-host-fixture-')
atexit.register(_fixture_tmp.cleanup)
FIXTURE=Path(_fixture_tmp.name)
sys.path.insert(0,str(ROOT));import active_runtime
for name in ['wire','rl_core']:
    path=active_runtime.runtime_source(ROOT,'python/'+name+'.py')
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m)
w=sys.modules['wire'];core=sys.modules['rl_core']
class Clock:
    now=100.
    def monotonic(self):return self.now
    def strftime(self,*a):return 'offline-fixture'
    def time(self):return self.now+1700000000
    def time_ns(self):return int(self.time()*1000000000)
clock=Clock()
class Bridge:
    def __init__(self):self.sent=[]
    def notify(self,name,payload):self.sent.append((clock.now,name,payload))
class Policy:
    file='fixture.onnx';action_scale=1.;action_alpha=.45;action_alpha_head=.45;callback=None
    def load(self,*a):return True,'fixture'
    def reset_action_filter(self):pass
    def warm(self,*a):pass
    def infer(self,*a):
        if self.callback:self.callback()
        return [0.]*14
    def action_to_deg(self,*a):return list(core.DEFAULT_DEG)
class Filter:
    gyro_alpha=.5;vel_alpha=.4
    def reset(self):pass
    def update(self,acc,gyro,angles,velocities,**k):return dict(grav=[0,0,-1],gyro_rad=[0]*3,angles_deg=angles,vels_deg=velocities,imu_fusion_ready=True,imu_fusion_updated=True)
class Recovery:
    phase='walk';fall_elapsed=0
    def reset(self):self.phase='walk'
    def update(self,*a):return False
    def status(self):return dict(phase=self.phase)
class Keyboard:
    def _keyboard_init(self):
        self.fast_policy=Policy();self.navigation_policy=Policy();self.fast_active=self.navigation_active=False
        self.manual_pick_phase=self.manual_pick_target=0;self._keyboard_manual_blending=False
    def _keyboard_cancel_locked(self):pass
    def _keyboard_select_locked(self,s,t,h,m,**k):return s,t,h,m
    def _keyboard_state_locked(self,*a):return {}
class Factory:
    def _factory_init(self,*a):self.factory_pending=None;self.factory_active=False;self.factory_resets=0
    def _factory_reboot(self):self.factory_resets+=1
    def factory_status(self):return dict(pending=self.factory_pending)
class Display:
    def snapshot(self,*a):return {}
class Current:
    def submit(self,*a):pass
    def snapshot(self,*a):return {}
def atomic_json(p,value):
    p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value),encoding='utf-8')
def load(path):
    module=ast.parse(path.read_text('utf-8'));cls=next(n for n in module.body if isinstance(n,ast.ClassDef) and n.name=='Controller')
    env={**vars(w),'Policy':Policy,'Recovery':Recovery,'SignalFilter':Filter,'KeyboardRuntimeMixin':Keyboard,'FactoryCalibrationMixin':Factory,'DisplayTelemetryCache':Display,'CurrentTelemetry':Current,'atomic_json':atomic_json,'collections':collections,'json':json,'math':math,'threading':threading,'time':clock,'Path':Path,'ZERO_PATH':FIXTURE/'zero_pos.json','FACTORY_ZERO':[2236,2005,1912,1866,1945,2127,2135,1914,2228,2206,2550,2060,2128,2034,2190],'EXCLUSIVE_MODES':('calibrate','servo_debug'),'PICK_PERIOD_S':4.,'PICK_MOUTH_CLOSE_PHI':.4,'ROULADE_S':1.9,'ROULADE_ALPHA':.15,'CONTROL_PERIOD':.02}
    for name in ['MOTOR_IDS','HOME_DEG','DEFAULT_DEG','GRAVITY_TAU','IMU_BIAS_DEG','MOUTH_INDEX','MOUTH_MAX_DEG']:env[name]=getattr(core,name)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),env)
    return env['Controller'],env
def fixture(path):
    clock.now=100.;cls,env=load(path);env['ZERO_PATH'].parent.mkdir(exist_ok=True)
    atomic_json(env['ZERO_PATH'],{'zero_pos':[2048]*15});b=Bridge();c=cls(b)
    c.feedback=types.SimpleNamespace(imu_ok=True,mask=32767,sample_us=10000,imu_us=10010,seq=1000,angles=[1.]*15,velocities=[0.]*15,targets=[0.]*15,enabled=False,flags=1,period_us=10000,work_us=7500,overruns=0,report_drops=0,invalid_commands=0,command_seq=1,command_age_us=0,acc=[0,0,-9.8],gyro=[0]*3,calibrating=False,servo_debugging=False)
    c.filtered=c.filter.update([0,0,-9.8],[0]*3,[1.]*15,[0.]*15);c.rx_time=clock.now;c.web_time=clock.now
    return c,b,env
def raw(zeros,mask=32767):return w.RAW.pack(b'DQR1',mask,*([2048]*15),*zeros)
def blocked(c):
    try:c._require_feedback(0)
    except ValueError:return True
    return False
def baseline():
    source=active_runtime.runtime_source(ROOT,'python/controller.py');c,b,env=fixture(source)
    original=list(c.zero_pos);stale=list(env['FACTORY_ZERO']);c.receive_raw(raw(stale))
    v=dict(reproduced=c.zero_pos==stale and c._zeros_pushed and len(b.sent)==1 and not blocked(c),authority_before=original,memory_after_first_stale_RAW=list(c.zero_pos),disk_after=json.loads(env['ZERO_PATH'].read_text('utf-8'))['zero_pos'],marked_pushed=c._zeros_pushed,arm_feedback_guard_passed=not blocked(c),bridge_notifications=len(b.sent),device_requests=0,meaning='notify is not an ACK; same old RAW overwrites authoritative in-memory zeros')
    assert v['reproduced'];(FIXTURE/'baseline-bug-fixture.json').write_text(json.dumps(v,indent=2),encoding='utf-8');return v
if __name__=='__main__':
    result=baseline();print(json.dumps(result,indent=2))
    if '--baseline-only' not in sys.argv:
        label=sys.argv[sys.argv.index('--candidate')+1] if '--candidate' in sys.argv else 'candidate'
        assert label in ('candidate','combined-candidate')
        candidate=OUT/label/'python/controller.py';tests=[]
        c,b,e=fixture(candidate);old=list(e['FACTORY_ZERO']);c.receive_raw(raw(old));assert c.zero_pos==[2048]*15 and not c._zeros_pushed and blocked(c);assert c._mcu_zero_pos==old
        for i in range(1,100):clock.now=100+i*.01;c.receive_raw(raw(old))
        sends=[t for t,n,p in b.sent];assert len(sends)<=4 and all(y-x>=.25-1e-9 for x,y in zip(sends,sends[1:]));assert all(n=='duck_cal' and w.CAL.unpack(p)[1]==w.CAL_SET_ZEROS for t,n,p in b.sent)
        tests.append('stale RAW preserves disk authority, cannot ARM, idempotent RAM retries at most 4Hz')
        clock.now=101.;c.rx_time=clock.now;c.receive_raw(raw([2048]*15));assert c._zeros_pushed and not blocked(c)
        s=c.status();assert s['zero_sync']['confirmed'] and s['zero_sync']['mcu_zero_pos']==[2048]*15 and s['control_ready'] and s['full_robot_ready'];tests.append('only subsequent matching RAW confirms; status exposes MCU zeros and readiness')
        c,b,e=fixture(candidate);c.receive_raw(raw([2048]*15));assert not c._zeros_pushed and blocked(c);c.receive_raw(raw([2048]*15));assert c._zeros_pushed;tests.append('first already-matching packet sends RAM request but cannot self-ACK')
        c,b,e=fixture(candidate);c.factory_pending={'target_id':10};c.receive_raw(raw(old));c.receive_raw(raw([2048]*15));assert not b.sent and c.zero_pos==[2048]*15 and blocked(c) and not c.status()['zero_sync']['confirmed'];tests.append('unknown pending factory transaction forbids all zero pushes and confirmation')
        c.factory_pending=None;c.receive_raw(raw([2048]*15));assert len(b.sent)==1 and not c._zeros_pushed;c.receive_raw(raw([2048]*15));assert c._zeros_pushed
        c.zero_pos[0]=2123;c._save_zeros();assert not c._zeros_pushed and blocked(c);assert json.loads(e['ZERO_PATH'].read_text('utf-8'))['zero_pos'][0]==2123
        c.receive_raw(raw([2048]*15));assert c.zero_pos[0]==2123 and not c._zeros_pushed;c.receive_raw(raw([2123]+[2048]*14));assert c._zeros_pushed;tests.append('persisted factory-result authority invalidates ACK; old RAW cannot roll it back')
        c,b,e=fixture(candidate);p=active_runtime.runtime_source(ROOT,'python/factory_calibration.py');tree=ast.parse(p.read_text('utf-8'));fc=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='FactoryCalibrationMixin');scope={'time':clock,'atomic_json':atomic_json,'Path':Path};exec(compile(ast.Module(body=[fc],type_ignores=[]),str(p),'exec'),scope)
        c.factory_action=types.MethodType(scope['FactoryCalibrationMixin'].factory_action,c);c.factory_dir=FIXTURE/'factory_case';c.mode='servo_debug';c.factory_active=True;operations=[]
        def fake_factory_request(action,id):
            operations.append(action)
            return dict(ok=True,zero_changed=action=='calibrate',zero_pos=2123 if action=='calibrate' else 2048,position=2123 if action=='calibrate' else 2048)
        c._factory_request=fake_factory_request;r=c.factory_action('calibrate',10);assert r['host_zero_saved'] and c.zero_pos[0]==2123 and c.factory_pending is None and not c._zeros_pushed
        for i in range(50):clock.now=100+i*.01;c.receive_raw(raw([2048]*15))
        assert operations==['read','calibrate'] and c.zero_pos[0]==2123;tests.append('actual factory_action saves authoritative result; RAM retries never repeat EEPROM calibration')
        packet=w.STATE.pack(b'DQS1',1,1,32767,*[1,20000,20010,2,0,10000,7500,0,0,0],*[0,0,-16384,0,0,0],*([1.]*15),*([0.]*15),*([0.]*15))
        c.receive(packet);assert c.factory_resets==1 and c.mode=='shadow' and not c._zeros_pushed and c._zero_push_at is None and c._mcu_zero_pos is None and blocked(c);c.receive_raw(raw([2048]*15));assert not c._zeros_pushed;tests.append('MCU sequence restart clears confirmation and requires fresh resynchronization')
        c,b,e=fixture(candidate);c.receive_raw(raw([2048]*15));c.receive_raw(raw([2048]*15));c.set_mode('policy');c.step();assert w.COMMAND.unpack(b.sent[-1][2])[4]==w.ARM and list(w.COMMAND.unpack(b.sent[-1][2])[7:])==[1.]*15
        c.step();assert w.COMMAND.unpack(b.sent[-1][2])[4]==w.POLICY;tests.append('confirmed normal ARM still uses measured angles, then unchanged POLICY command')
        c.receive_raw(raw(old));assert not c._zeros_pushed;c.step();assert c.mode=='shadow' and w.COMMAND.unpack(b.sent[-1][2])[4]==w.DISARM;tests.append('loss of matching MCU zeros takes existing disarm path')
        c,b,e=fixture(candidate);c.receive_raw(raw([2048]*15));c.receive_raw(raw([2048]*15));c.mode='policy';c.feedback.enabled=True;count=len(b.sent);c.receive_raw(raw(old));assert len(b.sent)==count and not c._zeros_pushed;c.step();assert w.COMMAND.unpack(b.sent[-1][2])[4]==w.DISARM
        c.feedback.enabled=False;clock.now+=.25;c.rx_time=clock.now;count=len(b.sent);c.receive_raw(raw(old));assert len(b.sent)==count+1 and b.sent[-1][1]=='duck_cal';tests.append('active MCU is disarmed before retrying RAM zero reference writes')
        c,b,e=fixture(candidate);c.receive_raw(raw([2048]*15));c.receive_raw(raw([2048]*15));c.set_mode('policy');c.policy.callback=lambda:c.receive_raw(raw(old));c.step();assert w.COMMAND.unpack(b.sent[-1][2])[4]==w.DISARM;tests.append('RAW mismatch during inference cannot send obsolete ARM/POLICY')
        c,b,e=fixture(candidate);s=c.status();assert not s['full_robot_ready'] and not s['control_ready'];before=list(c.zero_pos);c.receive_raw(b'bad');assert c.zero_pos==before and not b.sent;tests.append('unconfirmed readiness false and malformed RAW has no effect')
        name='zero-sync-verification.json' if label=='candidate' else 'combined-zero-sync-verification.json'
        v=dict(ok=True,device_requests=0,tests=tests,candidate_only=True,candidate_label=label,baseline_reproduced=result['reproduced']);(FIXTURE/name).write_text(json.dumps(v,indent=2),encoding='utf-8');print(json.dumps(v,indent=2))
