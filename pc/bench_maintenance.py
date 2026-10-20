"""Explicit firmware maintenance sessions for the preserved official workbench.

Construction/import is offline. Only enter()/restore() contact the bound board.
Each mutating attempt is claimed once, with no reconnect/reflash retry.
"""
from __future__ import annotations
from device_config import DEVICE_SERIAL

import base64
import copy
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import uuid
import zipfile
from active_runtime import runtime_manifest, runtime_source

IDS = [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]
REMOTE_ROOT = '/home/arduino/.cache/xgoduck-bench-maintenance'
ACTIVE_MARKER = REMOTE_ROOT + '/active-session.json'


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _atomic(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    with temporary.open('x', encoding='utf-8') as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.write('\n')
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


# Prepend this to maintenance RPC sources. It intentionally performs no writes.
RPC_GUARD = r'''
from pathlib import Path
import json
assert p.get('session_id') and p.get('stage'), 'Maintenance session required'
assert str(Path(p['stage'])) == '/home/arduino/.cache/xgoduck-bench-maintenance/'+p['session_id']
assert json.loads(Path('/home/arduino/.cache/xgoduck-bench-maintenance/active-session.json').read_text())['session_id'] == p['session_id'], 'Maintenance session changed'
assert (Path(p['stage'])/'entered.json').is_file() and not (Path(p['stage'])/'restore-attempt.json').exists(), 'Maintenance RPC not active'
'''

SDK_CLIENT = r'''
import json,sys
from arduino.app_utils import Bridge
p=json.load(sys.stdin);ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]
def read(i,a):
 v=Bridge.call('ft_read',i,a,1,timeout=3)
 if type(v) is not int or v<0:raise RuntimeError('Maintenance read failed')
 return v
result={'torque':[read(i,40) for i in ids], 'mode':[read(i,33) for i in ids],
        'lock':[read(i,55) for i in ids], 'held_mask':Bridge.call('ft_held',timeout=3)}
print('SESSION_JSON='+json.dumps(result))
'''

REMOTE = r'''
import base64,fcntl,hashlib,json,os,subprocess,sys,time,traceback,types
from pathlib import Path
from urllib.request import ProxyHandler,Request,build_opener
p=json.load(sys.stdin);mutating=False;result=None
try:
 assert Path('/sys/devices/soc0/serial_number').read_text().strip()==DEVICE_SERIAL
 session=p['session_id'];assert len(session)==32 and all(c in '0123456789abcdef' for c in session)
 root=Path('/home/arduino/.cache/xgoduck-bench-maintenance');root.mkdir(parents=True,exist_ok=True,mode=0o700)
 lock=open(root/'operation.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 stage=root/session;active=root/'active-session.json';app=Path('/home/arduino/ArduinoApps/xgoduck-commissioning')
 for name,blob in p['modules'].items():
  raw=base64.b64decode(blob,validate=True);assert hashlib.sha256(raw).hexdigest()==p['module_sha256'][name]
  module=types.ModuleType(name);sys.modules[name]=module;exec(compile(raw,'<bench-maintenance-'+name+'>','exec'),module.__dict__)
 tx=sys.modules['transaction'];m=p['manifest'];container='xgoduck-commissioning-main-1'
 def inspect(running):
  rows=json.loads(subprocess.check_output(['docker','inspect',container],timeout=10));assert len(rows)==1
  value=rows[0];assert value['Id']==m['container_id'] and value['Image']==m['image_id'] and value['State']['Running'] is running
  return value
 def check_official():
  assert tx.sources(app)==m['official_sources_sha256'],'Official source files changed'
  tx.check_files(app,dict(zip(tx.ARTIFACTS,(m['official_build']['elf_sha256'],m['official_build']['zsk_sha256']))))
  tx.validate_artifacts(*(tx.regular(app/name).read_bytes() for name in tx.ARTIFACTS))
 def zeros():
  raw=tx.regular(app/'data/zero_pos.json').read_bytes();data=json.loads(raw)
  assert data['ids']==tx.IDS and len(data['zero_pos'])==15
  assert all(type(v) is int and 0<=v<=4095 for v in data['zero_pos'])
  return raw,data['zero_pos']
 def health(expected,allow_shadow=False):
  with build_opener(ProxyHandler({})).open('http://127.0.0.1:9527/api/status',timeout=5) as response:s=json.load(response)
  assert s.get('deployment',{}).get('reference')=='rl/raspi_deploy'
  assert s['mode'] in (('off','shadow') if allow_shadow else ('off',)) and s['enabled'] is False and s['full_robot_ready'] is True
  assert s['raw_mask']==32767 and s['zero_pos']==expected
  assert 0<=s['feedback_age_ms']<150 and s['mcu_command_seq']>0 and 0<=s['mcu_command_age_us']<150000
  assert not s.get('mcu_calibrating',False) and not s.get('mcu_servo_debug',False)
  return {'mode':s['mode'],'enabled':False,'zero_pos':s['zero_pos'],'all15_fresh':True}
 def board():return sys.modules['board_adapter'].adapter(base64.b64decode(p['helper']),p['board_ip'],base64.b64decode(p['router']))
 def sdk():
  inspect(False)
  run=subprocess.run(['docker','run','--rm','-i','--pull','never','--network','none','--mount',
   'type=bind,src=/var/run/arduino-router.sock,dst=/var/run/arduino-router.sock','--entrypoint','python3',m['image_id'],'-B','-c',p['client']],
   input=b'{}',capture_output=True,timeout=100,shell=False)
  tx.new(stage/('sdk-read-'+str(time.time_ns())+'.log'),run.stdout+b'\n'+run.stderr)
  rows=[row[13:] for row in run.stdout.splitlines() if row.startswith(b'SESSION_JSON=')]
  assert run.returncode==0 and len(rows)==1,'Maintenance read unconfirmed'
  value=json.loads(rows[0]);assert value['torque']==[0]*15 and value['held_mask']==0,'Release all joints before firmware restore'
  assert value['mode']==[4]*15 and value['lock']==[1]*15,'Unexpected servo mode or EEPROM lock'
  return value
 check_official()
 if p['phase']=='enter':
  inspect(True)
  if active.exists():
   mutating=True
   raise RuntimeError('An earlier maintenance session is unresolved; read-only diagnosis required')
  raw,baseline=zeros();observed=health(baseline);b=board();identity=b.identity()
  assert identity['serial']==DEVICE_SERIAL
  stage.mkdir(mode=0o700)
  tx.new(stage/'manifest.json',tx.encoded(m));tx.new(stage/'identity.json',tx.encoded(identity))
  tx.new(stage/'zero_pos.before.json',raw)
  for name in m['official_sources_sha256']:tx.new(stage/'official-backup'/name,tx.regular(app/name).read_bytes())
  for name in tx.ARTIFACTS:tx.new(stage/'official-backup'/name,tx.regular(app/name).read_bytes())
  for name,blob in p['files'].items():
   tx.safe_name(name);contents=base64.b64decode(blob,validate=True)
   assert tx.sha(contents)==m['maintenance_files_sha256'][name]
   tx.new(stage/name,contents)
  b.audit_upload(stage,m['maintenance_build'],m['upload_contract_sha256'])
  assert zeros()[0]==raw,'Zero file changed during preflight';check_official();health(baseline)
  tx.new(stage/'enter-attempt.json',tx.encoded({'session_id':session,'automatic_retry':False}))
  tx.new(active,tx.encoded({'session_id':session,'stage':str(stage)}));mutating=True
  b.stop(app,stage/'host-stop.log');inspect(False)
  assert zeros()[0]==raw;check_official();b.refresh_serial()
  b.upload(stage,stage/'maintenance-upload.log',m['maintenance_build'])
  off=sdk()
  result={'ok':True,'phase':'maintenance','session_id':session,'stage':str(stage),'baseline_zero':baseline,
          'image_id':m['image_id'],'official_identity':identity,'off':off,'motion_resume_requested':False}
  tx.new(stage/'entered.json',tx.encoded(result))
 elif p['phase']=='restore':
  inspect(False);assert json.loads(active.read_text())['session_id']==session
  assert json.loads((stage/'manifest.json').read_text())==m and (stage/'entered.json').is_file()
  assert not (stage/'restore-attempt.json').exists(),'Restore already attempted; inspect before any recovery'
  baseline=json.loads((stage/'zero_pos.before.json').read_text())['zero_pos']
  ledger=p['calibration_state'];assert not ledger.get('unknown'),'Unconfirmed calibration; read-only diagnosis required'
  expected=list(baseline)
  for key,value in ledger.get('saved',{}).items():
   joint=int(key);assert str(joint)==key and joint in tx.IDS
   zero=value['zero'];assert type(zero) is int and 0<=zero<=4095;expected[tx.IDS.index(joint)]=zero
  raw,current=zeros();assert current==expected,'Official zero file has not synchronized with this session'
  off=sdk();tx.new(stage/('restore-preflight-'+str(time.time_ns())+'.json'),tx.encoded({'zero_pos':expected,'ledger':ledger,'off':off}))
  b=board();b.audit_upload(app,m['official_build'],m['upload_contract_sha256'])
  check_official();assert zeros()[0]==raw
  tx.new(stage/'restore-attempt.json',tx.encoded({'session_id':session,'automatic_retry':False}));mutating=True
  b.refresh_serial();b.upload(app,stage/'official-restore-upload.log',m['official_build'])
  # Seed the frozen helper's stopped-container SDK contract from separately pinned source.
  ns={'__name__':'bench-official-host-restart'};exec(compile(base64.b64decode(p['helper']),'<frozen-upload-helper>','exec'),ns)
  low=ns['Board'](board_ip=p['board_ip']);low.sdk_socket=ns['sdk_socket_default'](p['sdk_source'])
  identity=json.loads((stage/'identity.json').read_text());identity.pop('serial')
  actual=low.container_identity(False);actual['mounts']=sorted(actual['mounts'],key=lambda row:row['Destination'])
  assert actual==identity,'Stopped container/SDK identity changed'
  low.command(['docker','start',container],app,stage/'official-host-start.log',60)
  inspect(True);check_official();assert zeros()[0]==raw
  # Official startup enters disarmed shadow inference. Read it first, then claim
  # exactly one authenticated OFF request; never replay a lost POST response.
  deadline=time.monotonic()+15;last=None
  while time.monotonic()<deadline:
   try:health(expected,allow_shadow=True);break
   except (OSError,ValueError,KeyError,AssertionError) as exc:last=exc;time.sleep(.25)
  else:raise RuntimeError('Restored Host disarmed startup/feedback unconfirmed: '+str(last))
  key=tx.regular(app/'data/calibration_key').read_text().strip();assert key,'Local official API authentication unavailable'
  tx.new(stage/'restore-off-attempt.json',tx.encoded({'session_id':session,'off_mode_requests':1,'automatic_retry':False}))
  request=Request('http://127.0.0.1:9527/api/mode',data=b'{"mode":"off"}',headers={'Content-Type':'application/json','X-Cal-Key':key},method='POST')
  with build_opener(ProxyHandler({})).open(request,timeout=5) as response:off_reply=json.load(response)
  assert off_reply.get('mode')=='off','Restored Host OFF request was not acknowledged'
  deadline=time.monotonic()+15;last=None
  while time.monotonic()<deadline:
   try:verified=health(expected);break
   except (OSError,ValueError,KeyError,AssertionError) as exc:last=exc;time.sleep(.25)
  else:raise RuntimeError('Restored Host OFF/feedback unconfirmed: '+str(last))
  result={'ok':True,'phase':'restored','session_id':session,'expected_zeros':expected,'runtime':verified,
          'firmware_upload_calls':1,'host_start_calls':1,'off_mode_requests':1,'motion_resume_requested':False}
  tx.new(stage/'restored.json',tx.encoded(result));active.unlink();tx.fsync_dir(root)
 else:raise ValueError('Unknown phase')
except BaseException as exc:
 result={'ok':False,'error':str(exc),'unknown_outcome':mutating,'automatic_retry':False,'phase':p.get('phase'),'traceback':traceback.format_exc()}
 try:
  if 'stage' in locals() and stage.is_dir():tx.new(stage/('failure-'+str(time.time_ns())+'.json'),tx.encoded(result))
 except BaseException:pass
print(json.dumps(result))
'''


class MaintenanceSession:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.store = self.root/'bench-results'/'maintenance'
        self.state_path = self.store/'current.json'
        self._lock = threading.RLock()
        self._operation = threading.Lock()
        self.state = {'phase':'official','active':False,'error':'','session_id':None,'unknown':False}
        if self.state_path.exists():
            self.state.update(json.loads(self.state_path.read_text('utf-8')))
            if self.state['phase'] in ('entering','restoring'):
                self.state.update(phase='unknown',active=True,unknown=True,error='上次维护操作中断，必须先只读核对，未自动重试')
                self._save()

    @property
    def session_dir(self):
        return self.store/self.state['session_id'] if self.state.get('session_id') else None

    @property
    def baseline_zero(self):
        return copy.deepcopy(self.state.get('baseline_zero', []))

    @property
    def expected_zeros(self):
        return copy.deepcopy(self.state.get('expected_zeros'))

    def _save(self):
        _atomic(self.state_path,self.state)

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self.state)

    def rpc_ready(self):
        with self._lock:
            return self.state['phase']=='maintenance' and not self.state.get('unknown')

    def rpc_payload(self):
        with self._lock:
            if not self.rpc_ready():raise RuntimeError('尚未进入已验证的校准维护模式')
            return {'image':self.state['image_id'],'session_id':self.state['session_id'],'stage':self.state['remote_stage']}

    def update_calibration_state(self, state):
        with self._lock:
            if self.state['phase'] not in ('maintenance','restored') or self.state.get('unknown'):
                raise RuntimeError('维护模式未就绪')
            if state.get('session_id',self.state['session_id'])!=self.state['session_id']:
                raise RuntimeError('校准台账不属于本次会话')
            value=copy.deepcopy(state);value['session_id']=self.state['session_id']
            _atomic(self.session_dir/'calibration-state.json',value)

    def _payload(self, phase):
        pure=self.root/'official-pure-20261019';source=self.root/'feetech-zero-20261019'
        pin=json.loads((pure/'installer/install-manifest.json').read_text('utf-8'))
        deploy=runtime_manifest(self.root)
        sdk=json.loads((source/'sdk-read.json').read_text('utf-8'))
        modules={name:(pure/'installer'/f'{name}.py').read_bytes() for name in ('transaction','board_adapter')}
        for name,raw in modules.items():
            if _sha(raw)!=pin['installer_sha256'][name]:raise ValueError('冻结上传助手已变化')
        expected=deploy['new_source_manifest']
        if len(expected)!=28:raise ValueError('正式28文件清单不完整')
        for name,digest in expected.items():
            path=runtime_source(self.root,name,deploy)
            if _sha(path.read_bytes())!=digest:raise ValueError('本地正式来源哈希不匹配: '+name)
        audit=json.loads((source/'build-audit.json').read_text('utf-8'))
        if audit['compile_exit_code']!=0 or audit['uploaded'] is not False:raise ValueError('维护固件构建未通过')
        for name,digest in audit['source_sha256'].items():
            if _sha((source/'sketch'/name).read_bytes())!=digest:raise ValueError('维护固件来源已变化: '+name)
        files={p.relative_to(source).as_posix():p.read_bytes() for p in (source/'sketch').rglob('*') if p.is_file()}
        files['sketch/sketch.yaml']=(pure/'candidate/sketch/sketch.yaml').read_bytes()
        for name,digest in audit['artifacts'].items():
            raw=(source/'build'/name).read_bytes()
            if _sha(raw)!=digest:raise ValueError('维护固件产物已变化: '+name)
            files['.cache/sketch/'+name]=raw
        manifest={'schema':'bench-maintenance-session-v1','session_id':self.state['session_id'],
                  'container_id':sdk['container_id'],'image_id':sdk['image_id'],
                  'official_sources_sha256':expected,'official_build':pin['build'],
                  'maintenance_files_sha256':{name:_sha(raw) for name,raw in files.items()},
                  'maintenance_build':{'elf_sha256':audit['artifacts']['sketch.ino.elf'],'zsk_sha256':audit['artifacts']['sketch.ino.elf-zsk.bin']},
                  'upload_contract_sha256':pin['upload_contract_sha256']}
        with zipfile.ZipFile(self.root/'downloads/xgoduck-slow-home-hold-v7-20261015.zip') as archive:
            helper=archive.read('operator/remote_install_helper.py');router=archive.read('operator/router_serial_link.py')
        value={'phase':phase,'session_id':self.state['session_id'],'manifest':manifest,'client':SDK_CLIENT,
               'modules':{name:base64.b64encode(raw).decode() for name,raw in modules.items()},
               'module_sha256':{name:_sha(raw) for name,raw in modules.items()},
               'files':{name:base64.b64encode(raw).decode() for name,raw in files.items()},
               'helper':base64.b64encode(helper).decode(),'router':base64.b64encode(router).decode(),'sdk_source':sdk['sdk_source']}
        if phase=='enter':
            _atomic(self.session_dir/'manifest.json',manifest)
        elif manifest!=json.loads((self.session_dir/'manifest.json').read_text('utf-8')):
            raise ValueError('当前上传清单与本会话不符')
        return value

    def _run(self,phase,payload):
        import model_transport
        payload['board_ip']=model_transport._configuration(self.root)[1]
        result=model_transport.execute(self.root,REMOTE,payload,timeout=300)
        _atomic(self.session_dir/(phase+'-result.json'),result)
        return result

    def enter(self):
        if not self._operation.acquire(blocking=False):raise RuntimeError('维护切换正在进行')
        dispatched=False;started=False
        try:
            with self._lock:
                if self.state['active']:raise RuntimeError('已有活动或待核对的维护会话')
                expected=self.state.get('expected_zeros')
                self.state={'phase':'entering','active':True,'error':'','unknown':False,'session_id':uuid.uuid4().hex,'created_at':time.time(),'expected_zeros':expected}
                started=True
                self._save()
            payload=self._payload('enter');dispatched=True;result=self._run('enter',payload)
            with self._lock:
                if result.get('ok'):
                    self.state.update(phase='maintenance',active=True,baseline_zero=result['baseline_zero'],image_id=result['image_id'],remote_stage=result['stage'])
                    _atomic(self.session_dir/'calibration-state.json',{'session_id':self.state['session_id'],'saved':{},'unknown':{},'held_mask':0})
                else:
                    unknown=bool(result.get('unknown_outcome',True))
                    self.state.update(phase='unknown' if unknown else 'official',active=unknown,unknown=unknown,error=result.get('error','维护切换失败'))
                self._save()
            return self.snapshot()
        except BaseException as exc:
            with self._lock:
                # A rejected concurrent/current-session request must not reset its state.
                if started:
                    self.state.update(phase='unknown' if dispatched else 'official',active=dispatched,unknown=dispatched,error=str(exc));self._save()
            raise
        finally:self._operation.release()

    def restore(self,calibration_state=None):
        if not self._operation.acquire(blocking=False):raise RuntimeError('维护切换正在进行')
        dispatched=False;started=False
        try:
            with self._lock:
                if not self.rpc_ready():raise RuntimeError('只能从已验证的维护模式恢复')
                durable=json.loads((self.session_dir/'calibration-state.json').read_text('utf-8'))
                if calibration_state is None:calibration_state=durable
                if calibration_state.get('session_id',self.state['session_id'])!=self.state['session_id']:
                    raise RuntimeError('校准台账不属于本次会话')
                if calibration_state.get('unknown') or durable.get('unknown'):
                    raise RuntimeError('存在未确认校准，先只读核对，不能恢复模型')
                if any(calibration_state.get(key,{})!=durable.get(key,{}) for key in ('saved','unknown')):
                    raise RuntimeError('校准台账与本次持久记录不一致，先刷新核对')
                _atomic(self.session_dir/'calibration-state.json',calibration_state)
                self.state.update(phase='restoring',error='');started=True;self._save()
            payload=self._payload('restore');payload['calibration_state']=calibration_state
            dispatched=True;result=self._run('restore',payload)
            with self._lock:
                if result.get('ok'):
                    self.state.update(phase='restored',active=False,unknown=False,expected_zeros=result['expected_zeros'])
                else:
                    unknown=bool(result.get('unknown_outcome',True))
                    self.state.update(phase='unknown' if unknown else 'maintenance',active=True,unknown=unknown,error=result.get('error','官方恢复失败'))
                self._save()
            return self.snapshot()
        except BaseException as exc:
            with self._lock:
                if started:
                    self.state.update(phase='unknown' if dispatched else 'maintenance',active=True,unknown=dispatched,error=str(exc));self._save()
            raise
        finally:self._operation.release()

REMOTE = 'DEVICE_SERIAL = ' + repr(DEVICE_SERIAL) + '\n' + REMOTE
