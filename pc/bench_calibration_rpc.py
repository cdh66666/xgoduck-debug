"""One-container manufacturer calibration with a durable before-write handshake.

Only exports source strings; importing this module never contacts the robot.
The Bridge client may calibrate only after the host has fsynced its backup.
"""
from device_config import DEVICE_SERIAL
from bench_maintenance import RPC_GUARD


CLIENT = r'''
import json,sys,time,select
started=time.monotonic()
from arduino.app_utils import Bridge
sdk_init_ms=(time.monotonic()-started)*1000
p=json.loads(sys.stdin.readline());joint=p['id'];action=p['action'];attempted=False;calls=0;before=None
ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]
timing={'sdk_init_ms':sdk_init_ms,'snapshot_ms':0.0,'calibrate_ms':0.0}
def emit(kind,**data):
 print('RPC_EVENT='+json.dumps(dict(kind=kind,**data)),flush=True)
def call(name,*args):
 global calls
 calls+=1
 return Bridge.call(name,*args,timeout=5)
def read(address,length=1):
 value=call('ft_read',joint,address,length)
 if type(value) is not int or not 0<=value<(1<<(8*length)):
  raise RuntimeError('舵机读取失败: '+str(address))
 return value
def held():
 value=call('ft_held')
 if type(value) is not int or not 0<=value<=32767:raise RuntimeError('保持状态读取失败')
 return value
def state(eeprom=None):
 return {'id':joint,'position':read(56,2),'target':read(42,2),'torque':read(40),'lock':read(55),
         'mode':eeprom[33] if eeprom is not None else read(33),
         'offset':(eeprom[31]|eeprom[32]<<8) if eeprom is not None else read(31,2),'held_mask':held(),
         'calibration_reply':None}
def receive(seconds):
 ready,_,_=select.select([sys.stdin],[],[],seconds)
 if not ready:raise RuntimeError('等待备份确认超时；不重试')
 line=sys.stdin.readline()
 if not line:raise RuntimeError('维护主机已断开')
 return json.loads(line)
def release_current():
 try:return call('ft_release',joint)==0 and read(40)==0
 except BaseException:return False
try:
 if action=='calibrate':
  stamp=time.monotonic();eeprom=[]
  for address in range(0,40,2):
   word=read(address,2);eeprom.extend((word&255,word>>8))
  before=state(eeprom);before['eeprom']=eeprom
  timing['snapshot_ms']=(time.monotonic()-stamp)*1000
  if before['torque']!=0:raise RuntimeError('请先释放当前关节，再手动摆位')
  if before['mode']!=4 or before['lock']!=1 or eeprom[:2]!=[3,46]:
   raise RuntimeError('舵机模式、锁或固件与厂家校准基线不符')
  emit('before',snapshot=before,timing_ms=timing,bridge_calls=calls)
  ack=receive(20)
  if ack!={'ack':p['token']}:raise RuntimeError('未收到本次备份确认；未写入校准')
  # stopAll clears torque but retains the MCU's per-axis one-shot flag. For an
  # explicitly repeated calibration of an already-unloaded axis, clear that flag.
  if p.get('reset_saved') and call('ft_release',joint)!=0:raise RuntimeError('当前轴校准准备卸力未确认')
  stamp=time.monotonic();attempted=True
  reply=call('ft_calibrate_hold',joint)
  if type(reply) is not int or not 0<=reply<=4095:raise RuntimeError('校准/保持失败，错误码 '+str(reply))
  result=state();result['calibration_reply']=reply
  if result['torque']!=1 or result['lock']!=1 or result['mode']!=4 or result['target']!=reply:
   raise RuntimeError('校准后保持读回不一致')
  timing['calibrate_ms']=(time.monotonic()-stamp)*1000
  emit('result',result=dict(result,ok=True,timing_ms=timing,bridge_calls=calls))
  # Keep the same client available to release this axis if software-zero commit fails.
  commit=receive(20)
  if commit!={'commit':p['token']}:
   emit('finished',committed=False,released_after_error=release_current(),bridge_calls=calls)
  else:emit('finished',committed=True,bridge_calls=calls)
 elif action=='preflight':
  stamp=time.monotonic();rows=[]
  for joint in ids:rows.append({'id':joint,'mode':read(33),'torque':read(40),'lock':read(55)})
  result={'rows':rows,'held_mask':held()};timing['preflight_ms']=(time.monotonic()-stamp)*1000
  emit('result',result=dict(result,ok=True,timing_ms=timing,bridge_calls=calls))
 elif action in ('read','release','stop'):
  if action=='release' and call('ft_release',joint)!=0:raise RuntimeError('当前关节卸力未确认')
  if action=='stop' and call('ft_stop')!=32767:raise RuntimeError('未收到全部15轴卸力读回')
  result=state();emit('result',result=dict(result,ok=True,timing_ms=timing,bridge_calls=calls))
 else:raise ValueError('Unknown action')
except BaseException as exc:
 released=release_current() if attempted else None
 emit('result',result={'ok':False,'error':str(exc),'unknown':attempted,'released_after_error':released,
                      'timing_ms':timing,'bridge_calls':calls})
'''


REMOTE = r'''
import base64,fcntl,json,os,select,stat,subprocess,sys,time,traceback
from pathlib import Path
p=json.load(sys.stdin);attempted=False;child=None;stderr_file=None;session_lock=None;result=None;events=[];buffer=b''
started=time.monotonic();timing={};docker_count=0;released=None
try:
 assert Path('/sys/devices/soc0/serial_number').read_text().strip()==DEVICE_SERIAL
 ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]
 assert p['id'] in ids and p['action'] in ('read','release','stop','calibrate','preflight')
 token=p['token'];assert len(token)==32 and all(c in '0123456789abcdef' for c in token)
 session_lock=Path('/home/arduino/.cache/xgoduck-bench-maintenance/operation.lock').open('a')
 fcntl.flock(session_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
''' + '\n'.join(' '+line for line in RPC_GUARD.splitlines()) + r'''
 stage=Path(p['stage']);log=stage/'rpc';log.mkdir(mode=0o700,exist_ok=True)
 def sync_dir(path):
  fd=os.open(path,os.O_RDONLY|os.O_DIRECTORY)
  try:os.fsync(fd)
  finally:os.close(fd)
 def write_new(path,data):
  with path.open('xb') as output:output.write(data);output.flush();os.fsync(output.fileno())
  sync_dir(path.parent)
 def encoded(value):return (json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode()
 claim=log/(token+'-attempt.json')
 if claim.exists():
  attempted=p['action']=='calibrate'
  raise RuntimeError('本次操作已尝试，不能重放；先只读核对')
 info=json.loads(subprocess.check_output(['docker','inspect','xgoduck-commissioning-main-1'],timeout=10))[0]
 assert not info['State']['Running'] and info['Image']==p['image'],'主控不在校准维护模式'
 write_new(claim,encoded({'session_id':p['session_id'],'id':p['id'],'action':p['action'],'token':token,'automatic_retry':False}))
 args=['docker','run','--rm','-i','--pull','never','--network','none','--mount',
       'type=bind,src=/var/run/arduino-router.sock,dst=/var/run/arduino-router.sock',
       '--entrypoint','python3',p['image'],'-u','-B','-c',p['client']]
 stderr_file=(log/(token+'-stderr.log')).open('xb')
 docker_started=time.monotonic();child=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stderr_file,bufsize=0,shell=False);docker_count=1
 deadline=time.monotonic()+35
 def send(value):
  child.stdin.write((json.dumps(value)+'\n').encode());child.stdin.flush()
 def event(seconds=None):
  global buffer
  until=deadline if seconds is None else min(deadline,time.monotonic()+seconds)
  while True:
   while b'\n' in buffer:
    line,buffer=buffer.split(b'\n',1)
    if line.startswith(b'RPC_EVENT='):
     value=json.loads(line[len(b'RPC_EVENT='):]);events.append(value);return value
   remaining=until-time.monotonic()
   if remaining<=0:raise TimeoutError('维护客户端超时；未自动重试')
   ready,_,_=select.select([child.stdout.fileno()],[],[],remaining)
   if not ready:raise TimeoutError('维护客户端超时；未自动重试')
   chunk=os.read(child.stdout.fileno(),65536)
   if not chunk:raise RuntimeError('维护客户端提前结束；未自动重试')
   buffer+=chunk
   if len(buffer)>256*1024:raise RuntimeError('维护客户端输出过大')
 send({'id':p['id'],'action':p['action'],'token':token,'reset_saved':p.get('reset_saved',False)})
 target=Path('/home/arduino/ArduinoApps/xgoduck-commissioning/data/zero_pos.json')
 if p['action']=='calibrate':
  first=event()
  if first['kind']!='before':
   result=first.get('result',{});raise RuntimeError(result.get('error','未收到校准前快照'))
  before=first['snapshot'];timing.update(first.get('timing_ms',{}));old=target.read_bytes();data=json.loads(old)
  assert data['ids']==ids and len(data['zero_pos'])==15 and all(type(v) is int and 0<=v<=4095 for v in data['zero_pos'])
  backup={'session_id':p['session_id'],'id':p['id'],'token':token,'before':before,
          'zero_before_base64':base64.b64encode(old).decode(),'timing_ms':timing}
  write_new(log/(token+'-before.json'),encoded(backup))
  assert target.read_bytes()==old,'软件零位被其他操作修改'
  attempted=True;send({'ack':token})
  answer=event();assert answer['kind']=='result','校准客户端回执格式错误'
  result=answer['result'];timing.update(result.get('timing_ms',{}))
  if not result.get('ok'):raise RuntimeError(result.get('error','校准未确认'))
  raw=result['calibration_reply'];assert type(raw) is int and 0<=raw<=4095
  index=ids.index(p['id']);previous=data['zero_pos'][index];data['zero_pos'][index]=raw
  assert target.read_bytes()==old,'软件零位被其他操作修改'
  metadata=target.stat();temporary=target.with_name('zero_pos.json.'+token+'.new')
  write_new(temporary,encoded(data))
  os.chown(temporary,metadata.st_uid,metadata.st_gid);os.chmod(temporary,stat.S_IMODE(metadata.st_mode))
  os.replace(temporary,target);sync_dir(target.parent)
  assert json.loads(target.read_bytes())==data,'软件零位保存读回不一致'
  result.update(saved_zero=raw,old_zero=previous,zero_pos=data['zero_pos'])
  # A durable receipt precedes COMMIT. Lost replies remain unknown and never replay.
  write_new(log/(token+'-saved.json'),encoded(result))
  send({'commit':token});finished=event()
  assert finished['kind']=='finished' and finished.get('committed') is True,'校准保存后保持状态未确认'
  result['bridge_calls']=finished.get('bridge_calls',result.get('bridge_calls'))
 else:
  answer=event();assert answer['kind']=='result'
  result=answer['result'];timing.update(result.get('timing_ms',{}))
  if not result.get('ok'):raise RuntimeError(result.get('error','操作读回失败'))
 child.stdin.close();assert child.wait(timeout=5)==0,'维护客户端退出异常'
 timing['docker_ms']=(time.monotonic()-docker_started)*1000
 result.update(ok=True,docker_count=docker_count,timing_ms=timing)
except BaseException as exc:
 if isinstance(result,dict):released=result.get('released_after_error')
 if attempted and child is not None and child.poll() is None:
  try:
   send({'abort':p['token']});answer=event(8)
   if answer.get('kind')=='finished':released=answer.get('released_after_error')
   elif answer.get('kind')=='result':released=answer.get('result',{}).get('released_after_error')
  except BaseException:pass
 result={'ok':False,'error':str(exc) or type(exc).__name__,'traceback':traceback.format_exc(),
         'unknown':attempted,'automatic_retry':False,'released_after_error':released,
         'docker_count':docker_count,'timing_ms':timing,
         'bridge_calls':result.get('bridge_calls') if isinstance(result,dict) else None}
finally:
 if child is not None:
  try:
   if child.stdin and not child.stdin.closed:child.stdin.close()
   child.wait(timeout=2)
  except BaseException:
   try:child.terminate()
   except BaseException:pass
 if stderr_file is not None:stderr_file.close()
 timing['remote_ms']=(time.monotonic()-started)*1000
 if result is None:result={'ok':False,'error':'维护结果未知','unknown':attempted,'automatic_retry':False}
 result['timing_ms']=timing
 try:
  if 'log' in locals():write_new(log/(p['token']+'-result.json'),encoded({'result':result,'events':events}))
 except BaseException as exc:
  result={'ok':False,'error':'操作记录落盘失败: '+str(exc),'unknown':attempted,'automatic_retry':False,'timing_ms':timing,'docker_count':docker_count}
 if session_lock is not None:session_lock.close()
print(json.dumps(result))
'''

REMOTE = 'DEVICE_SERIAL = ' + repr(DEVICE_SERIAL) + '\n' + REMOTE
