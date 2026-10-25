"""Read actual board identities and install the locally prepared, hash-bound release."""
import json
from pathlib import Path
import threading
import time
import uuid
from model_transport import execute

REMOTE=r'''
import hashlib,json,pathlib,shutil,subprocess,sys,urllib.request
p=json.load(sys.stdin);root=pathlib.Path('/home/arduino/ArduinoApps/xgoduck-commissioning')
def api(path):
 with urllib.request.urlopen('http://127.0.0.1:9527/api/'+path,timeout=3) as f:return json.load(f)
def digest(file):return hashlib.sha256(file.read_bytes()).hexdigest()
def relative(value):
 q=pathlib.Path(value)
 if q.is_absolute() or '..' in q.parts:raise ValueError('Invalid release path')
 return q
release=p.get('release') or {};stage=root/'.codex-stage'/relative(release.get('stage','absent'))
def candidate():
 result={'ready':False,'version':release.get('version'),'reason':'尚未准备更新包'}
 if not release:return result
 b=stage/'compile-result.json'
 if not b.exists():return result
 build=json.loads(b.read_text());result['reason']=build.get('phase','unknown')
 if build.get('phase')!='compiled':return result
 if digest(stage/'build/sketch.ino.elf')!=release.get('elf_sha256'):result['reason']='固件校验不匹配';return result
 cfg=json.loads((stage/'candidate.json').read_text())
 if cfg.get('sha')!=release.get('files'):result['reason']='程序清单不匹配';return result
 for n,h in cfg['sha'].items():
  if digest(stage/relative(n))!=h:result['reason']='更新源文件已变化';return result
 result.update(ready=True,reason='已编译并校验',mcu_build=release.get('mcu_build'));return result
operation=p['operation']
if operation=='read':
 disk={n:digest(root/n) for n in ('python/controller.py','python/main.py','.cache/sketch/sketch.ino.elf') if (root/n).exists()}
 try:running=api('runtime')
 except Exception:running=None
 try:
  s=api('status');state={k:s.get(k) for k in ('mode','enabled','feedback_age_ms','feedback_hz','inference_hz','missing_joint_ids')}
 except Exception:state={}
 job=None
 if p.get('job'):
  w=root/'.codex-stage/updates'/relative(p['job']);f=w/'install-result.json'
  if f.exists():
   r=json.loads(f.read_text());job={k:r.get(k) for k in ('phase','error','readiness_error','rollback_error','version','elf_sha256')}
 print(json.dumps({'connected':True,'running':running,'disk':disk,'state':state,'candidate':candidate(),'job':job}))
elif operation=='update':
 c=candidate()
 if not c['ready']:raise RuntimeError(c['reason'])
 s=api('status')
 if s.get('mode')!='off' or s.get('enabled') is not False or s.get('feedback_age_ms',999)>150:raise RuntimeError('Fresh OFF required')
 w=root/'.codex-stage/updates'/relative(p['job']);w.mkdir(parents=True,exist_ok=False)
 for n in release['files']:
  target=w/relative(n);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(stage/n,target)
 (w/'build').mkdir();shutil.copy2(stage/'build/sketch.ino.elf',w/'build/sketch.ino.elf')
 (w/'candidate.json').write_text(json.dumps({'sha':release['files']}));(w/'compile-result.json').write_text(json.dumps({'phase':'compiled','elf_sha256':release['elf_sha256']}))
 (w/'release.json').write_text(json.dumps(release));(w/'install.py').write_text(p['installer'])
 (w/'install-result.json').write_text(json.dumps({'phase':'queued'}))
 process=subprocess.Popen([sys.executable,str(w/'install.py')],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
 print(json.dumps({'phase':'queued','job':p['job'],'pid':process.pid}))
else:raise ValueError('Unknown firmware operation')
'''

ACTIVE={'queued','preflight','stopping_off','starting_disabled','rolling_back'}
class FirmwareManager:
    def __init__(self,root,execute_remote=execute):
        self.root=Path(root);self.execute=execute_remote;self.lock=threading.Lock()
        self.job_file=self.root/'qa/firmware-update-state.json';self.job=None
        if self.job_file.exists():self.job=json.loads(self.job_file.read_text()).get('job')
        self.value={'connected':False,'checked_at':None,'busy':False};self.reading=False
    def release(self):
        path=self.root/'firmware-release.json'
        return json.loads(path.read_text(encoding='utf8')) if path.exists() else None
    def refresh(self):
        with self.lock:
            if self.reading:return self.snapshot_locked()
            self.reading=True
            read_job=self.job
        try:
            result=self.execute(self.root,REMOTE,{'operation':'read','release':self.release(),'job':read_job},timeout=15)
            result.update(checked_at=time.time(),busy=(result.get('job') or {}).get('phase') in ACTIVE,job_id=read_job)
            with self.lock:
                # A poll started before update() cannot clear the new job's lock.
                if self.job==read_job:
                    if read_job and result.get('job') is None:
                        result.update(busy=True,error='更新任务结果未确认，请重新读取；未重复安装')
                    self.value=result
        except Exception as exc:
            with self.lock:self.value=dict(self.value,connected=False,error=str(exc))
        finally:
            with self.lock:self.reading=False
        return self.snapshot()
    def snapshot_locked(self):return dict(self.value,reading=self.reading)
    def snapshot(self):
        with self.lock:return self.snapshot_locked()
    def update(self,version):
        release=self.release()
        if not release or version!=release['version']:raise ValueError('更新版本已变化，请重新读取')
        with self.lock:
            if self.value.get('busy'):raise ValueError('固件更新正在进行')
            if not self.value.get('connected') or time.time()-(self.value.get('checked_at') or 0)>15:raise ValueError('请先读取当前版本')
            state=self.value.get('state',{})
            if state.get('mode')!='off' or state.get('enabled') is not False:raise ValueError('请先停止并确认卸力')
            if not self.value.get('candidate',{}).get('ready'):raise ValueError('更新包尚未编译校验完成')
            self.value['busy']=True
            self.job=uuid.uuid4().hex
            self.job_file.parent.mkdir(exist_ok=True)
            self.job_file.write_text(json.dumps({'job':self.job,'version':version}),encoding='utf8')
        try:
            result=self.execute(self.root,REMOTE,{'operation':'update','release':release,'job':self.job,
                       'installer':(self.root/'firmware_install_remote.py').read_text(encoding='utf8')},timeout=20)
            with self.lock:self.value['job']=result
        except Exception as exc:
            # A lost SSH reply may still have started the board job. Keep the ID;
            # refresh that job instead of replaying installation.
            with self.lock:self.value['error']=str(exc)
            raise
        return self.snapshot()
