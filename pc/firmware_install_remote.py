"""Install one prepared receiver build while inhibited and verified unloaded."""
import hashlib,json,pathlib,shutil,subprocess,time,urllib.request
ROOT=pathlib.Path('/home/arduino/ArduinoApps/xgoduck-commissioning')
WORK=pathlib.Path(__file__).resolve().parent
import fcntl
update_lock=(ROOT/'data/firmware-update.lock').open('a')
fcntl.flock(update_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
release=json.loads((WORK/'release.json').read_text())
cfg=json.loads((WORK/'candidate.json').read_text())
report={'phase':'preflight','version':release['version'],'motion_requests':0,'calibration_requests':0}
save_lock=ROOT/'data/motion-inhibit'
had_lock=save_lock.exists()
def save():(WORK/'install-result.json').write_text(json.dumps(report,indent=2))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
key=(ROOT/'data/calibration_key').read_text().strip()
def api(path='status',body=None):
    req=urllib.request.Request('http://127.0.0.1:9527/api/'+path,data=None if body is None else json.dumps(body).encode(),headers={} if body is None else {'Content-Type':'application/json','X-Cal-Key':key})
    with urllib.request.urlopen(req,timeout=5) as f:return json.load(f)
log=(WORK/'install.log').open('w',buffering=1)
def run(args,timeout=120):
    log.write(repr(args)+'\n');log.flush()
    subprocess.run(args,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=timeout)
stopped=False;before={};added=[]
try:
    built=json.loads((WORK/'compile-result.json').read_text());assert built['phase']=='compiled'
    assert sha(WORK/'build/sketch.ino.elf')==built['elf_sha256']
    s=api();assert s['mode']=='off' and s['enabled'] is False and s['feedback_age_ms']<150
    report['before_status']=s
    if not had_lock:save_lock.write_text('firmware update: motors remain off')
    off=api('factory',{'action':'off','target_id':10});assert off['ok'] and off['off_mask']==32767,off
    report['before_off']=off
    info=json.loads(subprocess.check_output(['docker','inspect','xgoduck-commissioning-main-1'],text=True))[0]
    assert any(m['Source']==str(ROOT) and m['Destination']=='/app' for m in info['Mounts'])
    protected={str(p.relative_to(ROOT)):sha(p) for p in [ROOT/'python/controller.py',ROOT/'python/policy.py',ROOT/'python/rl_core.py',ROOT/'python/recovery.py',ROOT/'data/motion-inhibit',ROOT/'data/native-calibration/zero_pos.json']+list((ROOT/'python').glob('*.onnx'))}
    for n,h in cfg['sha'].items():
        assert sha(WORK/n)==h,n
        if (ROOT/n).exists():
            p=WORK/'before'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/n,p);before[n]=sha(p)
        else:added.append(n)
    shutil.copy2(ROOT/'.cache/sketch/sketch.ino.elf',WORK/'before.elf')
    report.update(phase='stopping_off',before=before,added=added,protected=protected);save()
    run(['arduino-app-cli','app','stop',str(ROOT)]);stopped=True
    for n in cfg['sha']:
        p=ROOT/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(WORK/n,p)
    shutil.copy2(WORK/'build/sketch.ino.elf',ROOT/'.cache/sketch/sketch.ino.elf')
    descriptor={'version':release['version'],'mcu_build':release['mcu_build'],'elf_sha256':release['elf_sha256'],'python_sha256':{n:h for n,h in release['files'].items() if n.startswith('python/')}}
    descriptor['python_sha256'].setdefault('python/controller.py',protected['python/controller.py'])
    old_descriptor=ROOT/'data/firmware-release.json'
    if old_descriptor.exists():shutil.copy2(old_descriptor,WORK/'before-release.json')
    old_descriptor.write_text(json.dumps(descriptor))
    report['phase']='starting_disabled';save()
    run(['arduino-app-cli','app','start',str(ROOT)],300)
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        try:
            s=api()
            report['last_status']={k:s.get(k) for k in ('mode','enabled','received','feedback_hz','feedback_age_ms','servo_bus','last_error')};save()
            if s.get('enabled') is False and s['feedback_age_ms']<150:
                api('mode',{'mode':'off'})
                if s.get('servo_bus',{}).get('age_s') is not None and s['servo_bus']['age_s']<3:break
        except Exception as e:report['readiness_error']=repr(e);save()
        time.sleep(.3)
    else:raise RuntimeError('Fresh disabled receiver diagnostics not ready')
    off=api('factory',{'action':'off','target_id':10});assert off['ok'] and off['off_mask']==32767,off
    for n,h in protected.items():assert sha(ROOT/n)==cfg['sha'].get(n,h),n
    s=api();assert s['mode']=='off' and s['enabled'] is False
    identity=api('runtime');assert identity['running_verified'] and identity['release']==release['version'],identity
    report['identity']=identity
    report.update(phase='installed_off_verified',after_status=s,after_off=off,elf_sha256=sha(ROOT/'.cache/sketch/sketch.ino.elf'))
except Exception as e:
    report.update(error=repr(e),failed_phase=report['phase']);save()
    if stopped:
        report['phase']='rolling_back';save()
        try:
            try:api('mode',{'mode':'off'})
            except Exception:pass
            run(['arduino-app-cli','app','stop',str(ROOT)])
            descriptor=ROOT/'data/firmware-release.json'
            if (WORK/'before-release.json').exists():shutil.copy2(WORK/'before-release.json',descriptor)
            else:descriptor.unlink(missing_ok=True)
            for n in before:shutil.copy2(WORK/'before'/n,ROOT/n)
            for n in added:(ROOT/n).unlink(missing_ok=True)
            shutil.copy2(WORK/'before.elf',ROOT/'.cache/sketch/sketch.ino.elf')
            run(['arduino-app-cli','app','start',str(ROOT)],300)
            for _ in range(100):
                try:
                    api('mode',{'mode':'off'});s=api()
                    if s.get('enabled') is False:break
                except Exception:pass
                time.sleep(.3)
            else:raise RuntimeError('Rollback OFF unconfirmed')
            report['phase']='rolled_back_off'
        except Exception as re:report.update(phase='rollback_failed',rollback_error=repr(re))
    else:report['phase']='preflight_failed'
finally:
    if not had_lock and report['phase'] in ('installed_off_verified','rolled_back_off','preflight_failed'):save_lock.unlink(missing_ok=True)
    save();log.close()
