"""Real SignalFilter/ImuFilter with current Controller method AST and no devices."""
from pathlib import Path
import importlib.util,json,math,sys,hashlib
import numpy as np
import test_zero_sync as fixture
OUT=Path(__file__).resolve().parent;ROOT=fixture.ROOT;clock=fixture.clock;w=fixture.w
p=fixture.active_runtime.runtime_source(ROOT,'python/proc.py');spec=importlib.util.spec_from_file_location('source_clock_real_proc',p);proc=importlib.util.module_from_spec(spec);spec.loader.exec_module(proc)
def controller(path):
    clock.now=100.;cls,env=fixture.load(path);env['SignalFilter']=proc.SignalFilter
    b=fixture.Bridge();c=cls(b);return c,b
def packet(seq,sample_us,i=0,acc=None):
    phi=.55*math.sin(i*.04)
    acc=acc if acc is not None else [0,9.8*math.sin(phi),-9.8*math.cos(phi)]
    gyro=[-.55*.04/.01*math.cos(i*.04)*180/math.pi,0,0]
    return w.STATE.pack(b'DQS1',1,1,32767,*[seq,sample_us&0xffffffff,(sample_us+20)&0xffffffff,seq,0,10000,7500,0,0,0],*[round(v*16384/9.8) for v in acc],*[round(v*32) for v in gyro],*([i*.01]*15),*([10*math.sin(i*.1)]*15),*([0.]*15))
def replay(path,arrival):
    c,b=controller(path);out=[]
    for i in range(500):
        clock.now=arrival(i);c.receive(packet(i+1,1000000+i*10000,i));out.append([*c.filtered['grav'],*c.filtered['gyro_rad'],*c.filtered['vels_deg']])
    return np.array(out),c
uniform=lambda i:100.+i*.01
jitter=lambda i:100.+(i//10)*.1+(i%10)*.0001
baseline=ROOT.parent/'robot-app/python/controller.py';single=OUT/'clock-candidate/python/controller.py';combined=OUT/'combined-candidate/python/controller.py'
old_uniform,_=replay(baseline,uniform);old_jitter,_=replay(baseline,jitter)
results=[]
for path in [single,combined]:
    tests=[];a,c=replay(path,uniform);b,j=replay(path,jitter)
    normal=float(np.max(np.abs(a-old_uniform)));jitter_error=float(np.max(np.abs(a-b)))
    assert normal<1e-8 and jitter_error<1e-12
    assert c.received==j.received==500 and c.feedback.seq==500
    tests.append('500 uniform 10ms frames agree with baseline; same MCU input invariant under Host batch jitter')
    c,b=controller(path);clock.now=100.;c.receive(packet(1,1000000));rx=c.rx_time;received=c.received;times=len(c.rx_times);intervals=len(c.rx_intervals)
    for seq,sample in [(1,1010000),(2,1000000),(3,999999)]:clock.now+=.1;c.receive(packet(seq,sample,seq))
    assert c.rx_time==rx and c.received==received and len(c.rx_times)==times and len(c.rx_intervals)==intervals and c.feedback.seq==1
    clock.now=101.;c.receive(packet(4,1010000,4));assert c.received==2 and c.feedback.seq==4 and c.rx_time==101. and c.gaps==2
    tests.append('duplicate sequence, duplicate/backward sample_us do not refresh feedback or sample histories')
    c,b=controller(path);clock.now=100.;c.receive(packet(1,0xfffff000));clock.now+=.01;c.receive(packet(2,(0xfffff000+10000)&0xffffffff,1));assert abs(c._feedback_clock_s-.01)<1e-12 and c.received==2
    tests.append('u32 sample_us wrap unwraps continuously without reset or missing-frame interpolation')
    c,b=controller(path);clock.now=100.;c.receive(packet(1,1000000));clock.now+=.001;c.receive(packet(2,1250000,2,acc=[9.8,0,0]));assert c.received==2 and np.allclose(c.filtered['grav'],[1,0,0],atol=1e-12)
    assert abs(c.filter._last_t-.25)<1e-12
    tests.append('250ms source gap triggers unchanged ImuFilter reset even if Host delivery interval is 1ms; exactly two samples')
    c,b=controller(path);clock.now=100.;c.receive(packet(1000,20000000));clock.now+=.01;c.receive(packet(1,10000,1,acc=[0,9.8,0]));assert c.factory_resets==1 and c._feedback_clock_us==10000 and c._feedback_clock_s==0 and np.allclose(c.filtered['grav'],[0,1,0],atol=1e-12)
    tests.append('existing MCU sequence-restart path clears unwrap and filter together')
    clock.now+=.2
    try:c._require_feedback(0)
    except ValueError as e:assert 'stale' in str(e)
    else:raise AssertionError('Host freshness was replaced by source clock')
    assert c.status()['feedback_timing']['freshness_clock']=='host_monotonic'
    tests.append('Host monotonic still determines stale feedback and diagnostic timing clock')
    results.append(dict(candidate=str(path),passed=True,tests=tests,uniform_baseline_max_abs_error=normal,jitter_max_abs_error=jitter_error))
old_angle=np.degrees(np.arccos(np.clip(np.sum(old_uniform[:,:3]*old_jitter[:,:3],axis=1),-1,1)))
v=dict(ok=True,device_requests=0,formula_unchanged=True,real_filter_source=str(p),real_filter_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),old_host_jitter_max_abs_difference=float(np.max(np.abs(old_uniform-old_jitter))),old_host_jitter_attitude_error_deg=dict(median=float(np.median(old_angle)),p95=float(np.percentile(old_angle,95)),max=float(np.max(old_angle))),candidates=results,limits=['Existing seq-regression reboot classification is retained; DQS1 alone cannot distinguish every replayed old-epoch packet from a genuine reboot.','This validates preprocessing timing, not robot or policy stability.'])
(fixture.FIXTURE/'clock-verification.json').write_text(json.dumps(v,indent=2),encoding='utf-8');print(json.dumps(v,indent=2))
