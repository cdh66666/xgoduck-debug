"""Read-only MCU display telemetry. No policy, servo or calibration dependencies."""
import math
import struct
import threading
import time

DISPLAY_V1=struct.Struct('<4sBBBB6I12f8I')
DISPLAY=struct.Struct('<4sBBBB6I12f8I3fI')
assert DISPLAY.size==128


class DisplayTelemetryCache:
    def __init__(self,clock=time.monotonic):
        self.clock=clock
        self.lock=threading.Lock()
        self.value=None
        self.received_at=None
        self.previous=None
        self.previous_received_at=None
        self.retired=set()

    def receive(self,raw):
        if not isinstance(raw,(bytes,bytearray)) or len(raw) not in (DISPLAY.size,DISPLAY_V1.size):return False
        extended=len(raw)==DISPLAY.size
        x=(DISPLAY if extended else DISPLAY_V1).unpack(raw)
        if x[0]!=b'DQD1' or x[1]!=(2 if extended else 1) or (not extended and x[2]&~31) or x[3]!=10:return False
        if extended and ((x[2]&224) not in (32,64,128) or not all(math.isfinite(v) for v in x[31:34])):return False
        if not all(math.isfinite(v) for v in x[11:23]):return False
        if abs(math.hypot(*x[11:15])-1)>.01 or abs(math.hypot(*x[15:19])-1)>.01:return False
        boot='%08x%08x'%(x[6],x[5])
        value=dict(flags=x[2],servo_id=x[3],raw=x[4],boot_epoch=boot,source_seq=x[7],
                   sample_us=x[8],source_imu_us=x[9],segment=x[10],quaternion_xyzw=list(x[11:15]),
                   relative_quaternion_xyzw=list(x[15:19]),gravity=list(x[19:22]),deg=x[22],
                   voltage_sample_us=x[23],read_attempts=x[24],read_successes=x[25],voltage_skipped=x[26],
                   compute_us=x[27],max_compute_us=x[28],voltage_request_us=x[29],max_voltage_request_us=x[30])
        value.update(calibration=('ready' if x[2]&32 else 'calibrating' if x[2]&64 else 'failed') if extended else 'unsupported',
                     bias_gyro_deg_s=list(x[31:34]) if extended else None,bias_samples=x[34] if extended else 0)
        with self.lock:
            if boot in self.retired:return False
            previous=self.value
            if previous and boot==previous['boot_epoch']:
                step=(value['source_seq']-previous['source_seq'])&0xffffffff
                if not 0<step<0x80000000:return False
            elif previous:
                self.retired.add(previous['boot_epoch'])
                if len(self.retired)>64:self.retired.pop()
            self.previous=previous if previous and boot==previous['boot_epoch'] else None
            self.previous_received_at=self.received_at if self.previous else None
            self.value=value;self.received_at=self.clock()
        return True

    def snapshot(self,feedback,rx_time,now=None):
        now=self.clock() if now is None else now
        with self.lock:
            value=dict(self.value) if self.value else None
            previous=dict(self.previous) if self.previous else None
        unavailable={'available':False,'deg':None,'quality':'unavailable','relative':True,
                     'reference':'mcu-power-on-full-pose','drift_expected':True,
                     'calibration':'waiting','bias_samples':0,'bias_gyro_deg_s':None}
        if not value:
            return dict(display_heading=unavailable,
                        servo_power=dict(available=False,voltage_v=None,raw=None,servo_id=10,age_ms=None,cached=True),
                        display_telemetry_perf={})
        def age(source_us):
            if feedback is None or not math.isfinite(rx_time) or now<rx_time:return None
            delta=(feedback.sample_us+feedback.work_us-source_us)&0xffffffff
            return delta/1000+(now-rx_time)*1000 if delta<0x80000000 else None
        heading_age=age(value['source_imu_us'])
        # Bridge callbacks may deliver display N before state N. Use the prior
        # real endpoint until state catches up; never invent an age for N.
        if heading_age is None and previous and previous['boot_epoch']==value['boot_epoch']:
            prior_age=age(previous['source_imu_us'])
            if prior_age is not None and prior_age<150:
                value=previous;heading_age=prior_age
        flags=value['flags'];has_heading=bool(flags&6)
        calibrated=value['calibration'] in ('ready','unsupported')
        current=bool(flags&1 and has_heading and calibrated and heading_age is not None and heading_age<150)
        quality='held' if flags&4 else 'current' if flags&2 else 'unavailable'
        heading=dict(unavailable,available=current,deg=value['deg'] if current else None,
                     quality=quality if current else 'unavailable',boot_epoch=value['boot_epoch'],
                     source_seq=value['source_seq'],source_imu_us=value['source_imu_us'],
                     segment=value['segment'],continuous=bool(flags&8),age_ms=heading_age,
                     calibration=value['calibration'],bias_samples=value['bias_samples'],
                     bias_gyro_deg_s=value['bias_gyro_deg_s'])
        voltage_age=age(value['voltage_sample_us']) if flags&16 else None
        power=dict(available=bool(flags&16 and voltage_age is not None and voltage_age<6000),
                   voltage_v=value['raw']/10 if flags&16 else None,raw=value['raw'] if flags&16 else None,
                   servo_id=10,age_ms=voltage_age,cached=True,read_attempts=value['read_attempts'],
                   read_successes=value['read_successes'])
        perf={key:value[key] for key in ('compute_us','max_compute_us','voltage_request_us',
                                        'max_voltage_request_us','voltage_skipped')}
        return dict(display_heading=heading,servo_power=power,display_telemetry_perf=perf)
