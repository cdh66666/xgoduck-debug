"""Small host adapter for the official runtime's manufacturer calibration command."""
import json
import secrets
import threading
import time
from pathlib import Path
from wire import encode_factory, decode_factory_reply, FACTORY_OPS


def atomic_json(path, value):
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    temp.replace(path)


class FactoryCalibrationMixin:
    def _factory_init(self, zero_path):
        self.factory_zero_path=Path(zero_path)
        self.factory_dir=self.factory_zero_path.parent/'factory_calibration'
        self.factory_expected=None
        self.factory_reply=None
        self.factory_event=threading.Event()
        self.factory_active=False
        self.factory_caps=0
        self.factory_held=0
        self.factory_reads={}
        self.factory_epoch=secrets.token_hex(8)
        self.factory_pending=None
        pending=self.factory_dir/'pending.json'
        if pending.exists():
            self.factory_pending=json.loads(pending.read_text('utf-8'))

    def _factory_reboot(self):
        self.factory_active=False
        self.factory_caps=0
        self.factory_held=0
        self.factory_reads={}
        self.factory_epoch=secrets.token_hex(8)

    def receive_factory_reply(self, raw):
        try: reply=decode_factory_reply(raw)
        except ValueError: return
        with self.lock:
            if self.factory_expected!=(reply['op'],reply['target_id'],reply['request']): return
            self.factory_reply=reply
            self.factory_event.set()

    def _factory_request(self, action, target_id):
        request=secrets.randbits(32)
        op=FACTORY_OPS[action]
        with self.lock:
            self.web_time=time.monotonic()
            self.factory_expected=(op,target_id,request)
            self.factory_reply=None
            self.factory_event.clear()
        try:
            self.bridge.notify('duck_factory',encode_factory(op,target_id,request))
            if not self.factory_event.wait(2): raise ValueError('factory reply timeout; calibration is never retried automatically')
            with self.lock:
                result=dict(self.factory_reply)
                self.factory_caps=result['capabilities']
                self.factory_held=result['held_mask']
                if action in ('read','calibrate') and result['ok']:
                    record=dict(result,read_at=time.time(),read_monotonic=time.monotonic(),boot_epoch=self.factory_epoch)
                    self.factory_reads[str(target_id)]=record
            return result
        finally:
            with self.lock: self.factory_expected=None

    def factory_action(self, action, target_id=10):
        target_id=int(target_id)
        if not 0<=target_id<=253: raise ValueError('invalid target id')
        with self.servo_op_lock:
            if action=='capabilities': return self._factory_request(action,target_id)
            if action=='enter':
                if self.factory_active and self.mode=='servo_debug':
                    return dict(self._factory_request('capabilities',target_id),active=True)
                if self.mode in ('hold','policy'): raise ValueError('disarm before calibration')
                self.set_mode('servo_debug')
                # SERVO_ENTER has the same trailing unicast ACK as SERVO_EXIT.
                self._factory_request('capabilities',target_id)
                result=self._factory_request('off',target_id)
                if not result['ok']: raise ValueError('torque-off readback failed')
                self.factory_active=True
                return dict(result,active=True)
            if action in ('off','exit'):
                # OFF also works outside a calibration session and never resumes a model.
                self.set_mode('off')
                # SERVO_EXIT ends in a unicast gain write. A control-task RPC
                # barrier lets that cycle drain its ACK before OFF reads torque.
                self._factory_request('capabilities',target_id)
                result=self._factory_request('off',target_id)
                self.factory_active=False
                self.factory_reads={}
                return result
            if self.mode!='servo_debug' or not self.factory_active:
                raise ValueError('enter factory calibration first')
            if action=='read_all':
                from rl_core import MOTOR_IDS
                replies=[self._factory_request('read',int(i)) for i in MOTOR_IDS]
                return {'ok':all(r['ok'] for r in replies),'servos':replies,'boot_epoch':self.factory_epoch}
            if action not in ('read','release','calibrate'): raise ValueError('unknown factory action')
            if action!='calibrate': return self._factory_request(action,target_id)
            if self.factory_pending: raise ValueError('previous calibration outcome is unknown; inspect pending transaction before any further EEPROM write')
            from rl_core import MOTOR_IDS
            if target_id not in MOTOR_IDS: raise ValueError('calibration target is not a commissioned axis')
            before=self._factory_request('read',target_id)
            if not before['ok']: raise ValueError('could not read factory registers before calibration')
            transaction={'target_id':target_id,'before':before,'zero_before':list(self.zero_pos),'started_at':time.time()}
            pending=self.factory_dir/'pending.json'
            atomic_json(pending,transaction)
            self.factory_pending=transaction
            result=self._factory_request('calibrate',target_id)
            if result['zero_changed']:
                with self.lock:
                    self.zero_pos[list(MOTOR_IDS).index(target_id)]=result['zero_pos']
                    self._save_zeros()
                result['host_zero_saved']=True
            transaction['result']=result
            atomic_json(self.factory_dir/('axis-%s-%s.json'%(target_id,time.time_ns())),transaction)
            if result['ok'] or result['zero_changed']:
                pending.unlink()
                self.factory_pending=None
            return result

    def factory_status(self):
        now=time.monotonic()
        reads={key:dict(value,age_ms=(now-value['read_monotonic'])*1000) for key,value in self.factory_reads.items()}
        last=max((x['read_at'] for x in reads.values()),default=None)
        return {'supported':True,'mcu_supported':self.factory_caps==31,'capabilities':self.factory_caps,
                'active':self.mode=='servo_debug' and self.factory_active,'held_mask':self.factory_held,
                'held_mask_is_live':False,'last_read':last,'boot_epoch':self.factory_epoch,
                'registers':reads,'pending':self.factory_pending,'protocol':'DQF1/DQF2',
                'firmware_switch_required':False}
