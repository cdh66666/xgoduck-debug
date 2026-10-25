"""Workbench adapter for calibration in the normal official runtime firmware."""
from __future__ import annotations

import copy
import http.client
import json
import secrets
from pathlib import Path
import threading
import time

IDS = [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]


class FactoryMaintenance:
    """UI compatibility; entering/exiting calibration never uploads firmware."""
    def __init__(self, root):
        self.active = False
        self.expected_zeros = None
        self.error = ''

    def snapshot(self):
        return {'phase': 'maintenance' if self.active else 'official',
                'active': self.active, 'error': self.error, 'unknown': False,
                'integrated': True, 'firmware_switch': False}


class FactoryCalibration:
    def __init__(self, root, maintenance, request):
        self.path = Path(root)/'bench-results'/'factory-calibration.json'
        self.maintenance = maintenance
        self.call = request
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.state = dict(active=False, operation='', current_id=10,
                          saved={}, unknown={}, held_mask=0, completed=[],
                          last_read=None, last_batch=None, timing_ms={}, integrated=True,
                          message='直接校准，无需切换固件。',error='',run_id=None)
        if self.path.exists():
            previous = json.loads(self.path.read_text('utf-8'))
            self.state.update(saved=previous.get('saved', {}), unknown=previous.get('unknown', {}),last_batch=previous.get('last_batch'))
            # Stored receipts never establish the current torque/hold state.
        # Display provenance only: these are deliberately not restored from disk.
        self.prior_saved_count = len(self.state['saved'])
        self.current_saved_ids = set()
        self.current_batch_completed = None
        self.worker = None

    def close(self):
        self.cancel.set()

    def snapshot(self):
        with self.lock:
            value = copy.deepcopy(self.state)
            value.update(prior_saved_count=self.prior_saved_count,
                         current_saved_ids=[i for i in IDS if i in self.current_saved_ids],
                         current_batch_completed=copy.deepcopy(self.current_batch_completed))
            return value

    def _save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.state, ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(self.path)

    def request(self, action, joint=10, confirmed=False):
        if action not in ('enter','restore','read','release','stop','calibrate','all','recalibrate') or joint not in IDS:
            raise ValueError('无效校准操作或关节')
        if action in ('calibrate','all','recalibrate') and confirmed is not True:
            raise ValueError('请确认本次厂家中位校准')
        with self.lock:
            if action == 'stop':
                self.cancel.set()
                if self.state['active']:
                    self.state['message'] = '已取消后续轴，当前校准命令结束后全部卸力。'
                    return self.snapshot()
            if self.state['active']:
                raise RuntimeError('请等待当前操作结束')
            if action in ('calibrate','all','recalibrate') and self.state['unknown']:
                raise RuntimeError('存在未确认校准，请先核对回执')
            self.cancel.clear()
            self.state.update(active=True, operation=action, current_id=joint, completed=[],error='')
            if action=='recalibrate':
                self.prior_saved_count=len(self.state['saved'])
                self.current_saved_ids.clear()
                self.current_batch_completed=None
                self.state.update(run_id=secrets.token_hex(8),message='正在卸力并准备本次校准。')
            self.worker = threading.Thread(target=self._run, args=(action,joint), daemon=True)
            self.worker.start()
            return self.snapshot()

    def _command(self, action, joint):
        started = time.monotonic()
        result = self.call(action, joint)
        if result.get('ok') is not True:
            raise RuntimeError(result.get('error') or result.get('detail') or '主控操作未确认')
        with self.lock:
            self.state['timing_ms'] = {'total_ms': round((time.monotonic()-started)*1000,1)}
            selected=next((row for row in result.get('servos',[]) if row.get('id',row.get('target_id'))==joint),result)
            self.state['last_read'] = dict(selected, id=joint, observed_at=time.time())
            self.state['held_mask'] = result.get('held_mask',0)
            if action == 'enter':
                self.maintenance.active = True
            if action == 'exit':
                self.maintenance.active = False
            if result.get('zeros') is not None:
                self.maintenance.expected_zeros = list(result['zeros'])
        return result

    def _calibrate(self, joint):
        with self.lock:
            self.state['current_id'] = joint
            self.state['unknown'][str(joint)] = {'started_at': time.time()}
            self._save()
        result = self._command('calibrate',joint)
        if result.get('host_zero_saved') is not True:
            raise RuntimeError('厂家校准后软件零位保存未确认')
        zero = result.get('zero_pos',result.get('position'))
        if type(zero) is not int or not 0 <= zero <= 4095:
            raise RuntimeError('校准零位读回无效')
        with self.lock:
            self.state['saved'][str(joint)] = {'zero': zero, 'timing_ms':dict(self.state['timing_ms'])}
            self.state['unknown'].pop(str(joint),None)
            self.state['completed'].append(joint)
            self._save()
            self.current_saved_ids.add(joint)

    def _run(self, action, joint):
        started = time.monotonic()
        try:
            if action == 'recalibrate':
                off=self._command('off',joint)
                if off.get('off_mask') != (1<<len(IDS))-1:
                    raise RuntimeError('15轴卸力读回未齐，本次未开始写校准')
                if self.cancel.is_set():return
                self._command('enter',joint)
            if action in ('all','recalibrate'):
                rows = self._command('read_all',joint)['servos']
                if [row.get('id',row.get('target_id')) for row in rows] != IDS:
                    raise RuntimeError('15轴校准前读回不完整')
                for i,row in zip(IDS,rows):
                    if row.get('torque') != 0 or row.get('locked') not in (1,True) or row.get('mode') != 4:
                        raise RuntimeError(f'ID {i} 请先卸力并确认厂家模式和锁')
                for i in IDS:
                    if self.cancel.is_set(): break
                    self._calibrate(i)
                if self.cancel.is_set():
                    return
            elif action == 'calibrate':
                self._calibrate(joint)
            else:
                self._command({'restore':'exit','stop':'off'}.get(action,action),joint)
            with self.lock:
                self.state['message'] = {'all':'15/15 已校准并保存，扭矩状态见实时读回。','recalibrate':'校准成功，15颗新零位已保存。',
                    'calibrate':f'ID {joint} 已校准并保存，扭矩状态见实时读回。', 'enter':'校准模式已就绪；固件未切换。',
                    'restore':'已退出校准，保持 OFF。', 'stop':'全部卸力已确认。',
                    'release':f'ID {joint} 已释放。', 'read':'实际寄存器读回已更新。'}[action]
        except Exception as error:
            with self.lock:
                self.state['message'] = str(error)
                self.state['error'] = str(error)
        finally:
            if self.cancel.is_set() and action not in ('enter','restore'):
                try:
                    self._command('off',joint)
                    with self.lock: self.state['message'] = '后续轴已取消，全部卸力已确认。'
                except Exception as error:
                    with self.lock: self.state['message'] = '卸力回执未确认：'+str(error)
            with self.lock:
                self.state['active'] = False
                self.state['batch_elapsed_ms'] = round((time.monotonic()-started)*1000,1)
                if action in ('all','recalibrate'):
                    self.state['last_batch']={'completed':list(self.state['completed']),
                        'elapsed_ms':self.state['batch_elapsed_ms'],'finished_at':time.time()}
                    self.current_batch_completed = list(self.state['completed'])
                self._save()


def board_request(tunnel, key):
    def request(action, joint):
        if tunnel.port is None:
            raise ConnectionError('主控无线未连接')
        connection = http.client.HTTPConnection('127.0.0.1',tunnel.port,timeout=4)
        try:
            connection.request('POST','/api/factory',json.dumps({'action':action,'target_id':joint}),
                               {'Content-Type':'application/json','X-Cal-Key':key()})
            response = connection.getresponse()
            value = json.loads(response.read(256*1024))
            if response.status != 200:
                raise RuntimeError(value.get('detail','厂家校准请求未确认'))
            return value
        finally:
            connection.close()
    return request
