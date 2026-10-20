"""Explicit fixture/individual manufacturer calibration; no IO at construction."""
from __future__ import annotations
import copy
import json
from pathlib import Path
import threading
import time
import uuid

from bench_calibration_rpc import CLIENT, REMOTE
from bench_calibration_transport import CalibrationTransport

IDS = [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]


class Calibration:
    def __init__(self, root, maintenance, transport=None):
        self.root = Path(root)
        self.maintenance = maintenance
        self.transport = transport if transport is not None else CalibrationTransport(root)
        self.lock = threading.RLock()
        self.io_lock = threading.Lock()
        self.cancel = threading.Event()
        self.state = {'session_id':None, 'active':False, 'operation':None, 'current_id':None,
                      'completed':[], 'saved':{}, 'unknown':{}, 'held_mask':0, 'last_read':None,
                      'timing_ms':{}, 'message':'进入校准维护后，选择固定台全身校准或逐颗校准。'}
        self.worker = None
        if self.maintenance.rpc_ready():
            self._session()

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def _save(self):
        if self.maintenance.rpc_ready():
            self.maintenance.update_calibration_state(self.snapshot())

    def _session(self):
        m = self.maintenance.snapshot()
        if m['phase'] != 'maintenance':
            raise RuntimeError('请先进入厂家校准维护，不能向模型固件发送校准指令')
        if self.state['session_id'] != m['session_id']:
            # Recover this session's ledger, never reuse historical saved/unknown.
            path = self.maintenance.session_dir/'calibration-state.json'
            saved = json.loads(path.read_text('utf-8')) if path.exists() else {}
            self.state.update(session_id=m['session_id'], saved=saved.get('saved',{}),
                              unknown=saved.get('unknown',{}), held_mask=saved.get('held_mask',0),
                              completed=[], last_read=None)
        return m

    def _rpc(self, action, joint, token=None):
        m = self._session()
        with self.io_lock:
            started=time.monotonic()
            result = self.transport(self.root, REMOTE, {'id':joint, 'action':action,
                'token':token or uuid.uuid4().hex, 'image':m['image_id'],
                'reset_saved':str(joint) in self.state['saved'],
                'client':CLIENT, **self.maintenance.rpc_payload()}, timeout=55)
            result.setdefault('timing_ms',{}).update(getattr(self.transport,'last_timing',{}))
            result['timing_ms']['total_ms']=round((time.monotonic()-started)*1000,1)
            return result

    def request(self, action, joint=10, confirmed=False):
        if action not in ('read','release','stop','calibrate','all','enter','restore') or joint not in IDS:
            raise ValueError('无效校准操作或关节')
        if action in ('enter','restore','calibrate','all') and confirmed is not True:
            raise ValueError('请阅读页面动作预告后确认本次操作')
        with self.lock:
            if action == 'stop':
                self.cancel.set()  # immediate cancellation of all later joints
                if self.state['active']:
                    if self.state['operation'] in ('enter','restore'):
                        self.state['message']='正在切换固件；不会开始校准。切换完成后需核对状态。'
                    else:
                        self.state['message']='已取消后续轴，当前单轴事务结束后立即全部卸力。'
                    return self.snapshot()
            if self.state['active']:
                raise RuntimeError('操作进行中，请等待结果或停止后续校准')
            if action != 'enter':
                self._session()
            if action in ('calibrate','all') and self.state['unknown']:
                raise RuntimeError('存在未确认写入，先核对记录；不会自动重写')
            if action == 'all' and self.state['held_mask']:
                raise RuntimeError('全身校准前先点全部卸力，并在固定台上摆好全身中位')
            self.cancel.clear()
            self.state.update(active=True, operation=action, current_id=joint,
                              completed=[], message='请求已接收，等待实际读回。')
            self.worker = threading.Thread(target=self._run, args=(action,joint), daemon=True)
            self.worker.start()
            return self.snapshot()

    def _one(self, action, joint):
        token = uuid.uuid4().hex
        with self.lock:
            self.state['current_id'] = joint
            if action == 'calibrate':
                self.state['unknown'][str(joint)] = token
                self._save()  # claim before first possible hardware write
        result = self._rpc(action,joint,token)
        with self.lock:
            (self.maintenance.session_dir/('rpc-'+token+'.json')).write_text(
                json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            if not result.get('ok'):
                if action == 'calibrate' and result.get('unknown') is False:
                    self.state['unknown'].pop(str(joint),None)
                self._save()
                raise RuntimeError(result.get('error','操作结果未确认；不重试'))
            self.state['held_mask'] = result['held_mask']
            self.state['last_read'] = result
            self.state['timing_ms'] = result.get('timing_ms',{})
            if action == 'calibrate':
                self.state['saved'][str(joint)] = {'zero':result['saved_zero'],'token':token,'timing_ms':result.get('timing_ms',{})}
                self.state['unknown'].pop(str(joint),None)
                self.state['completed'].append(joint)
            self._save()
        return result

    def _run(self, action, joint):
        try:
            if action == 'enter':
                result = self.maintenance.enter()
                if result['phase'] != 'maintenance':
                    raise RuntimeError(result.get('error','维护切换未确认'))
                with self.lock:
                    self._session()
                    self.state['message']='厂家校准维护已就绪，当前全部卸力。摆好后再点击校准。'
            elif action == 'restore':
                result = self.maintenance.restore(self.snapshot())
                if result['phase'] != 'restored':
                    raise RuntimeError(result.get('error','正式模型恢复未确认'))
                if hasattr(self.transport,'close'): self.transport.close()
                with self.lock:
                    self.state['message']='已恢复正式模型程序并核对 OFF，不会自动开始动作。'
            elif action == 'all':
                # Read all 15 before any calibration. Never silently release held joints.
                with self.lock:
                    self.state['message']='正在一次读取全部15轴卸力、模式和锁状态；尚未写入校准。'
                result = self._one('preflight',joint)
                rows=result['rows']
                if [row['id'] for row in rows] != IDS:
                    raise RuntimeError('全身校准前15轴读回不完整')
                for row in rows:
                    if row['torque'] != 0 or row['mode'] != 4 or row['lock'] != 1:
                        raise RuntimeError(f'ID {row["id"]} 未卸力或寄存器状态异常，请先核对')
                for i in IDS:
                    if self.cancel.is_set(): break
                    with self.lock:
                        self.state['message']=f'全身校准 {len(self.state["completed"])+1}/15 · ID {i}'
                    self._one('calibrate',i)
                if not self.cancel.is_set():
                    with self.lock:
                        self.state['message']='15/15 已校准、保存并保持；结束时请先全部卸力，再恢复模型程序。'
            else:
                self._one(action,joint)
                with self.lock:
                    self.state['message']={'read':'当前读数已更新。','release':f'ID {joint} 已释放，其余保持轴不变。',
                        'stop':'全部15轴卸力读回已确认。','calibrate':f'ID {joint} 已校准、保存并保持。'}[action]
        except Exception as error:
            with self.lock:
                self.state['message']=str(error)+'；未自动重试。'
        finally:
            if self.cancel.is_set() and action not in ('enter','restore'):
                try:
                    self._one('stop',joint)
                    with self.lock:
                        self.state['message']='后续校准已取消，全部15轴卸力读回已确认。'+('存在未知写入，需核对记录。' if self.state['unknown'] else '')
                except Exception as error:
                    with self.lock:
                        self.state['message']='停止读回未确认：'+str(error)
            with self.lock:
                self.state['active']=False
                try:
                    if self.state['session_id']: self._save()
                except Exception as error:
                    self.state['message']+='；本机台账保存失败：'+str(error)
