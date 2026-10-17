from collections import deque
from datetime import datetime, timezone
import hmac
import secrets
import time
import calibration_wire as calibration
import sequence_wire as sequence
from sequence_store import make_groups


class SequenceController:
    def __init__(self, owner):
        self.owner = owner
        self.packet = self.observed_at = self.current = self.last_heartbeat = None
        self.pending = None
        self.completed = set()
        self.unknown = False
        self.faults = deque(maxlen=8)
        self.parameters = {}
        self.parameter_requests = {}

    @staticmethod
    def error(code, message, status=409, ids=()):
        from calibration_host import CalibrationError
        return CalibrationError(code, message, status, ids)

    def reset(self):
        self.packet = self.observed_at = self.current = self.last_heartbeat = self.pending = None
        self.unknown = False
        self.completed.clear()
        self.parameters.clear()
        self.parameter_requests.clear()

    def fresh(self):
        owner = self.owner
        return bool(self.packet and self.observed_at is not None and owner.fresh()
                    and self.packet['boot_id'] == owner.snapshot['boot_id']
                    and 0 <= owner.clock() - self.observed_at < .150)

    def busy(self):
        return bool(self.current or self.unknown or self.packet and self.packet['phase'] in
                    ('preparing', 'selected', 'capturing', 'enabling', 'holding', 'releasing'))

    def mask_verified(self, packet=None):
        packet = packet or self.packet
        if not packet or packet['fresh_mask'] != calibration.ALL_MASK or packet['metadata_mask'] != calibration.ALL_MASK:
            return False
        held = packet['held_mask']
        if packet['enabled_mask'] != held or packet['off_mask'] != calibration.ALL_MASK ^ held:
            return False
        return all(packet['modes'][index] in (0, 4) and packet['torques'][index] == int(bool(held & (1 << index)))
                   and (not held & (1 << index) or abs(packet['positions'][index] - packet['goals'][index]) <= 57)
                   for index in range(15))

    def receive(self, payload):
        owner = self.owner
        try:
            packet = sequence.decode(payload)
        except (ValueError, TypeError):
            with owner.condition:
                owner.errors.append({'code': 'invalid_sequence_snapshot'})
                if self.busy():
                    self.stop_send('invalid_sequence_snapshot')
            return
        with owner.condition:
            if not owner.snapshot or packet['boot_id'] != owner.snapshot['boot_id']:
                return
            previous = self.packet
            if previous and (not 0 < calibration.elapsed(packet['sequence'], previous['sequence']) < 0x80000000
                             or calibration.elapsed(packet['sample_us'], previous['sample_us']) >= 0x80000000):
                return
            self.packet, self.observed_at = packet, owner.clock()
            current = self.current
            if packet['error'] and (not self.faults or self.faults[-1]['capture_seq'] != packet['capture_seq']):
                self.faults.append({key: packet[key] for key in ('boot_id', 'capture_seq', 'error', 'selected_id', 'stage', 'goals', 'positions', 'fresh_mask')})
            if current and packet['error'] and packet['operation_ack_seq'] in current.get('operation_requests', ()):
                current.setdefault('failure', dict(packet))
            if current and current.get('confirmed') and (packet['session'] != current['nonce'] or packet['phase'] in ('off', 'done', 'fault', 'releasing')):
                self.current = None
                self.pending = None
            if packet['phase'] in ('done', 'fault', 'off') and packet['off_mask'] == calibration.ALL_MASK and not packet['session']:
                self.unknown = False
                if isinstance(owner.operation, str) and owner.operation.startswith('sequence_'):
                    owner.operation = None
            elif packet['phase'] in ('preparing', 'selected', 'capturing', 'enabling', 'holding', 'releasing'):
                self.unknown = True
                owner.operation = 'sequence_' + packet['phase']
            owner.condition.notify_all()

    def send(self, operation, target=0, current=None, selected=None):
        owner = self.owner
        if not owner.snapshot:
            raise self.error('mcu_unknown', '主控反馈尚未就绪。', 503)
        owner.request_seq = (owner.request_seq + 1) & 0xffffffff or 1
        request = owner.request_seq
        payload = sequence.encode(operation, target, owner.snapshot['boot_id'], request,
                                  current['nonce'] if current else 0, owner.snapshot['sample_us'],
                                  selected['selection_seq'] if selected else 0,
                                  selected['positions'][calibration.MOTOR_IDS.index(target)] if selected else 0,
                                  selected['modes'][calibration.MOTOR_IDS.index(target)] if selected else 0)
        owner.requests[request] = (100 + operation, target, current['nonce'] if current else 0, 0)
        owner.sender.put(request, calibration.OFF_ALL if operation == sequence.STOP else 100 + operation, payload, owner.generation)
        owner.sent += 1
        if current is not None:
            current['last_sent'] = request
            if operation in (sequence.BEGIN, sequence.SELECT, sequence.HOLD):
                current.setdefault('operation_requests', set()).add(request)
        return request

    def stop_send(self, reason='user_stop'):
        current, self.current = self.current, None
        self.pending = None
        self.unknown = True
        self.owner.operation = 'sequence_releasing'
        if current is not None:
            current['stopped_reason'] = reason
        try:
            return self.send(sequence.STOP)
        except Exception:
            self.owner.errors.append({'code': 'sequence_release_unconfirmed'})
            return None

    def session(self, body):
        current = self.current
        if not current or not isinstance(body.get('session'), str) or not hmac.compare_digest(body['session'], current['token']):
            raise self.error('sequence_session_mismatch', '逐颗校准会话已结束，请先确认卸力。')
        if self.lease_expired(current) or current['generation'] != self.owner.generation:
            self.stop_send('client_lease')
            raise self.error('sequence_session_expired', '逐颗校准连接超时，已请求全部卸力。')
        return current

    def lease_expired(self, current):
        return (self.owner.clock() - current['client_at'] > .400 if current.get('output_started')
                else self.owner.clock() >= current['prepare_deadline'])

    def wait(self, request, current, phase, timeout=3.0):
        owner = self.owner
        deadline = time.monotonic() + timeout
        with owner.condition:
            while True:
                if request in owner.failed_requests:
                    owner.raise_delivery_error(request)
                packet = self.packet
                failure = current.get('failure') if current else None
                if failure:
                    raise self.error('sequence_mcu_rejected', '逐颗校准被主控停止，错误代码' + str(failure['error']) + '。', ids=[failure['selected_id']])
                if current and self.current is not current:
                    raise self.error('sequence_cancelled', '逐颗校准已取消；本颗未登记完成。')
                if (self.fresh() and packet['operation_ack_seq'] == request and packet['phase'] in phase
                        and packet['result'] == calibration.OK and not packet['error']):
                    if current is None or packet['session'] == current['nonce'] and packet['boot_id'] == current['boot']:
                        return dict(packet)
                if current is None and self.fresh() and packet['phase'] in ('done', 'fault', 'off') and packet['off_mask'] == calibration.ALL_MASK and not packet['session'] and packet['operation_ack_seq'] == request:
                    return dict(packet)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise self.error('sequence_timeout', '逐颗校准未收到完整回执，不自动重试。', 503)
                owner.condition.wait(min(.025, remaining))

    def status(self):
        packet = self.packet
        fresh = self.fresh()
        verified = fresh and self.mask_verified() and packet['phase'] in ('selected', 'holding') and not packet['error']
        held_ids = [sid for index, sid in enumerate(calibration.MOTOR_IDS) if verified and packet['held_mask'] & (1 << index)]
        return {'supported': bool(packet), 'contract_revision': sequence.VERSION, 'fresh': fresh,
                'tolerance_ticks': 57, 'arrival_tolerance_deg': 5,
                'active': self.busy(), 'phase': packet['phase'] if packet else 'off',
                'selected_id': packet['selected_id'] if packet else 0,
                'held_ids': held_ids, 'completed_ids': sorted(self.completed),
                'holding_verified': bool(held_ids), 'all_holding_verified': len(held_ids) == 15 and len(self.completed) == 15,
                'release_verified': bool(fresh and packet['off_mask'] == calibration.ALL_MASK and not packet['session']
                                         and packet['phase'] in ('off', 'done', 'fault')),
                'capture_pending': self.pending is not None,
                'lease_stage': 'motion' if self.current and self.current.get('output_started') else 'prepare' if self.current else None,
                'error': packet['error'] if packet else None,
                'feedback': {key: packet[key] for key in ('sequence', 'sample_us', 'selection_seq', 'capture_seq', 'held_mask', 'off_mask', 'fresh_mask', 'goals', 'positions', 'modes', 'torques')} if packet else None,
                'fault_history': list(self.faults), 'physical_acceptance_verified': False}

    def action(self, body):
        owner = self.owner
        operation = body.get('op')
        operations = {'sequence_begin': sequence.BEGIN, 'sequence_select': sequence.SELECT,
                      'sequence_capture_hold': sequence.HOLD, 'sequence_heartbeat': sequence.HEARTBEAT, 'sequence_stop': sequence.STOP}
        if operation not in operations or operation != 'sequence_stop' and (type(body.get('sequence_contract_revision')) is not int or body['sequence_contract_revision'] != sequence.VERSION):
            raise self.error('sequence_contract_mismatch', '逐颗校准页面版本不匹配，请刷新。', 400)
        target = body.get('id', 0)
        if type(target) is not int or target not in (calibration.MOTOR_IDS if operation in ('sequence_select', 'sequence_capture_hold') else (0,)):
            raise self.error('invalid_id', '逐颗校准需要准确的关节编号。', 400)
        if operation == 'sequence_stop':
            with owner.lock:
                request = self.stop_send()
            if request is None:
                raise self.error('release_unconfirmed', '卸力命令尚未确认，请保持支撑。', 503)
            self.wait(request, None, ('done', 'fault', 'off'))
            return {'ok': True, 'op': operation, 'status': owner.calibration_status()}
        with owner.lock:
            if operation == 'sequence_begin':
                if body.get('supported') is not True or body.get('clear') is not True:
                    raise self.error('confirmation_required', '先托稳机身并确认可以原位保持。')
                owner.require_idle()
                owner.require_fresh()
                if not self.fresh() or not owner.home_actual_off():
                    raise self.error('sequence_not_ready', '逐颗校准需要新版主控和15颗新鲜卸力反馈。')
                self.completed.clear()
                current = {'token': secrets.token_hex(24), 'nonce': secrets.randbelow(0xffffffff) + 1,
                           'boot': owner.snapshot['boot_id'], 'generation': owner.generation,
                           'client_at': owner.clock(), 'confirmed': False, 'output_started': False,
                           'prepare_deadline': owner.clock() + 30}
                self.current = current
                self.unknown = True
                request = self.send(sequence.BEGIN, current=current)
            else:
                current = self.session(body)
                if operation == 'sequence_heartbeat':
                    current['client_at'] = owner.clock()
                    return {'ok': True, 'op': operation, 'status': owner.calibration_status()}
                if self.pending is not None or not self.fresh() or not self.mask_verified() or self.packet['phase'] not in ('selected', 'holding'):
                    raise self.error('sequence_busy', '等待当前关节保持与记录完成。')
                if self.packet['held_mask'] & (1 << calibration.MOTOR_IDS.index(target)):
                    raise self.error('sequence_already_held', '该颗已保持；重新调整请先停止卸力。', ids=[target])
                if operation == 'sequence_capture_hold':
                    if body.get('pose_confirmed') is not True or body.get('supported') is not True:
                        raise self.error('confirmation_required', '先确认这颗已摆到机械零位并托稳机身。')
                    if self.packet['selected_id'] != target:
                        raise self.error('sequence_selection_changed', '选中的关节已变化，请重新选择。')
                    selected = dict(self.packet)
                    current['output_started'] = True
                    current['client_at'] = owner.clock()
                    request = self.send(sequence.HOLD, target, current, selected)
                else:
                    request = self.send(sequence.SELECT, target, current)
                self.pending = request
        try:
            if operation == 'sequence_begin':
                self.wait(request, current, ('selected',))
                with owner.lock:
                    current['confirmed'] = True
                return {'ok': True, 'op': operation, 'session': current['token'], 'status': owner.calibration_status()}
            receipt = self.wait(request, current, ('holding',) if operation == 'sequence_capture_hold' else ('selected',))
            if operation == 'sequence_capture_hold':
                evidence = {'selected_payload_hex': selected['raw_payload_hex'], 'held_payload_hex': receipt['raw_payload_hex'],
                            'user_confirmed_axis_pose': True, 'user_confirmed_support': True}
                groups = make_groups(owner.store, owner._groups, target, evidence, datetime.now(timezone.utc).isoformat())
                def check_commit():
                    with owner.lock:
                        packet = self.packet
                        if (self.current is not current or current['generation'] != owner.generation or not self.fresh()
                                or not self.mask_verified() or packet['phase'] != 'holding' or packet['error']
                                or packet['capture_seq'] != request or packet['held_mask'] != receipt['held_mask']
                                or packet['selection_seq'] != receipt['selection_seq']
                                or owner.clock() - current['client_at'] > .400):
                            raise self.error('sequence_save_cancelled', '记录期间保持证据已失效，未提交本颗零位。')
                group_file, native_file = owner.store.save_groups(groups, check_commit)
                with owner.lock:
                    check_commit()
                    owner._groups = groups
                    owner._group_summary = dict(group_file)
                    owner._records = owner.store.group_records(groups)
                    if native_file is not None:
                        owner._native_summary = dict(native_file)
                        owner._native_published_records = {sid: dict(value) for sid, value in owner._records.items()}
                    owner._home_arrival = None
                    self.completed.add(target)
            with owner.lock:
                self.pending = None
            return {'ok': True, 'op': operation, 'id': target, 'saved_id': target if operation == 'sequence_capture_hold' else None,
                    'holding_verified': operation == 'sequence_capture_hold', 'status': owner.calibration_status()}
        except Exception as error:
            with owner.lock:
                if self.current is current:
                    self.stop_send('operation_or_save_failed')
            if isinstance(error, (OSError, ValueError)) and not hasattr(error, 'code'):
                raise self.error('sequence_save_failed', '本颗零位记录或保持核验失败，已请求全部卸力。', 503) from None
            raise

    def tick(self):
        owner = self.owner
        with owner.lock:
            current = self.current
            if not current:
                if self.fresh() and self.packet['session'] and self.packet['phase'] in ('selected', 'capturing', 'enabling', 'holding'):
                    self.stop_send('orphaned_sequence')
                return
            if (not self.fresh() or current['boot'] != owner.snapshot['boot_id'] or current['generation'] != owner.generation
                    or self.lease_expired(current) or current.get('last_sent') in owner.failed_requests):
                self.stop_send('lease_or_feedback_expired')
            elif self.last_heartbeat is None or owner.clock() - self.last_heartbeat >= .100:
                try:
                    self.send(sequence.HEARTBEAT, current=current)
                    self.last_heartbeat = owner.clock()
                except Exception:
                    self.stop_send('heartbeat_send_failed')

    def receive_parameters(self, payload):
        owner = self.owner
        try:
            packet = sequence.decode_parameters(payload)
        except (ValueError, TypeError):
            return
        with owner.condition:
            expected = self.parameter_requests.get(packet['request_seq'])
            if expected == (packet['boot_id'], packet['id'], owner.generation) and packet['result'] != calibration.PENDING:
                self.parameters[packet['request_seq']] = packet
                owner.condition.notify_all()

    def read_parameters(self, body):
        owner = self.owner
        target = body.get('id')
        if type(target) is not int or target not in calibration.MOTOR_IDS:
            raise self.error('invalid_id', '请选择要读取的关节编号。', 400)
        with owner.lock:
            owner.require_idle()
            owner.require_fresh()
            if not owner.home_actual_off():
                raise self.error('parameters_require_off', '寄存器读取需要15颗新鲜卸力反馈。')
            owner.request_seq = (owner.request_seq + 1) & 0xffffffff or 1
            request = owner.request_seq
            self.parameter_requests[request] = (owner.snapshot['boot_id'], target, owner.generation)
            payload = calibration.COMMAND.pack(b'DWC1', 1, 10, target, 0, owner.snapshot['boot_id'], request, 0, owner.snapshot['sample_us'], 250000, 0, 0)
            owner.requests[request] = (10, target, 0, 0)
            owner.sender.put(request, 10, payload, owner.generation)
            owner.sent += 1
        deadline = time.monotonic() + 2
        with owner.condition:
            while request not in self.parameters:
                if request in owner.failed_requests:
                    owner.raise_delivery_error(request)
                if time.monotonic() >= deadline:
                    raise self.error('parameters_timeout', '寄存器只读结果尚未收到。', 503)
                owner.condition.wait(.025)
            packet = self.parameters[request]
            if packet['result'] != calibration.OK or packet['error']:
                raise self.error('parameters_rejected', '主控拒绝寄存器读取，错误' + str(packet['error']) + '。')
            return {'ok': True, 'op': 'read_servo_parameters', 'parameters': dict(packet), 'status': owner.calibration_status()}
