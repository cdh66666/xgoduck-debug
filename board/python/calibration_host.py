"""Authenticated commissioning only: no policy and no automatic zero loading."""
from device_config import HOME_TURN_TARGETS
from device_config import DEVICE_SERIAL
from collections import deque
from datetime import datetime, timezone
import hashlib
import hmac
import json
import math
import os
import queue
from pathlib import Path
import secrets
import threading
import time
import calibration_wire as W

VERSION = '2026-10-15-wireless-calibration-v7'
SCHEMA = 'wireless-calibration-v2'
FRESH_S, CLIENT_LEASE_S, SESSION_S, HEARTBEAT_S = .150, .400, 30.0, .100
HOME_MOTION_S = 450.0
HOME_TARGET_RESOLUTION = 'audited-finite-turn-v3'
HOME_TURN_REFERENCE_SCHEMA = 'xgoduck-home-turn-equivalence-v1'
HOME_TURN_REFERENCE_IDS = frozenset((10, 22, 30))
HOME_TURN_EQUIVALENT_TARGETS = HOME_TURN_TARGETS
STABLE_US, MAX_SPREAD = 500000, 3
SIGNS = {sid: -1 for sid in W.MOTOR_IDS}
# FEETECH STS integral-limit documentation explicitly covers position modes 0/4.
# https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/
POSITION_MODES = (0, 4)
LEGACY_AUDIT_VERSION = '2026-10-13-wireless-calibration-v2'
SINGLE_TURN_AUDIT_VERSIONS = (LEGACY_AUDIT_VERSION, '2026-10-13-wireless-calibration-v3')
NATIVE_AUDIT_VERSIONS = ('2026-10-13-wireless-calibration-v4', '2026-10-13-wireless-calibration-v5', '2026-10-13-wireless-calibration-v6', VERSION)
NATIVE_DOMAIN = 'native-sign-magnitude-v1'
SINGLE_DOMAIN = 'single-turn-0-4095'


def position_mode(mode):
    return type(mode) is int and mode in POSITION_MODES


def audit_position_mode(mode, host_version):
    # Old v2 evidence was certified under mode-0-only gates; do not widen it.
    return type(mode) is int and mode in ((0,) if host_version == LEGACY_AUDIT_VERSION else POSITION_MODES)


def coordinate_domain(mode):
    return SINGLE_DOMAIN if mode == 0 else NATIVE_DOMAIN if mode == 4 else None


def single_turn_position(raw, signed):
    return type(raw) is int and type(signed) is int and 0 <= raw <= 4095 and signed == raw


def capture_position(mode, raw, signed, host_version=VERSION):
    if not audit_position_mode(mode, host_version):
        return False
    if host_version in SINGLE_TURN_AUDIT_VERSIONS or mode == 0:
        return single_turn_position(raw, signed)
    return (type(raw) is int and type(signed) is int and 0 <= raw <= 65535 and raw != 32768
            and -32767 <= signed <= 32767 and signed == W.signed_word(raw))


def home_position_allowed(sid, current, target):
    if type(current) is not int or type(target) is not int:return False
    q=-(current-target)*360
    if sid==30:return -682<=current-target<=2047
    if sid==31:return -105*4096<=q<=105*4096
    return abs(current-target)<=1024


def resolve_home_target(saved, current, mode, reference=None, sid=None):
    """Return a temporary native target; never rewrite saved zero or feedback.

    Only a finite, evidence-bound pair may be considered. Never invent a
    target by rounding to the nearest revolution. PREPARE freezes the selected
    target together with the live boot/Host/file identities.
    """
    if (type(saved) is not int or type(current) is not int
            or not -32767 <= saved <= 32767 or not -32767 <= current <= 32767
            or not position_mode(mode)):
        return None, None
    if mode == 0 and not (0 <= saved <= 4095 and 0 <= current <= 4095):
        return None, None
    if reference is not None:
        if not isinstance(reference, dict):
            return None, None
        target, offset = reference.get('target'), reference.get('turn_offset')
        targets = reference.get('equivalent_targets')
        if (mode != 4 or reference.get('mode') != 4 or reference.get('saved') != saved
                or type(offset) is not int or offset not in (-1, 1)
                or type(target) is not int or target != saved+4096*offset
                or not -32767 <= target <= 32767 or not isinstance(targets, list)
                or len(targets) != 2 or any(type(item) is not int for item in targets)
                or sorted(targets) != sorted((saved, target))):
            return None, None
        matches = [item for item in targets if home_position_allowed(sid, current, item)]
        return (matches[0], 0 if matches[0] == saved else offset) if len(matches) == 1 else (None, None)
    return (saved, 0) if home_position_allowed(sid,current,saved) else (None, None)


def load_home_turn_reference(data_dir):
    """Optional startup-only audit; never writes native zeros or config."""
    path = Path(data_dir) / 'home-turn-reference.json'
    try:
        if path.is_symlink() or path.stat().st_size > 16384:
            return None
        raw = path.read_bytes()
        value = json.loads(raw)
        def valid_hash(item):
            return isinstance(item, str) and len(item) == 64 and all(c in '0123456789abcdef' for c in item)
        if (not isinstance(value, dict) or value.get('schema') != HOME_TURN_REFERENCE_SCHEMA
                or value.get('device_serial') != DEVICE_SERIAL
                or type(value.get('source_boot_id')) is not int or not 0 < value['source_boot_id'] <= 0xffffffff
                or value.get('user_confirmed_original_pose') is not True
                or not all(valid_hash(value.get(key)) for key in
                           ('native_sha256', 'group_records_sha256', 'source_evidence_sha256'))
                or not isinstance(value.get('source_app_instance_id'), str)
                or len(value['source_app_instance_id']) != 32
                or any(c not in '0123456789abcdef' for c in value['source_app_instance_id'])
                or not isinstance(value.get('axes'), dict) or not value['axes']
                or not set(value['axes']).issubset({str(sid) for sid in HOME_TURN_REFERENCE_IDS})):
            return None
        for sid, axis in value['axes'].items():
            if not isinstance(axis, dict):
                return None
            saved, target, offset = axis.get('saved'), axis.get('target'), axis.get('turn_offset')
            positions = axis.get('confirmed_positions')
            if (type(saved) is not int or type(target) is not int or type(offset) is not int
                    or axis.get('mode') != 4 or offset not in (-1, 1)
                    or (saved, target) != HOME_TURN_EQUIVALENT_TARGETS[sid]
                    or not isinstance(axis.get('equivalent_targets'), list)
                    or len(axis['equivalent_targets']) != 2
                    or any(type(item) is not int for item in axis['equivalent_targets'])
                    or sorted(axis['equivalent_targets']) != sorted((saved, target))
                    or not valid_hash(axis.get('saved_record_sha256'))
                    or target != saved+4096*offset
                    or not -32767 <= saved <= 32767 or not -32767 <= target <= 32767):
                return None
            if sid == '30':
                observations = axis.get('boot_shift_observations')
                if (axis.get('evidence_kind') != 'passive_boot_turn_shift'
                        or not isinstance(observations, list) or len(observations) != 2):
                    return None
                for observation in observations:
                    if (not isinstance(observation, dict)
                            or type(observation.get('boot_id')) is not int
                            or not 0 < observation['boot_id'] <= 0xffffffff
                            or observation.get('mode') != 4
                            or observation.get('fresh') is not True
                            or observation.get('torque_off_verified') is not True
                            or type(observation.get('position')) is not int
                            or not -32767 <= observation['position'] <= 32767
                            or not valid_hash(observation.get('source_sha256'))
                            or observation.get('saved_record_sha256') != axis['saved_record_sha256']):
                        return None
                before, after = observations
                if (before['boot_id'] == after['boot_id']
                        or after['position'] - before['position'] != 4096 * offset
                        or not home_position_allowed(30, before['position'], saved)
                        or not home_position_allowed(30, after['position'], target)):
                    return None
            elif (not isinstance(positions, list) or len(positions) < 2
                    or any(type(position) is not int or abs(position-target) > 64 for position in positions)):
                return None
        return {'audit': value, 'sha256': hashlib.sha256(raw).hexdigest()}
    except (OSError, ValueError, TypeError, KeyError):
        return None


class CalibrationError(ValueError):
    def __init__(self, code, message, status=409, ids=()):
        super().__init__(message)
        self.code, self.status = code, status
        self.ids = [sid for sid in ids if sid in W.MOTOR_IDS]


def load_private_key(data_dir):
    data_dir = Path(data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    path = data_dir / 'calibration_key'
    if data_dir.is_symlink() or path.is_symlink():
        raise RuntimeError('Authentication path cannot be redirected')
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        pass
    else:
        with os.fdopen(descriptor, 'w', encoding='ascii') as stream:
            stream.write(secrets.token_hex(32) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
    if os.name == 'posix':
        os.chmod(path, 0o600)
    key = path.read_text(encoding='ascii').strip()
    if len(key) != 64 or any(c not in '0123456789abcdef' for c in key):
        raise RuntimeError('Authentication file is invalid')
    return key


class ZeroStore:
    def __init__(self, data_dir):
        self.directory = Path(data_dir)
        self.path = self.directory / 'zero_pos.json'
        self.native_path = self.directory / 'zero_pos.native.json'
        self.group_path = self.directory / 'group_records.json'
        self.lock = threading.RLock()

    def read(self):
        with self.lock:
            if not self.path.exists():
                return {'present': False, 'verified': False, 'sha256': None}, {}
            try:
                raw = self.path.read_bytes()
                value = json.loads(raw)
                zeros = value['zero_pos']
                audit = value.get('audit', {})
                if not isinstance(audit, dict):
                    raise ValueError('invalid zero audit')
                if value.get('ids') != list(W.MOTOR_IDS) or len(zeros) != 15 or any(type(v) is not int or not 0 <= v <= 4095 for v in zeros):
                    raise ValueError('invalid zero file')
                records = {str(sid): {'position': zeros[i], 'saved_at': value.get('saved_at'),
                                     'physical_acceptance_verified': False} for i, sid in enumerate(W.MOTOR_IDS)}
                return {'present': True, 'verified': True, 'sha256': hashlib.sha256(raw).hexdigest(),
                        'group_records_sha256': audit.get('group_records_sha256')}, records
            except (OSError, ValueError, KeyError, TypeError):
                return {'present': True, 'verified': False, 'sha256': None}, {}

    def save(self, zeros, audit, before_commit):
        if len(zeros) != 15 or any(type(v) is not int or not 0 <= v <= 4095 for v in zeros):
            raise CalibrationError('invalid_zeros', '需要全部 15 个有效编码值。')
        value = {'version': 1, 'ids': list(W.MOTOR_IDS), 'zero_pos': list(zeros),
                 'saved_at': datetime.now(timezone.utc).isoformat(), 'audit': audit,
                 'physical_acceptance_verified': False, 'policy_applied': False}
        raw = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)+'\n').encode('utf-8')
        with self.lock:
            self.directory.mkdir(parents=True, exist_ok=True)
            if self.directory.is_symlink() or self.path.is_symlink():
                raise CalibrationError('unsafe_path', '零位文件路径不能重定向。')
            old = self.path.read_bytes() if self.path.exists() else None
            temporary = self.directory / ('.zero-pos-'+secrets.token_hex(12)+'.tmp')
            committed = False
            try:
                descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                with os.fdopen(descriptor, 'wb') as stream:
                    stream.write(raw); stream.flush(); os.fsync(stream.fileno())
                if temporary.read_bytes() != raw:
                    raise OSError('temporary readback differs')
                before_commit()
                if (self.path.read_bytes() if self.path.exists() else None) != old:
                    raise OSError('zero file changed before commit')
                os.replace(temporary, self.path)
                committed = True
                if os.name == 'posix':
                    directory_fd = os.open(self.directory, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
                    try:
                        os.fsync(directory_fd)
                    finally:
                        os.close(directory_fd)
                if self.path.read_bytes() != raw:
                    raise OSError('saved readback differs')
                before_commit()  # A cancellation during file I/O rolls back the transaction.
                return {'present': True, 'verified': True, 'sha256': hashlib.sha256(raw).hexdigest()}
            except Exception:
                if committed:
                    if old is None:
                        self.path.unlink(missing_ok=True)
                    else:
                        with temporary.open('wb') as stream:
                            stream.write(old); stream.flush(); os.fsync(stream.fileno())
                        os.replace(temporary, self.path)
                    if (self.path.read_bytes() if self.path.exists() else None) != old:
                        raise CalibrationError('rollback_failed', '保存及恢复未核实，请保持卸力并检查文件。')
                raise
            finally:
                temporary.unlink(missing_ok=True)

    @staticmethod
    def group_records(groups):
        records = {}
        for name, group in groups.items():
            if group.get('format') == 'head-mouth-override-v1':
                records.update({sid: dict(value) for sid, value in group['records'].items()})
                continue
            packet = W.decode_snapshot(bytes.fromhex(group['audit']['snapshot_payload_hex']))
            for sid in W.GROUPS[name][2]:
                row = packet['joints'][W.MOTOR_IDS.index(sid)]
                records[str(sid)] = {**group['records'][str(sid)], 'raw_word': row['raw_word'],
                                    'signed_steps': row['signed_steps'], 'mode': row['mode'],
                                    'coordinate_domain': coordinate_domain(row['mode']),
                                    'historical': True, 'model_deployment_ready': False}
        return records

    def read_native(self):
        with self.lock:
            if not self.native_path.exists():
                return {'present': False, 'verified': False, 'sha256': None, 'path': 'data/zero_pos.native.json'}, {}
            try:
                if self.directory.is_symlink() or self.native_path.is_symlink():
                    raise ValueError('redirected native file')
                raw = self.native_path.read_bytes()
                value = json.loads(raw)
                groups = value['audit']['groups']
                if (value.get('version') != 2 or value.get('coordinate_domain') != NATIVE_DOMAIN
                        or value.get('model_deployment_ready') is not False or value.get('policy_applied') is not False
                        or value.get('ids') != list(W.MOTOR_IDS) or set(groups) != set(W.GROUPS)):
                    raise ValueError('invalid native evidence file')
                for name, group in groups.items():
                    self.validate_group(name, group)
                records = self.group_records(groups)
                if (self._json_bytes(value.get('records')) != self._json_bytes(records)
                        or any(type(number) is not int for key in ('zero_native','raw_words','modes') for number in value.get(key, []))
                        or value.get('zero_native') != [records[str(sid)]['position'] for sid in W.MOTOR_IDS]
                        or value.get('raw_words') != [records[str(sid)]['raw_word'] for sid in W.MOTOR_IDS]
                        or value.get('modes') != [records[str(sid)]['mode'] for sid in W.MOTOR_IDS]):
                    raise ValueError('native record does not match original evidence')
                expected_group = {'schema': SCHEMA, 'groups': groups, 'complete': True,
                                  'physical_acceptance_verified': False, 'policy_applied': False}
                if value['audit'].get('group_records_sha256') != hashlib.sha256(self._json_bytes(expected_group)).hexdigest():
                    raise ValueError('native publication audit hash differs')
                return {'present': True, 'verified': True, 'sha256': hashlib.sha256(raw).hexdigest(),
                        'group_records_sha256': value['audit'].get('group_records_sha256'),
                        'coordinate_domain': NATIVE_DOMAIN, 'model_deployment_ready': False,
                        'path': 'data/zero_pos.native.json'}, records
            except (OSError, ValueError, KeyError, TypeError):
                return {'present': True, 'verified': False, 'sha256': None, 'path': 'data/zero_pos.native.json'}, {}

    @staticmethod
    def validate_group(name, record):
        """Persisted evidence is historical; it never certifies today's pose."""
        if isinstance(record, dict) and record.get('format') == 'head-mouth-override-v1':
            if name != 'head':
                raise ValueError('mouth override is head-only')
            return ZeroStore.validate_mouth_override(record)
        _, mask, ids = W.GROUPS[name]
        if not isinstance(record, dict) or record.get('ids') != list(ids) or set(record.get('records', {})) != {str(sid) for sid in ids}:
            raise ValueError('group IDs differ')
        audit = record['audit']
        packet = W.decode_snapshot(bytes.fromhex(audit['snapshot_payload_hex']))
        if (audit.get('host_version') not in (*SINGLE_TURN_AUDIT_VERSIONS, *NATIVE_AUDIT_VERSIONS) or audit.get('snapshot_version') != W.SNAPSHOT_VERSION
                or audit.get('user_confirmed_group_pose') is not True
                or audit.get('user_confirmed_support') is not True
                or packet['op'] not in (W.CAPTURE_ALL, W.CAPTURE_GROUP)
                or packet['result'] != W.OK or packet['state'] == W.FAULT
                or packet['operation_ack_seq'] != audit['operation_ack_seq']
                or packet['last_received_req_seq'] != audit['operation_ack_seq']
                or packet['capture_mask'] != (W.ALL_MASK if packet['op'] == W.CAPTURE_ALL else mask)
                or packet['metadata_mask'] != W.ALL_MASK
                or packet['flags'] & W.TORQUE_MAY_BE_ON
                or not packet['flags'] & W.ALL_TORQUE_OFF_VERIFIED
                or not packet['flags'] & W.POLICY_LOCKED
                or any(row['torque'] != 0 or row['metadata_age_ms'] > 750 for row in packet['joints'])):
            raise ValueError('group lacks complete off/readback evidence')
        stability = audit['stability']
        if stability['samples'] < 3 or stability['duration_ms'] < 500 or max(stability['spread_ticks']) > MAX_SPREAD:
            raise ValueError('group lacks stable window')
        samples = [W.decode_snapshot(bytes.fromhex(raw)) for raw in stability['snapshot_payloads_hex']]
        if len(samples) != stability['samples'] or len(samples) < 3 or samples[0]['raw_payload_hex'] != packet['raw_payload_hex']:
            raise ValueError('stable sample evidence differs')
        if (W.elapsed(samples[0]['sample_us'], samples[-1]['sample_us']) != int(stability['duration_ms'] * 1000)
                or W.elapsed(samples[0]['sample_us'], samples[-1]['sample_us']) < STABLE_US):
            raise ValueError('stable sample duration differs')
        previous = None
        for sample in samples:
            if (sample['boot_id'] != packet['boot_id'] or sample['operation_ack_seq'] != packet['operation_ack_seq']
                    or sample['last_received_req_seq'] != packet['operation_ack_seq']
                    or sample['op'] != packet['op'] or sample['result'] != W.OK
                    or sample['capture_mask'] != packet['capture_mask'] or sample['metadata_mask'] != W.ALL_MASK
                    or sample['state'] == W.FAULT or sample['flags'] & W.TORQUE_MAY_BE_ON
                    or not sample['flags'] & W.ALL_TORQUE_OFF_VERIFIED
                    or not sample['flags'] & W.POLICY_LOCKED
                    or any(row['torque'] != 0 for row in sample['joints'])):
                raise ValueError('stable sample safety differs')
            if previous and not 0 < W.elapsed(previous['sample_us'], sample['sample_us']) <= 150000:
                raise ValueError('stable samples repeated or discontinuous')
            previous = sample
        for sid in ids:
            index = W.MOTOR_IDS.index(sid)
            row, value = packet['joints'][index], record['records'][str(sid)]['position']
            if (type(value) is not int or row['signed_steps'] != value
                    or not capture_position(row['mode'], row['raw_word'], row['signed_steps'], audit['host_version'])
                    or not packet['position_mask'] & (1 << index)
                    or not packet['stable_mask'] & (1 << index)):
                raise ValueError('group position is not certified')
            values = []
            for sample in samples:
                row = sample['joints'][index]
                if (not sample['position_mask'] & (1 << index)
                        or not capture_position(row['mode'], row['raw_word'], row['signed_steps'], audit['host_version'])):
                    raise ValueError('stable position sample is invalid')
                values.append(row['signed_steps'])
            if max(values) - min(values) > MAX_SPREAD:
                raise ValueError('recorded group moved')
            if audit['host_version'] in NATIVE_AUDIT_VERSIONS:
                saved = record['records'][str(sid)]
                latest, first = packet['joints'][index], samples[-1]['joints'][index]
                expected = {'mode': latest['mode'], 'raw_word': latest['raw_word'],
                            'signed_steps': latest['signed_steps'], 'coordinate_domain': coordinate_domain(latest['mode'])}
                if any(type(saved.get(key)) is not type(val) or saved.get(key) != val for key, val in expected.items()):
                    raise ValueError('native saved value does not bind original raw word')
                endpoints = audit['raw_signed_endpoints'][str(sid)]
                if any(type(endpoints.get(key)) is not int for key in ('mode','first_raw_word','first_signed_steps','last_raw_word','last_signed_steps')) or endpoints != {'mode': latest['mode'], 'first_raw_word': first['raw_word'],
                                 'first_signed_steps': first['signed_steps'], 'last_raw_word': latest['raw_word'],
                                 'last_signed_steps': latest['signed_steps']}:
                    raise ValueError('native endpoints differ from original snapshots')
                if (audit.get('coordinate_domain') != NATIVE_DOMAIN or audit.get('model_deployment_ready') is not False
                        or any(sample['joints'][index]['mode'] != latest['mode'] or sample['joints'][index]['position_age_ms'] >= 150 for sample in samples)):
                    raise ValueError('native coordinate mode changed during capture')
                ack = W.decode_snapshot(bytes.fromhex(audit['ack_snapshot_payload_hex']))
                if (ack['boot_id'] != packet['boot_id'] or ack['op'] != packet['op'] or ack['result'] != W.OK
                        or ack['operation_ack_seq'] != packet['operation_ack_seq']
                        or ack['last_received_req_seq'] != packet['operation_ack_seq']
                        or ack['capture_mask'] != packet['capture_mask'] or ack['metadata_mask'] != W.ALL_MASK
                        or ack['state'] != W.IDLE or not ack['flags'] & W.POLICY_LOCKED
                        or not ack['flags'] & W.ALL_TORQUE_OFF_VERIFIED
                        or ack['flags'] & (W.TORQUE_MAY_BE_ON | W.SESSION_ACTIVE | W.CAPTURE_BUSY)
                        or any(r['torque'] != 0 or r['metadata_age_ms'] > 750 for r in ack['joints'])):
                    raise ValueError('native acknowledgement does not certify same capture')
                ack_row = ack['joints'][index]
                if (ack_row['mode'] != latest['mode'] or not ack['position_mask'] & (1 << index)
                        or not capture_position(ack_row['mode'], ack_row['raw_word'], ack_row['signed_steps'])
                        or ack_row['position_age_ms'] >= 150 or abs(ack_row['signed_steps'] - value) > MAX_SPREAD):
                    raise ValueError('native acknowledgement raw position differs')


    @staticmethod
    def mouth_observed_record(evidence_raw, saved_at):
        """Validate passive Host observations; never claim a full-head capture."""
        if not isinstance(evidence_raw, str) or len(evidence_raw) > 300000 or not isinstance(saved_at, str):
            raise ValueError('invalid mouth evidence')
        evidence = json.loads(evidence_raw)
        observations = evidence.get('observations')
        if (evidence.get('user_confirmed_closed') is not True
                or not isinstance(evidence.get('user_statement'), str) or not evidence['user_statement']
                or not isinstance(observations, list) or not 3 <= len(observations) <= 32):
            raise ValueError('mouth closure was not confirmed')
        previous = None; positions = []; boot = mode = first_time = None
        for observation in observations:
            stamp, status = observation['at_monotonic'], observation['status']
            if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or status.get('schema') != SCHEMA or status.get('version') not in NATIVE_AUDIT_VERSIONS
                    or status.get('fresh') is not True or status.get('policy_locked') is not True
                    or status.get('all_torque_off_verified') is not True or status.get('torque_may_be_on') is not False
                    or status.get('session_active') is not False or status.get('operation') is not None
                    or status.get('home', {}).get('active') is not False
                    or type(status.get('boot_id')) is not int or not 0 < status['boot_id'] <= 0xffffffff
                    or type(status.get('sequence')) is not int
                    or type(status.get('age_ms')) not in (int, float) or not 0 <= status['age_ms'] < 150):
                raise ValueError('mouth observation is stale, active, or unbound')
            if previous is not None and (not 0 < stamp-previous[0] <= 2
                    or not 0 < W.elapsed(status['sequence'], previous[1]) < 0x80000000
                    or status['boot_id'] != boot):
                raise ValueError('mouth observations changed boot or repeated')
            if first_time is None:first_time=stamp;boot=status['boot_id']
            rows = status.get('joints')
            if (not isinstance(rows, list) or len(rows) != 15
                    or [row.get('id') for row in rows] != list(W.MOTOR_IDS)
                    or any(row.get('torque_enabled') is not False or row.get('torque_off_verified') is not True
                           or row.get('fresh') is not True for row in rows)):
                raise ValueError('mouth evidence lacks all-axis off readback')
            row = rows[14]
            if (row.get('valid') is not True or row.get('stable') is not True
                    or not capture_position(row.get('mode'), row.get('raw_word'), row.get('signed_steps'))
                    or row.get('position') != row.get('signed_steps')
                    or type(row.get('position_age_ms')) not in (int, float) or not 0 <= row['position_age_ms'] < 150
                    or type(row.get('metadata_age_ms')) not in (int, float) or not 0 <= row['metadata_age_ms'] <= 750
                    or mode is not None and mode != row['mode']):
                raise ValueError('mouth encoder evidence is invalid')
            mode=row['mode'];positions.append(row['position']);previous=(stamp,status['sequence'])
        if previous[0]-first_time < .5 or max(positions)-min(positions) > MAX_SPREAD:
            raise ValueError('mouth samples are insufficient or moved')
        if evidence.get('positions') != positions or evidence.get('spread_ticks') != max(positions)-min(positions):
            raise ValueError('mouth evidence summary differs')
        return {'position':row['position'],'raw_word':row['raw_word'],'signed_steps':row['signed_steps'],
                'mode':mode,'coordinate_domain':coordinate_domain(mode),'saved_at':saved_at,
                'group':'head','historical':True,'model_deployment_ready':False,'physical_acceptance_verified':False}

    @staticmethod
    def validate_mouth_override(record):
        baseline, audit = record['baseline'], record['audit']
        if (not isinstance(baseline, dict) or baseline.get('format') is not None
                or record.get('ids') != list(W.GROUPS['head'][2])
                or audit.get('source') != 'passive_host_mouth_closed_v1'
                or audit.get('user_confirmed_group_pose') is not False):
            raise ValueError('invalid composite mouth group')
        ZeroStore.validate_group('head', baseline)
        expected = ZeroStore.group_records({'head':baseline})
        if audit.get('baseline_sha256') != hashlib.sha256(ZeroStore._json_bytes(baseline)).hexdigest():
            raise ValueError('mouth baseline identity differs')
        raw = audit['evidence_json']
        if not isinstance(raw, str) or audit.get('evidence_sha256') != hashlib.sha256(raw.encode('utf-8')).hexdigest():
            raise ValueError('mouth evidence identity differs')
        expected['34'] = ZeroStore.mouth_observed_record(raw, audit['saved_at'])
        if ZeroStore._json_bytes(record.get('records')) != ZeroStore._json_bytes(expected):
            raise ValueError('mouth override changed unconfirmed records')

    @staticmethod
    def make_mouth_override(groups, evidence_raw, saved_at):
        """Construct the authorized ID34-only update; caller uses save_groups."""
        groups=json.loads(ZeroStore._json_bytes(groups))  # Do not alias or mutate the prior evidence.
        if set(groups) != set(W.GROUPS):raise ValueError('complete baseline required')
        for name, group in groups.items():ZeroStore.validate_group(name, group)
        baseline=groups['head'].get('baseline') if groups['head'].get('format') == 'head-mouth-override-v1' else groups['head']
        records=ZeroStore.group_records({'head':baseline})
        records['34']=ZeroStore.mouth_observed_record(evidence_raw,saved_at)
        override={'format':'head-mouth-override-v1','ids':list(W.GROUPS['head'][2]),'baseline':baseline,'records':records,
                  'audit':{'source':'passive_host_mouth_closed_v1','user_confirmed_group_pose':False,
                           'baseline_sha256':hashlib.sha256(ZeroStore._json_bytes(baseline)).hexdigest(),
                           'evidence_json':evidence_raw,'evidence_sha256':hashlib.sha256(evidence_raw.encode('utf-8')).hexdigest(),
                           'saved_at':saved_at}}
        ZeroStore.validate_mouth_override(override)
        return {**groups,'head':override}

    def read_groups(self):
        with self.lock:
            if not self.group_path.exists():
                return {'present': False, 'verified': False, 'sha256': None}, {}
            try:
                if self.directory.is_symlink() or self.group_path.is_symlink():
                    raise ValueError('redirected group path')
                raw = self.group_path.read_bytes()
                value = json.loads(raw)
                groups = value['groups']
                if value.get('schema') != SCHEMA or not isinstance(groups, dict) or not groups or set(groups) - set(W.GROUPS):
                    raise ValueError('invalid group file')
                for name, record in groups.items():
                    self.validate_group(name, record)
                return {'present': True, 'verified': True, 'sha256': hashlib.sha256(raw).hexdigest()}, groups
            except (OSError, ValueError, KeyError, TypeError):
                return {'present': True, 'verified': False, 'sha256': None}, {}

    @staticmethod
    def _json_bytes(value):
        return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)+'\n').encode('utf-8')

    def _fsync_directory(self):
        if os.name == 'posix':
            descriptor = os.open(self.directory, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)

    def save_groups(self, groups, before_commit):
        """Durable group evidence precedes atomic complete-zero publication.

        A process/power interruption between files can leave complete pending
        evidence and the previous published zero. Startup never auto-publishes.
        Normal cancellation/error rolls back both files using pre-fsynced backups.
        """
        for name, record in groups.items():
            self.validate_group(name, record)
        records = self.group_records(groups)
        complete = set(records) == {str(sid) for sid in W.MOTOR_IDS}
        group_value = {'schema': SCHEMA, 'groups': groups, 'complete': complete,
                       'physical_acceptance_verified': False, 'policy_applied': False}
        targets = [(self.group_path, self._json_bytes(group_value))]
        if complete:
            zero_value = {'version': 2, 'ids': list(W.MOTOR_IDS), 'coordinate_domain': NATIVE_DOMAIN,
                          'zero_native': [records[str(sid)]['position'] for sid in W.MOTOR_IDS],
                          'raw_words': [records[str(sid)]['raw_word'] for sid in W.MOTOR_IDS],
                          'modes': [records[str(sid)]['mode'] for sid in W.MOTOR_IDS], 'records': records,
                          'model_deployment_ready': False, 'legacy_runtime_compatible': False,
                          'saved_at': datetime.now(timezone.utc).isoformat(),
                          'audit': {**(next(iter(groups.values()))['audit']
                                       if all(group['audit'].get('user_confirmed_all_poses') for group in groups.values()) else {}),
                                    'source': ('DWR1_groups_with_passive_mouth_override' if groups.get('head', {}).get('format') == 'head-mouth-override-v1' else 'DWR1_group_captures'), 'host_version': VERSION,
                                    'snapshot_version': W.SNAPSHOT_VERSION, 'groups': groups,
                                    'group_records_sha256': hashlib.sha256(targets[0][1]).hexdigest(),
                                    'groups_captured_at_different_times': True},
                          'physical_acceptance_verified': False, 'policy_applied': False}
            targets.append((self.native_path, self._json_bytes(zero_value)))
        with self.lock:
            self.directory.mkdir(parents=True, exist_ok=True)
            if self.directory.is_symlink() or any(path.is_symlink() for path, _ in targets):
                raise CalibrationError('unsafe_path', '记录路径不能重定向。')
            prepared, committed, rollback_failed = [], [], False
            try:
                # Backups are durable before any public file is replaced.
                for path, raw in targets:
                    old = path.read_bytes() if path.exists() else None
                    temp = self.directory / ('.capture-'+secrets.token_hex(12)+'.tmp')
                    backup = self.directory / ('.capture-'+secrets.token_hex(12)+'.bak') if old is not None else None
                    prepared.append((path, raw, old, temp, backup))
                    for item_path, content in ((temp, raw), (backup, old)):
                        if item_path is None:
                            continue
                        descriptor = os.open(item_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                        with os.fdopen(descriptor, 'wb') as stream:
                            stream.write(content); stream.flush(); os.fsync(stream.fileno())
                        if item_path.read_bytes() != content:
                            raise OSError('prepared readback differs')
                self._fsync_directory()
                before_commit()
                for path, raw, old, temp, backup in prepared:
                    before_commit()
                    if (path.read_bytes() if path.exists() else None) != old:
                        raise OSError('record file changed before commit')
                    os.replace(temp, path)
                    committed.append((path, old, backup))
                    self._fsync_directory()
                    if path.read_bytes() != raw:
                        raise OSError('saved readback differs')
                    before_commit()
                return ({'present': True, 'verified': True, 'sha256': hashlib.sha256(targets[0][1]).hexdigest()},
                        {'present': True, 'verified': True, 'sha256': hashlib.sha256(targets[-1][1]).hexdigest(),
                         'group_records_sha256': hashlib.sha256(targets[0][1]).hexdigest(),
                         'coordinate_domain': NATIVE_DOMAIN, 'model_deployment_ready': False,
                         'path': 'data/zero_pos.native.json'} if complete else None)
            except Exception:
                for path, old, backup in reversed(committed):
                    try:
                        if old is None:
                            path.unlink(missing_ok=True)
                        else:
                            # Keep the pre-fsynced backup until directory recovery
                            # is durable, even if that fsync subsequently fails.
                            recovery = self.directory / ('.capture-'+secrets.token_hex(12)+'.recover')
                            os.link(backup, recovery)
                            try:
                                os.replace(recovery, path)
                            finally:
                                recovery.unlink(missing_ok=True)
                        if (path.read_bytes() if path.exists() else None) != old:
                            raise OSError('rollback readback differs')
                    except OSError:
                        rollback_failed = True
                        raise CalibrationError('rollback_failed', '保存及恢复未核实，请保持卸力并检查文件。') from None
                if committed:
                    try:
                        self._fsync_directory()
                    except OSError:
                        rollback_failed = True
                        raise CalibrationError('rollback_failed', '恢复文件读回一致，但目录持久化未确认，请检查保存记录。') from None
                raise
            finally:
                for path, raw, old, temp, backup in prepared:
                    try:
                        temp.unlink(missing_ok=True)
                        if backup is not None and not rollback_failed:
                            backup.unlink(missing_ok=True)
                    except OSError:
                        pass  # Cleanup cannot change a verified commit/rollback result.


class SerialSender:
    """A single FIFO writer; potentially slow SDK I/O never owns the host mutex."""
    def __init__(self, owner, bridge):
        if not callable(getattr(bridge, 'notify', None)):
            raise RuntimeError('The installed Arduino SDK does not provide Bridge.notify')
        self.owner, self.bridge = owner, bridge
        self.queue = queue.Queue(maxsize=32)
        self.stop_event = threading.Event()
        self.attempted = 0
        self.worker = threading.Thread(target=self.run, name='calibration-rpc-writer', daemon=True)
        self.worker.start()

    def put(self, request, op, payload, generation):
        try:
            with self.owner.lock:
                boot = self.owner.snapshot['boot_id'] if self.owner.snapshot else None
            self.queue.put_nowait((request, op, payload, generation, boot))
        except queue.Full:
            raise CalibrationError('transport_busy', '主控通信队列已满；未自动重试。', 503) from None

    def run(self):
        while not self.stop_event.is_set():
            try:
                request, op, payload, generation, boot = self.queue.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                with self.owner.lock:
                    allowed = (self.owner.snapshot and self.owner.snapshot['boot_id'] == boot
                               and (generation == self.owner.generation or op in (W.CANCEL, W.OFF_ALL)))
                if allowed:
                    self.bridge.notify('duck_calibration', payload)
                    with self.owner.lock:
                        self.attempted += 1
            except Exception as error:
                code = getattr(error, 'code', 'bridge_send_failed')
                self.owner.send_failed(request, op, code, boot=boot, generation=generation)
            finally:
                self.queue.task_done()

    def close(self):
        self.stop_event.set()
        self.worker.join(timeout=.2)


class WirelessCalibrationHost:
    def __init__(self, telemetry, bridge, key, data_dir, clock=time.monotonic):
        self.telemetry, self.bridge, self._key, self.clock = telemetry, bridge, key, clock
        self.store = ZeroStore(data_dir)
        self._home_turn_reference = load_home_turn_reference(data_dir)
        # Bind the persistent finite-set evidence to this Host object. Its
        # source boot is historical; each PREPARE binds the actual live boot.
        # No restart restores an action/session/target stream.
        self._home_turn_instance = getattr(telemetry, 'app_instance_id', None)
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.snapshot = self.observed_at = None
        self.history = deque(maxlen=128)
        self.seen_boots = set()
        self.request_seq = 0
        self._session = self.operation = self.last_heartbeat = None
        self._capture_owner = None
        self._home_session = self._home_snapshot = self._home_observed_at = self._home_heartbeat = None
        self._home_unknown = False
        self._home_arrival = None
        self._home_idle_release = None
        self._home_faults = deque(maxlen=8)
        self._home_cancellations = deque(maxlen=8)
        self.generation = self.sent = 0
        self.errors = deque(maxlen=20)
        self.results = {}
        self.requests = {}
        self._file_summary, self._published_records = self.store.read()
        self._group_summary, self._groups = self.store.read_groups()
        self._native_summary, self._native_published_records = self.store.read_native()
        self._records = self.store.group_records(self._groups)  # Startup only; API/lease callbacks never do file I/O.
        self.failed_requests = set()
        self.delivery_errors = {}
        self.protocol_fault = False
        self.sender = SerialSender(self, bridge)

    def receive(self, payload):
        try:
            packet = W.decode_snapshot(payload)
        except (ValueError, TypeError):
            with self.condition:
                self.errors.append({'code': 'invalid_calibration_snapshot'})
                current, self._session = self._session, None
                self.protocol_fault = True; self.generation += 1; self.history.clear()
                if self._home_session or self._home_unknown:
                    self.home_stop_send()
                if current:
                    try:
                        self.send(W.CANCEL, current['id'], current['nonce'])
                    except Exception:
                        self.errors.append({'code': 'invalid_frame_cancel_failed', 'ids': [current['id']]})
                self.condition.notify_all()
            return
        with self.condition:
            previous = self.snapshot
            if previous and packet['boot_id'] == previous['boot_id']:
                step = W.elapsed(packet['sequence'], previous['sequence'])
                if step == 0 or step >= 0x80000000:
                    return
                if W.elapsed(packet['sample_us'], previous['sample_us']) >= 0x80000000:
                    return
            elif previous:
                if packet['boot_id'] in self.seen_boots:
                    return
                self.history.clear(); self._session = None; self._home_session = None; self._home_snapshot = None; self._home_observed_at = None; self._home_unknown = False; self._home_arrival = None; self.operation = None; self.generation += 1
                self._home_idle_release = None
                self._capture_owner = None
                self.results.clear(); self.requests.clear()
                self.failed_requests.clear(); self.delivery_errors.clear()
                self.errors.append({'code': 'mcu_restarted'})
            if not previous or packet['boot_id'] != previous['boot_id']:
                self.request_seq = packet['last_received_req_seq']
            self.seen_boots.add(packet['boot_id'])
            self.protocol_fault = False
            self.snapshot, self.observed_at = packet, self.clock()
            self.history.append((packet, self.observed_at))
            self.home_reconcile()
            self.condition.notify_all()

    def receive_result(self, payload):
        try:
            result = W.decode_action_result(payload)
        except (ValueError, TypeError):
            with self.lock:
                self.errors.append({'code': 'invalid_calibration_result'})
            return
        with self.condition:
            if not self.snapshot or result['boot_id'] != self.snapshot['boot_id']:
                return
            request = self.requests.get(result['request_seq'])
            if not request or request[:3] != (result['op'], result['target_id'], result['session']):
                return
            key = (result['boot_id'], result['request_seq'])
            if key in self.results:
                return  # Replayed operation results never refresh their arrival time.
            self.results[key] = (result, self.clock())
            if len(self.results) > 64:
                del self.results[next(iter(self.results))]
            self.condition.notify_all()

    def home_fresh(self):
        return bool(self._home_snapshot and self._home_observed_at is not None
                    and 0 <= self.clock()-self._home_observed_at < FRESH_S and self.fresh()
                    and self._home_snapshot['boot_id'] == self.snapshot['boot_id'])

    def home_actual_off(self):
        packet = self.snapshot
        return bool(self.fresh() and packet['state'] == W.IDLE
                    and packet['flags'] & W.POLICY_LOCKED and packet['flags'] & W.BOOT_READY
                    and packet['flags'] & W.ALL_TORQUE_OFF_VERIFIED
                    and not packet['flags'] & (W.TORQUE_MAY_BE_ON | W.SESSION_ACTIVE | W.CAPTURE_BUSY)
                    and all(row['torque_off_verified'] for row in self.rows()))

    def home_released(self):
        packet = self._home_snapshot
        current, receipt = self._home_session, self._home_idle_release
        if current is not None:
            # An old, freshly retransmitted released snapshot is still old
            # evidence. It cannot cancel a newer prepare waiting in the queue.
            requests = {value for value in (current.get('prepare_request'), current.get('run_request'))
                        if type(value) is int and value > 0}
            if (not packet or packet['boot_id'] != current['boot']
                    or packet['operation_ack_seq'] not in requests
                    or packet['native_sha256'] != current['sha']
                    or packet['targets'] != current['targets'] or packet['modes'] != current['modes']):
                return False
        elif receipt is not None:
            normal_off_ack = bool(self.snapshot and self.snapshot['boot_id'] == receipt['boot']
                                  and self.snapshot['operation_ack_seq'] == receipt['request']
                                  and self.snapshot['last_received_req_seq'] == receipt['request']
                                  and self.snapshot['op'] == W.OFF_ALL and self.snapshot['result'] == W.OK)
            if (not packet or receipt['generation'] != self.generation
                    or packet['boot_id'] != receipt['boot']
                    or packet['phase'] != 'off' and packet['operation_ack_seq'] != receipt['request'] and not normal_off_ack):
                return False
        return bool(self.home_idle_released() or self.home_fresh() and packet['phase'] in ('off','done','fault')
                    and packet['off_mask'] == W.ALL_MASK and not packet['session']
                    and self.home_actual_off())

    def home_idle_released(self):
        """Recognize this STOP's ordinary-engine ACK when HomeEngine never ran.

        HOME_OFF has no HomeEngine torque reads and therefore off_mask == 0.
        Only a matching, current-generation OFF_ALL receipt can bridge that gap;
        a new home prepare invalidates this receipt before it is sent.
        """
        receipt, packet, home = self._home_idle_release, self.snapshot, self._home_snapshot
        return bool(receipt and self._home_session is None and self._session is None
                    and receipt['generation'] == self.generation
                    and self.home_fresh() and self.home_actual_off()
                    and packet['boot_id'] == receipt['boot'] == home['boot_id']
                    and packet['operation_ack_seq'] == receipt['request']
                    and packet['last_received_req_seq'] == receipt['request']
                    and packet['op'] == W.OFF_ALL and packet['result'] == W.OK
                    and home['phase'] == 'off' and not home['session']
                    and not home['target_mask'] and home['native_sha256'] == '0' * 64
                    # OFF_ALL ACK proves this request's fifteen TORQUE_READs.
                    # The combined metadata age also includes the cached MODE,
                    # whose existing validity budget remains 750 ms.
                    and all(row['valid'] and row['torque_off_verified'] for row in self.rows()))

    def home_reconcile(self):
        if self.home_released():
            self._home_session = None; self._home_unknown = False
            if isinstance(self.operation, str) and self.operation.startswith('home_'):
                self.operation = None

    @staticmethod
    def home_progress_valid(packet):
        """ABI3 masks describe one fixed-prefix sequence, never fifteen-way enable."""
        if not isinstance(packet, dict) or packet.get('version') != W.HOME_CONTRACT_VERSION:
            return False
        phase, stage, active = packet['phase'], packet['stage'], packet['active_id']
        enabled, primed, arrived = packet['enabled_mask'], packet['primed_mask'], packet['arrived_mask']
        if phase in ('off', 'done', 'fault', 'releasing'):
            return True  # Fault diagnostics retain the exact failed ID/stage.
        if enabled & packet['off_mask']:
            return False
        if phase == 'preparing':
            return active == 0 and stage in (1, 2, 3) and enabled == primed == arrived == 0
        if packet['fresh_mask'] != W.ALL_MASK:
            return False
        if phase == 'prepared':
            return (active == 0 and stage == 0 and enabled == primed == arrived == 0
                    and packet['off_mask'] == W.ALL_MASK and all(t == 0 for t in packet['torques']))
        if phase == 'holding':
            return (active == 0 and stage == 9 and enabled == primed == arrived == W.ALL_MASK
                    and packet['result'] == W.OK and packet['error'] == 0
                    and packet['goals'] == packet['targets'] and all(t == 1 for t in packet['torques'])
                    and len(packet['positions']) == 15
                    and all(type(p) is int and -32767 <= p <= 32767 and abs(p-g) <= 32
                            for p,g in zip(packet['positions'],packet['targets'])))
        if active not in W.MOTOR_IDS:
            return False
        index = W.MOTOR_IDS.index(active)
        complete, powered = (1 << index)-1, (1 << (index+1))-1
        if arrived != complete or primed not in (complete, powered):
            return False
        if any(packet['torques'][i] != 1 for i in range(15) if enabled & (1 << i)):
            return False
        if phase == 'enabling':
            return (stage in (1, 2, 3) and enabled == primed == complete
                    or stage in (4, 5) and enabled in (complete, powered)
                    or stage == 6 and enabled == powered)
        return phase == 'moving' and stage in (7, 8) and enabled == powered

    def home_holding_verified(self, current):
        packet = self._home_snapshot
        arrived = self._home_arrival
        prior_same_run = bool(current and arrived and arrived['boot_id'] == current['boot']
                              and arrived['native_sha256'] == current['sha']
                              and arrived['run_request'] == current.get('run_request'))
        return bool(current and current.get('run_requested') and self.home_fresh()
                    and packet['phase'] == 'holding' and self.home_progress_valid(packet)
                    and packet['operation_ack_seq'] == current.get('run_request')
                    and packet['boot_id'] == current['boot'] and packet['session'] == current['nonce']
                    and packet['native_sha256'] == current['sha'] and packet['targets'] == current['targets']
                    and packet['modes'] == current['modes']
                    # First arrival observes all fifteen within3 after the MCU's
                    # continuous300ms all-joint gate. Later HOLD is historical
                    # arrival plus current MCU tracking (6/32 ticks), not a new
                    # claim that every present sample remains within3.
                    and (prior_same_run or all(abs(p-g) <= 3 for p,g in zip(packet['positions'],packet['targets']))))

    def receive_home(self, payload):
        try:
            packet = W.decode_home_snapshot(payload)
            if not self.home_progress_valid(packet):
                raise ValueError("invalid ABI3 home progress")
        except (ValueError, TypeError):
            with self.condition:
                self.errors.append({'code': 'invalid_home_snapshot'})
                if self._home_session or self._home_unknown:
                    self.home_stop_send()
                self.condition.notify_all()
            return
        with self.condition:
            if not self.snapshot or packet['boot_id'] != self.snapshot['boot_id']:
                return
            previous = self._home_snapshot
            if previous and packet['boot_id'] == previous['boot_id']:
                step = W.elapsed(packet['sequence'], previous['sequence'])
                if step == 0 or step >= 0x80000000 or W.elapsed(packet['sample_us'], previous['sample_us']) >= 0x80000000:
                    return
            self._home_snapshot, self._home_observed_at = packet, self.clock()
            current = self._home_session
            if (current and packet['error'] and packet['phase'] in ('releasing','fault')
                    and packet['boot_id'] == current['boot'] and packet['native_sha256'] == current['sha']
                    and packet['targets'] == current['targets'] and packet['modes'] == current['modes']
                    and packet['operation_ack_seq'] in {current.get('prepare_request'), current.get('run_request')}):
                # Preserve the first matching MCU cause before release clears nonce
                # and before a subsequent STOP changes its operation ACK.
                current.setdefault('mcu_failure', {'error': packet['error'], 'phase': packet['phase'],
                    'request': packet['operation_ack_seq'], 'active_id': packet['active_id'],
                    'stage': packet['stage'], 'stage_name': packet['stage_name']})
            if packet['error']:
                key = (packet['boot_id'], packet['operation_ack_seq'], packet['native_sha256'])
                fault = next((item for item in self._home_faults if
                              (item['boot_id'], item['request_seq'], item['native_sha256']) == key), None)
                if fault is None:
                    fault = {'boot_id': key[0], 'request_seq': key[1], 'native_sha256': key[2],
                             'first_observed_error': packet['error'], 'first_observed_phase': packet['phase'],
                             'first_observed_source_us': packet['sample_us'],
                             'active_id': packet['active_id'], 'execution_stage': packet['stage_name'],
                             'stage': 'prepare' if current and key[1] == current.get('prepare_request')
                             else 'run' if current and key[1] == current.get('run_request') else 'unknown'}
                    fault['first_feedback'] = {
                        'sequence': packet['sequence'], 'sample_us': packet['sample_us'],
                        'fresh_mask': packet['fresh_mask'], 'steps': packet['steps'],
                        **{name: list(packet[name]) for name in ('positions', 'goals', 'targets', 'torques')}}
                    if packet['active_id'] in W.MOTOR_IDS:
                        index = W.MOTOR_IDS.index(packet['active_id'])
                        position = packet['positions'][index]
                        fault['goal_minus_position_ticks'] = (
                            packet['goals'][index] - position
                            if packet['fresh_mask'] & (1 << index) and position != -32768 else None)
                    self._home_faults.append(fault)
                fault.update(last_observed_error=packet['error'], last_observed_phase=packet['phase'],
                             last_observed_source_us=packet['sample_us'], off_mask=packet['off_mask'],
                             fresh_mask=packet['fresh_mask'], enabled_mask=packet['enabled_mask'])
            if current and current['confirmed'] and self._home_arrival is None and self.home_holding_verified(current):
                self._home_arrival = {'boot_id': packet['boot_id'], 'native_sha256': packet['native_sha256'],
                                      'source_us': packet['sample_us'], 'sequence': packet['sequence'],
                                      'run_request': packet['operation_ack_seq'], 'tolerance_ticks': 3,
                                      'continuous_stable_ms': 300, 'historical': True} 
            if (packet['phase'] in ('preparing','prepared','enabling','moving','holding','releasing')
                    or packet['phase'] == 'fault' and not self.home_released()):
                self._home_unknown = True
                self.operation = 'home_'+packet['phase']
            self.home_reconcile()
            self.condition.notify_all()

    def home_turn_reference(self, summary):
        reference, instance = self._home_turn_reference, self._home_turn_instance
        if not reference or not isinstance(instance, str) or len(instance) != 32:
            return None
        audit = reference['audit']
        if (getattr(self.telemetry, 'app_instance_id', None) != instance or not self.snapshot
                or summary.get('verified') is not True or self._group_summary.get('verified') is not True
                or summary.get('group_records_sha256') != self._group_summary.get('sha256')):
            return None
        for sid, axis in audit['axes'].items():
            saved = self._native_published_records.get(sid)
            if not saved or hashlib.sha256(ZeroStore._json_bytes(saved)).hexdigest() != axis['saved_record_sha256']:
                return None
        return reference

    def home_plan(self, summary=None, records=None):
        summary = self._native_summary if summary is None else summary
        records = self._native_published_records if records is None else records
        blocked, targets, modes, spans, saved_targets, turn_offsets = [], [], [], [], [], []
        certified = bool(summary.get('verified') and summary.get('path') == 'data/zero_pos.native.json'
                         and self._group_summary.get('verified')
                         and summary.get('group_records_sha256') == self._group_summary.get('sha256')
                         and len(records) == 15)
        rows = self.rows()
        reference = self.home_turn_reference(summary) if certified else None
        axes = reference['audit']['axes'] if reference else {}
        for row in rows:
            saved = records.get(str(row['id']), {})
            saved_target, mode = saved.get('position'), saved.get('mode')
            target, turn_offset = resolve_home_target(saved_target, row['position'], mode,
                                                       axes.get(str(row['id'])), row['id'])
            good = (certified and row['valid'] and row['torque_off_verified']
                    and row['metadata_age_ms'] is not None and row['metadata_age_ms'] <= 750
                    and type(target) is int and mode in POSITION_MODES and row['mode'] == mode
                    and capture_position(mode, saved.get('raw_word'), saved.get('signed_steps'))
                    and saved.get('signed_steps') == saved_target and home_position_allowed(row['id'],row['position'],target))
            if not good:
                blocked.append(row['id'])
            targets.append(target); modes.append(mode)
            saved_targets.append(saved_target); turn_offsets.append(turn_offset)
            span_target = target if type(target) is int else saved_target
            if type(span_target) is int and type(row['position']) is int:
                spans.append(abs(span_target-row['position']))
        ready = bool(not blocked and self.home_fresh() and self.home_actual_off()
                     and not self.operation and not self._session and not self._home_session and not self._home_unknown)
        plan = {'boot_id': self.snapshot['boot_id'] if self.snapshot else None,
                'native_sha256': summary.get('sha256'), 'max_span_ticks': max(spans) if spans else None,
                'targets': targets, 'modes': modes, 'saved_targets': saved_targets,
                'turn_offsets': turn_offsets, 'target_resolution': HOME_TARGET_RESOLUTION,
                'range_profile': 'native-head-recovery-v2',
                'turn_equivalence_verified_ids': sorted(int(sid) for sid in axes),
                'turn_reference': {'sha256': reference['sha256'], 'boot_id': self.snapshot['boot_id'],
                                   'source_boot_id': reference['audit']['source_boot_id'],
                                   'native_sha256': summary.get('sha256'),
                                   'app_instance_id': self._home_turn_instance} if reference else None,
                'session_locked': False}
        current = self._home_session
        if current:
            # Public status describes this exact PREPARE, never a retargeted turn.
            for key in ('targets', 'modes', 'saved_targets', 'turn_offsets'):
                if key in current:
                    plan[key] = list(current[key])
            plan['max_span_ticks'] = current.get('max_span_ticks')
            plan['boot_id'], plan['native_sha256'] = current['boot'], current['sha']
            plan['session_locked'] = True
        return ready, blocked, plan

    def home_status(self):
        ready, blocked, plan = self.home_plan()
        packet, current = self._home_snapshot, self._home_session
        phase = packet['phase'] if packet else 'off'
        active = bool(self._home_unknown or current or packet and packet['phase'] in
                      ('preparing','prepared','enabling','moving','holding','releasing'))
        released = self.home_released()
        if released:
            active = False
        holding = self.home_holding_verified(current)
        return {'supported': self.home_fresh(), 'ready': ready, 'blocked_ids': blocked, 'plan': plan,
                'reason': '' if ready else '需要完整零位、同模式、新鲜卸力反馈及关节允许角度；颈部可从半圈内朝原中位缓慢收回，头俯仰±105°；其他轴仍限1024步。',
                'phase': phase, 'active': active,
                'expires_ms': None if holding else max(0, int((current['deadline']-self.clock())*1000)) if current else 0,
                'arrival_verified': bool(self.home_fresh() and self._home_arrival
                                         and self._native_summary.get('verified')
                                         and self._group_summary.get('verified')
                                         and self._native_summary.get('group_records_sha256') == self._group_summary.get('sha256')
                                         and self._home_arrival['native_sha256'] == self._native_summary.get('sha256')
                                         and self._home_arrival['boot_id'] == packet['boot_id']
                                         and self._home_arrival['native_sha256'] == packet['native_sha256']),
                'arrival_evidence': dict(self._home_arrival) if self._home_arrival else None,
                'release_verified': released, 'native_sha256': packet['native_sha256'] if packet else None,
                'error': packet['error'] if packet else None, 'contract_revision': W.HOME_CONTRACT_VERSION,
                'motion_limit_ms': int(HOME_MOTION_S*1000),
                'progress': {key: packet[key] for key in ('active_id','stage','stage_name','arrived_mask','enabled_mask','primed_mask','fresh_mask','off_mask')} if packet else None,
                'feedback': {key: packet[key] for key in ('sequence','sample_us','active_id','stage_name','positions','goals','targets','torques','steps')} if packet else None,
                'fault_history': [dict(item) for item in self._home_faults],
                'cancellation_history': [dict(item) for item in self._home_cancellations],
                'lease_stage': ('hold' if holding else 'motion' if current.get('run_requested') else 'prepare') if current else None,
                'physical_acceptance_verified': False}

    def home_send(self, op, current=None, prepare=False):
        if not self.snapshot:
            raise CalibrationError('mcu_unknown', '尚无主控反馈。', 503)
        self.request_seq = (self.request_seq+1) & 0xffffffff
        if not self.request_seq:
            self.request_seq = 1
        request = self.request_seq
        if prepare:
            self._home_idle_release = None
            current['prepare_request'] = request
            payload = W.encode_home_command(current['targets'], current['modes'], current['sha'],
                      current['boot'], request, current['nonce'], self.snapshot['sample_us'])
        else:
            payload = W.encode_home_control(op, self.snapshot['boot_id'], request,
                      current['nonce'] if current else 0, self.snapshot['sample_us'])
            if op == W.HOME_RUN and current is not None:
                current['run_request'] = request
        if not prepare and op == W.OFF_ALL:
            # Set before queueing: a fast fake/real ACK may arrive immediately.
            self._home_idle_release = {'boot': self.snapshot['boot_id'], 'request': request,
                                       'generation': self.generation}
            self.requests[request] = (op, 0, current['nonce'] if current else 0, 0)
            if len(self.requests) > 128:
                del self.requests[next(iter(self.requests))]
        self.sender.put(request, op, payload, self.generation)
        self.sent += 1
        return request

    def home_stop_send(self, reason=None):
        current, self._home_session = self._home_session, None
        if reason is not None and current is not None:
            now = self.clock()
            self._home_cancellations.append({
                'reason': reason, 'boot_id': current['boot'],
                'prepare_request': current.get('prepare_request'), 'run_request': current.get('run_request'),
                'lease_stage': 'motion' if current.get('run_requested') else 'prepare',
                'observed_home_phase': self._home_snapshot['phase'] if self._home_snapshot else None,
                'client_age_ms': round(max(0, now-current['client_at'])*1000, 2),
                'feedback_age_ms': round(max(0, now-self.observed_at)*1000, 2) if self.observed_at is not None else None,
                'home_feedback_age_ms': round(max(0, now-self._home_observed_at)*1000, 2) if self._home_observed_at is not None else None,
                'hard_deadline_remaining_ms': max(0, int((current['deadline']-now)*1000))})
        self._home_unknown = True; self.operation = 'home_releasing'
        self.generation += 1; self.condition.notify_all()
        try:
            return self.home_send(W.OFF_ALL, current)
        except Exception:
            self.errors.append({'code': 'home_release_unconfirmed'})
            return None

    def home_expiry_reason(self, current):
        if self.clock() >= current['deadline'] and not self.home_holding_verified(current):
            return 'hard_deadline'
        # PREPARE is strictly read-only. The 400 ms motion-client lease starts
        # only when the first explicit RUN is authenticated, never while the
        # PREPARE reply is still making its network round trip.
        if current.get('run_requested') and self.clock()-current['client_at'] > CLIENT_LEASE_S:
            return 'client_lease'
        if not self.fresh():
            return 'feedback_stale'
        if self.snapshot['boot_id'] != current['boot']:
            return 'source_identity'
        if current.get('turn_reference_sha256'):
            reference = self.home_turn_reference(self._native_summary)
            if (not reference or reference['sha256'] != current['turn_reference_sha256']
                    or getattr(self.telemetry, 'app_instance_id', None) != current.get('host_instance_id')
                    or self._group_summary.get('sha256') != current.get('group_sha256')):
                return 'home_turn_reference_identity'
        if current.get('confirmed'):
            if not self.home_fresh():
                return 'home_feedback_stale'
            if self._home_snapshot['session'] != current['nonce']:
                return 'home_session_identity'
            if (self._home_snapshot['native_sha256'] != current['sha']
                    or self._home_snapshot['targets'] != current['targets']
                    or self._home_snapshot['modes'] != current['modes']):
                return 'home_vector_identity'
            if not current.get('run_requested') and (
                    self._home_snapshot['phase'] != 'prepared'
                    or self.snapshot['flags'] & W.TORQUE_MAY_BE_ON
                    or self._home_snapshot['enabled_mask'] != 0):
                return 'unrequested_motion_phase'
        return None

    def home_session(self, body):
        current = self._home_session
        if not current or not isinstance(body.get('session'), str) or not hmac.compare_digest(body['session'], current['token']):
            raise CalibrationError('session_mismatch', '归位授权不匹配。')
        reason = self.home_expiry_reason(current)
        if reason is not None:
            self.home_stop_send(reason)
            raise CalibrationError('session_expired', '归位授权已失效，正在卸力。')
        return current

    def home_wait(self, request, current, phases, timeout=1.5):
        deadline = time.monotonic()+timeout
        with self.condition:
            while True:
                failure = current.get('mcu_failure') if current else None
                if failure is not None:
                    operation_stage = '准备' if failure['request'] == current.get('prepare_request') else '执行'
                    raise CalibrationError('home_rejected', '归位'+operation_stage+' MCU 错误 '+str(failure['error'])+'；ID '+str(failure['active_id'])+'，阶段 '+failure['stage_name']+'，卸力确认另行检查。', ids=[failure['active_id']] if failure['active_id'] in W.MOTOR_IDS else [])
                if request in self.failed_requests:
                    self.raise_delivery_error(request)
                packet = self._home_snapshot
                # receive_home may already reconcile this exact operation's
                # verified release. Preserve its MCU failure before treating a
                # missing owner as an unrelated cancellation.
                if (current is not None and self.home_fresh() and packet['operation_ack_seq'] == request
                        and packet['boot_id'] == current['boot'] and packet['native_sha256'] == current['sha']
                        and packet['targets'] == current['targets'] and packet['modes'] == current['modes']
                        and packet['error'] and packet['phase'] in ('releasing','fault')):
                    stage = '准备' if request == current.get('prepare_request') else '执行'
                    raise CalibrationError('home_rejected', '归位'+stage+'未完成，MCU错误 '+str(packet['error'])+'。')
                if current is not None and self._home_session is not current:
                    raise CalibrationError('cancelled', '归位已取消。')
                if (self.home_fresh() and packet['operation_ack_seq'] == request):
                    if current is None and packet['phase'] in phases and self.home_released():
                        # An explicit STOP succeeds on this request's complete
                        # release evidence, even while its original fault stays visible.
                        return packet
                    if packet['phase'] in phases and packet['result'] == W.OK:
                        # DHS is notified before DWR, and phase changes may send DHS
                        # without DWR. Wait for the matching source generation before
                        # checking its metadata age; never validate fresh DHS against
                        # an earlier idle DWR cache or relax the 150 ms safety limit.
                        if current and packet['phase'] == 'prepared' and (
                                self.snapshot['boot_id'] != packet['boot_id']
                                or self.snapshot['session'] != packet['session']
                                or self.snapshot['operation_ack_seq'] != packet['operation_ack_seq']
                                or self.snapshot['state'] != W.PREFLIGHT
                                or W.elapsed(self.snapshot['sequence'], packet['sequence']) >= 0x80000000):
                            remaining = deadline-time.monotonic()
                            if remaining <= 0:
                                raise CalibrationError('home_timeout', '归位准备的同源反馈未配齐；未启动运动。', 503)
                            self.condition.wait(min(.025, remaining))
                            continue
                        if current is None and not self.home_released():
                            raise CalibrationError('release_unconfirmed', '卸力读回未确认；保持支撑。')
                        if current and (packet['targets'] != current['targets'] or packet['modes'] != current['modes']
                                        or packet['fresh_mask'] != W.ALL_MASK
                                        or not self.home_progress_valid(packet)):
                            raise CalibrationError('home_readback', '归位目标或全部读回证据不符。')
                        if current and packet['phase'] == 'prepared' and (packet['primed_mask'] != 0
                                        or packet['off_mask'] != W.ALL_MASK or packet['enabled_mask'] != 0
                                        or any(torque != 0 for torque in packet['torques'])
                                        or any(not row['torque_off_verified'] or row['metadata_age_ms'] >= 150 for row in self.rows())
                                        or self.snapshot['flags'] & W.TORQUE_MAY_BE_ON):
                            raise CalibrationError('home_prime', '卸力目标读回不符。')
                        if current and packet['phase'] in ('moving','holding') and not self.home_progress_valid(packet):
                            raise CalibrationError('home_enable', '逐颗上力顺序或到位保持证据不符。')
                        if current and (packet['session'] != current['nonce'] or packet['native_sha256'] != current['sha']):
                            raise CalibrationError('home_identity', '归位读回身份不符。')
                        return packet
                    if packet['result'] == W.ERROR:
                        raise CalibrationError('home_rejected', '归位被拒绝，错误 '+str(packet['error'])+'。')
                if (current is None and self.home_released() and self.snapshot['operation_ack_seq'] == request
                        and self.snapshot['last_received_req_seq'] == request
                        and self.snapshot['op'] == W.OFF_ALL and self.snapshot['result'] == W.OK):
                    return None
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise CalibrationError('home_timeout', '归位未确认；不自动重试。', 503)
                self.condition.wait(min(.025, remaining))

    def home_verify_file(self, sha):
        summary, records = self.store.read_native()
        group, _ = self.store.read_groups()
        if (not isinstance(sha, str) or len(sha) != 64 or not summary.get('verified') or not group.get('verified')
                or summary.get('sha256') != sha or summary.get('group_records_sha256') != group.get('sha256')
                or summary.get('sha256') != self._native_summary.get('sha256')
                or group.get('sha256') != self._group_summary.get('sha256')):
            raise CalibrationError('native_changed', '零位审计文件改变或未验证；未启动运动。')
        return summary, records

    def discard_unsent_home_prepare(self, current, error):
        """Cancel only a provably undelivered prepare, never an uncertain move."""
        route_missing = (isinstance(error, CalibrationError)
                         and error.code == 'command_route_unavailable'
                         and self.delivery_errors.get(current.get('prepare_request')) == 'command_route_unavailable')
        queue_full = isinstance(error, CalibrationError) and error.code == 'transport_busy'
        packet = self._home_snapshot
        if (not (route_missing or queue_full) or self._home_session is not current
                or current.get('confirmed') or not self.home_fresh() or not self.home_actual_off()
                or packet['phase'] != 'off' or packet['session'] or packet['target_mask']
                or packet['native_sha256'] != '0'*64):
            return False
        self._home_session = None; self._home_unknown = False; self.operation = None
        self._home_idle_release = None; self._home_heartbeat = None
        self.generation += 1; self.condition.notify_all()
        return True

    def home_context_current(self, current):
        return bool(self._home_session is current and self.snapshot
                    and self.snapshot['boot_id'] == current['boot']
                    and self.generation == current['generation'])

    def home_action(self, body):
        op = body.get('op')
        if op not in ('home_prepare','home_run','home_heartbeat','home_stop') or body.get('id', 0) != 0:
            raise CalibrationError('invalid_operation', '无效归位操作。', 400)
        if op != 'home_stop' and (type(body.get('home_contract_revision')) is not int or body['home_contract_revision'] != W.HOME_CONTRACT_VERSION):
            raise CalibrationError('home_contract_mismatch', '慢速回中需要新版页面与 HOME 协议5。', 400)
        if op == 'home_stop':
            with self.lock:
                request = self.home_stop_send()
            if request is None:
                raise CalibrationError('release_unconfirmed', '卸力未确认；保持支撑。', 503)
            self.home_wait(request, None, ('done','fault'))
            with self.lock:
                if not self.home_released():
                    raise CalibrationError('release_unconfirmed', '卸力读回未确认；保持支撑。')
                self.home_reconcile()
            return {'ok': True, 'op': op, 'status': self.calibration_status()}
        if op == 'home_heartbeat':
            with self.lock:
                current = self.home_session(body)
                current['client_at'] = self.clock()
            return {'ok': True, 'op': op, 'expires_ms': max(0,int((current['deadline']-self.clock())*1000)), 'status': self.calibration_status()}
        if op == 'home_prepare':
            if body.get('supported') is not True or body.get('clear') is not True:
                raise CalibrationError('confirmation_required', '先托稳并确认全部关节无碰撞。')
            summary, records = self.home_verify_file(body.get('native_sha256'))
            with self.lock:
                self.require_idle(); self.require_fresh()
                ready, blocked, plan = self.home_plan(summary, records)
                if not ready:
                    raise CalibrationError('home_not_ready', '归位前读回或跨度未通过。', ids=blocked)
                now = self.clock()
                current = {'token': secrets.token_hex(24), 'nonce': secrets.randbelow(0xffffffff)+1,
                           'boot': plan['boot_id'], 'sha': plan['native_sha256'], 'targets': list(plan['targets']),
                           'saved_targets': list(plan['saved_targets']), 'turn_offsets': list(plan['turn_offsets']),
                           'turn_reference_sha256': (plan.get('turn_reference') or {}).get('sha256'),
                           'host_instance_id': self._home_turn_instance,
                           'group_sha256': self._group_summary.get('sha256'),
                           'max_span_ticks': plan['max_span_ticks'],
                           'modes': plan['modes'], 'deadline': now+SESSION_S, 'client_at': now, 'confirmed': False,
                           'run_requested': False,
                           'generation': self.generation}
                self._home_session = current; self._home_unknown = True; self._home_arrival = None; self.operation = 'home_preparing'
                try:
                    request = self.home_send(0, current, prepare=True)
                except Exception as error:
                    if self.home_context_current(current) and not self.discard_unsent_home_prepare(current, error):
                        self.home_stop_send()
                    raise
            try:
                self.home_wait(request, current, ('prepared',))
                with self.lock:
                    if not self.home_context_current(current):
                        raise CalibrationError('cancelled','归位请求已取消或主控已重启。')
                    current['confirmed'] = True
                    return {'ok': True, 'op': op, 'session': current['token'],
                            'expires_ms': max(0,int((current['deadline']-self.clock())*1000)), 'status': self.calibration_status()}
            except Exception as error:
                with self.lock:
                    if self.home_context_current(current) and not self.discard_unsent_home_prepare(current, error):
                        self.home_stop_send()
                raise
        with self.lock:
            current = self.home_session(body)
            if self._home_snapshot['phase'] != 'prepared':
                raise CalibrationError('home_not_prepared', '先准备并核对卸力目标读回。')
            if current.get('run_requested'):
                raise CalibrationError('home_run_already_requested', '本次归位执行已提交，不重复发送。')
            current['run_requested'] = True
            current['client_at'] = self.clock()
            current['deadline'] = self.clock()+HOME_MOTION_S
        try:
            self.home_verify_file(body.get('native_sha256'))
        except Exception:
            with self.lock:
                if self.home_context_current(current):
                    self.home_stop_send('run_verification_failed')
            raise
        with self.lock:
            current = self.home_session(body)
            if body.get('native_sha256') != current['sha']:
                raise CalibrationError('native_changed', '归位文件身份不匹配。')
            self.operation = 'home_enabling'
            request = self.home_send(W.HOME_RUN, current)
        try:
            self.home_wait(request, current, ('moving','holding'))
            with self.lock:
                if not self.home_context_current(current):
                    raise CalibrationError('cancelled','归位请求已取消或主控已重启。')
                return {'ok': True, 'op': op, 'session': current['token'],
                        'expires_ms': max(0,int((current['deadline']-self.clock())*1000)), 'status': self.calibration_status()}
        except Exception:
            with self.lock:
                if self.home_context_current(current):
                    self.home_stop_send()
            raise

    def home_tick(self):
        with self.lock:
            self.home_reconcile()
            current = self._home_session
            if not current:
                return
            reason = self.home_expiry_reason(current)
            if reason is None and current['confirmed']:
                if (self._home_snapshot['native_sha256'] != current['sha']
                        or self._home_snapshot['targets'] != current['targets']
                        or self._home_snapshot['modes'] != current['modes']):
                    reason = 'home_vector_identity'
                elif self._home_snapshot['phase'] not in ('prepared','enabling','moving','holding'):
                    reason = 'home_phase'
            if reason is not None:
                self.home_stop_send(reason)
            elif self._home_heartbeat is None or self.clock()-self._home_heartbeat >= HEARTBEAT_S:
                try:
                    self.home_send(W.HEARTBEAT, current)
                    self._home_heartbeat = self.clock()
                except Exception:
                    self.home_stop_send()

    def fresh(self):
        return bool(not self.protocol_fault and self.snapshot and self.observed_at is not None and 0 <= self.clock()-self.observed_at < FRESH_S)

    def rows(self, packet=None, current=True):
        packet = packet or self.snapshot
        elapsed_ms = max(0, (self.clock()-self.observed_at)*1000) if current and self.observed_at is not None else 0
        result = []
        for i, sid in enumerate(W.MOTOR_IDS):
            row = packet['joints'][i] if packet else None
            pa = row['position_age_ms']+elapsed_ms if row and row['position_age_ms'] != 65535 else None
            ma = row['metadata_age_ms']+elapsed_ms if row and row['metadata_age_ms'] != 65535 else None
            pf = bool(row and packet['position_mask'] & (1<<i) and pa is not None and pa < 150 and (not current or self.fresh()))
            mf = bool(row and packet['metadata_mask'] & (1<<i) and ma is not None and ma <= 750 and (not current or self.fresh()))
            mode = row['mode'] if mf else None
            torque = bool(row['torque']) if mf and row['torque'] in (0, 1) else None
            pos = row['signed_steps'] if pf and capture_position(mode, row['raw_word'], row['signed_steps']) else None
            reason = ('位置反馈过期' if not pf else '模式或扭矩状态读回过期' if not mf else '非受支持位置模式（0/4）' if not position_mode(mode)
                      else '编码待核对' if pos is None else '')
            code = ('stale_position' if not pf else 'stale_metadata' if not mf else 'invalid_mode' if not position_mode(mode)
                    else 'invalid_position' if pos is None else '')
            result.append({'id': sid, 'raw_word': row['raw_word'] if row else None,
                           'signed_steps': row['signed_steps'] if row else None, 'position': pos,
                           'mode': mode, 'torque_enabled': torque, 'position_age_ms': pa, 'metadata_age_ms': ma,
                           'fresh': pf and mf, 'stable': bool(row and packet['stable_mask'] & (1<<i)),
                           'valid': not reason, 'data_valid': not reason, 'coordinate_domain': coordinate_domain(mode),
                           'motion_valid': not reason and single_turn_position(row['raw_word'], row['signed_steps']),
                           'motion_blocked_reason': reason or ('原生多圈编码仅可记录；小步暂限0..4095。' if not single_turn_position(row['raw_word'], row['signed_steps']) else ''),
                           'blocked_reason': reason, 'blocked_code': code,
                           'torque_off_verified': mf and torque is False, 'sign': SIGNS[sid]})
        return result

    def stable_window(self, ids=W.MOTOR_IDS, request=None, op=None, *, _rows_cache=None):
        if not self.fresh():
            return None
        newest = self.history[-1][0]
        expected_mask = sum(1 << W.MOTOR_IDS.index(sid) for sid in ids)
        window = []
        for packet, received in reversed(self.history):
            span = W.elapsed(newest['sample_us'], packet['sample_us'])
            if packet['boot_id'] != newest['boot_id'] or span > 800000:
                break
            # A caller-local cache lives only while this status request holds the lock.
            # Action/capture callers do not pass it and keep their original evaluation.
            rows = _rows_cache.get(id(packet)) if _rows_cache is not None else None
            if rows is None:
                rows = self.rows(packet, False)
                if _rows_cache is not None:
                    _rows_cache[id(packet)] = rows
            if (not packet['flags'] & W.POLICY_LOCKED or packet['state'] == W.FAULT
                    or not packet['flags'] & W.ALL_TORQUE_OFF_VERIFIED
                    or packet['flags'] & W.TORQUE_MAY_BE_ON
                    or any(not row['torque_off_verified'] for row in rows)
                    or any(not row['valid'] for row in rows if row['id'] in ids)):
                break
            if request is not None and (packet['operation_ack_seq'] != request
                    or packet['last_received_req_seq'] != request or packet['op'] != op
                    or packet['result'] != W.OK or packet['capture_mask'] != expected_mask):
                break
            if window and W.elapsed(window[-1]['sample_us'], packet['sample_us']) > 150000:
                break
            window.append(packet)
            if span >= STABLE_US:
                break
        if len(window) < 3 or W.elapsed(newest['sample_us'], window[-1]['sample_us']) < STABLE_US:
            return None
        indexes = [W.MOTOR_IDS.index(sid) for sid in ids]
        spread = [max(p['joints'][i]['signed_steps'] for p in window)-min(p['joints'][i]['signed_steps'] for p in window) for i in indexes]
        if max(spread) > MAX_SPREAD:
            return None
        return {'samples': len(window), 'duration_ms': W.elapsed(newest['sample_us'], window[-1]['sample_us'])/1000,
                'ids': list(ids), 'spread_ticks': spread,
                'snapshot_payloads_hex': [p['raw_payload_hex'] for p in window]}

    def active(self):
        current = self._session
        return bool(current and self.clock() < current['deadline']
                    and self.clock()-current['client_at'] <= CLIENT_LEASE_S and self.fresh()
                    and (not current['confirmed'] or (self.snapshot['session'] == current['nonce']
                         and self.snapshot['active_id'] == current['id']
                         and self.snapshot['flags'] & W.SESSION_ACTIVE
                         and self.snapshot['state'] in (W.ARMED, W.MOVING, W.VERIFY_OFF))))

    def calibration_status(self):
        with self.lock:
            packet, rows = self.snapshot, self.rows()
            rows_cache = {}  # No reuse across requests, freshness checks or mutations.
            blocked = [row['id'] for row in rows if not row['valid'] or not row['stable'] or not row['torque_off_verified']]
            locked = bool(self.fresh() and packet['flags'] & W.POLICY_LOCKED)
            home_busy = bool(self._home_session or self._home_unknown)
            ready = bool(locked and packet['flags'] & W.BOOT_READY and packet['flags'] & W.ALL_TORQUE_OFF_VERIFIED
                         and not packet['flags'] & (W.TORQUE_MAY_BE_ON | W.SESSION_ACTIVE | W.CAPTURE_BUSY)
                         and not blocked and self.stable_window(_rows_cache=rows_cache) and not self._session and not self.operation and not home_busy)
            file, native_file, records = dict(self._file_summary), dict(self._native_summary), {sid: dict(value) for sid, value in self._records.items()}
            global_blocked = [row['id'] for row in rows if not row['torque_off_verified']]
            groups = {}
            for name, (_, mask, ids) in W.GROUPS.items():
                selected_blocked = [row['id'] for row in rows if row['id'] in ids and (not row['valid'] or not row['stable'])]
                group_ready = bool(locked and packet['state'] == W.IDLE and packet['flags'] & W.BOOT_READY
                                   and packet['flags'] & W.ALL_TORQUE_OFF_VERIFIED
                                   and not packet['flags'] & (W.TORQUE_MAY_BE_ON | W.SESSION_ACTIVE | W.CAPTURE_BUSY)
                                   and not selected_blocked and not global_blocked and self.stable_window(ids, _rows_cache=rows_cache)
                                   and not self._session and not self.operation and not home_busy)
                groups[name] = {'ids': list(ids), 'ready': group_ready, 'blocked_ids': selected_blocked,
                                'global_torque_blocked_ids': global_blocked, 'recorded': name in self._groups,
                                'records': {str(sid): records[str(sid)] for sid in ids if str(sid) in records},
                                'user_pose_required': True, 'historical_records': True,
                                'reason': '' if group_ready else '先停止归位并等卸力读回，再重录本组零位。' if home_busy else '等待本组 5 颗有效稳定位置和全部 15 颗卸力读回。'}
            active = self.active()
            native_published = bool(len(records) == 15 and native_file.get('verified')
                                    and self._group_summary.get('verified')
                                    and native_file.get('group_records_sha256') == self._group_summary.get('sha256'))
            return {'schema': SCHEMA, 'version': VERSION, 'supported': True, 'policy_locked': locked, 'fresh': self.fresh(),
                    'boot_id': packet['boot_id'] if packet else None, 'sequence': packet['sequence'] if packet else None,
                    'age_ms': round((self.clock()-self.observed_at)*1000, 2) if self.observed_at is not None else None,
                    'home': self.home_status(), 'joints': rows, 'armed': {'active': active, 'id': self._session['id'] if active else None,
                                             'expires_ms': max(0, int((self._session['deadline']-self.clock())*1000)) if active else 0},
                    'capture': {'ready': ready, 'blocked_ids': blocked, 'user_pose_required': True,
                                'reason': '' if ready else '等待 15 颗新鲜、卸力、稳定位置；先对齐全部实物零姿态。'},
                    'records': records, 'captured_ids': [sid for sid in W.MOTOR_IDS if str(sid) in records],
                    'missing_ids': [sid for sid in W.MOTOR_IDS if str(sid) not in records], 'complete': len(records)==15,
                    'groups': groups, 'group_file': dict(self._group_summary),
                    'published': native_published, 'native_published': native_published,
                    'published_records': {sid: dict(value) for sid, value in self._published_records.items()},
                    'file': file, 'native_file': native_file, 'native_only': True, 'coordinate_domain': NATIVE_DOMAIN,
                    'model_deployment_ready': False,
                    'native_published_records': {sid: dict(value) for sid, value in self._native_published_records.items()},
                    'errors': [{'code': error['code'], 'ids': error.get('ids', []),
                                             'message': error.get('message', error['code'])} for error in self.errors],
                    'physical_acceptance_verified': False, 'policy_applied': False,
                    'mcu_state': packet['state'] if packet else None, 'mcu_flags': packet['flags'] if packet else None,
                    'torque_may_be_on': bool(packet['flags'] & W.TORQUE_MAY_BE_ON) if packet else None,
                    'all_torque_off_verified': bool(packet['flags'] & W.ALL_TORQUE_OFF_VERIFIED) if packet else None,
                    'boot_ready': bool(packet['flags'] & W.BOOT_READY) if packet else None,
                    'torque_enable_ever_attempted': bool(packet['flags'] & W.TORQUE_ENABLE_EVER_ATTEMPTED) if packet else None,
                    'session_active': bool(packet['flags'] & W.SESSION_ACTIVE) if packet else None,
                    'active_id': packet['active_id'] if packet else None,
                    'last_received_req_seq': packet['last_received_req_seq'] if packet else None,
                    'operation': self.operation, 'snapshot_protocol': 'DWR1' if packet else None,
                    'mcu_contract_revision': W.SNAPSHOT_VERSION if packet else None,
                    'snapshot_version': W.SNAPSHOT_VERSION if packet else None, 'sticky_supported': bool(packet),
                    'diagnostic': {'ready_at_snapshot_source': bool(packet and packet['flags'] & W.DIAG_READY),
                                   'historical': True, 'id': packet['diag_id'] if packet and packet['flags'] & W.DIAG_READY else None,
                                   'offset_steps': packet['diag_offset'] if packet and packet['flags'] & W.DIAG_READY else None,
                                   'angle_limit_min': packet['diag_min'] if packet and packet['flags'] & W.DIAG_READY else None,
                                   'angle_limit_max': packet['diag_max'] if packet and packet['flags'] & W.DIAG_READY else None},
                    'sign_source': 'official_runtime; installed directions unverified'}

    def status(self):
        result, calibration = self.telemetry.status(), self.calibration_status()
        channel_status = getattr(self.bridge, 'status', None)
        if callable(channel_status):
            result['command_transport'] = channel_status()
        result.update(mode='wireless_calibration', app_code_version=VERSION, host_read_only=False, no_rpc_writes=False,
                      sent=self.sender.attempted, queued_commands=self.sent,
                      command_tx_scope='Command dispatch attempts; RPC receipt is not DWR/DHS completion',
                      policy_locked=calibration['policy_locked'], calibration=calibration, control_ready=False,
                      commissioning_motion_locked=False,
                      mcu_motion_locked=False,
                      motion_lock_evidence='Policy disabled; bounded single-joint calibration uses a separate authorization.')
        for key in ('mcu_state','mcu_flags','torque_may_be_on','all_torque_off_verified','boot_ready',
                    'torque_enable_ever_attempted','last_received_req_seq','operation','session_active','active_id',
                    'mcu_contract_revision','snapshot_version','sticky_supported'):
            result[key] = calibration[key]
        with self.lock:
            result['servo_power'] = self.power_status()
        return result

    def live_status(self):
        """Copy live DQS/DWR evidence only; never serialize calibration history."""
        result = dict(self.telemetry.live_status())
        with self.lock:
            now = self.clock()
            packet, observed = self.snapshot, self.observed_at
            fresh = bool(not self.protocol_fault and packet and observed is not None and 0 <= now-observed < FRESH_S)
            flags = packet['flags'] if packet else None
            age = round((now-observed)*1000, 2) if observed is not None else None
            values = {'mcu_state': packet['state'] if packet else None, 'mcu_flags': flags,
                      'torque_may_be_on': bool(flags & W.TORQUE_MAY_BE_ON) if packet else None,
                      'all_torque_off_verified': bool(flags & W.ALL_TORQUE_OFF_VERIFIED) if packet else None,
                      'session_active': bool(flags & W.SESSION_ACTIVE) if packet else None,
                      'torque_enable_ever_attempted': bool(flags & W.TORQUE_ENABLE_EVER_ATTEMPTED) if packet else None,
                      'policy_locked': bool(fresh and flags & W.POLICY_LOCKED),
                      'mcu_contract_revision': W.SNAPSHOT_VERSION if packet else None,
                      'snapshot_version': W.SNAPSHOT_VERSION if packet else None,
                      'calibration_supported': bool(packet and fresh)}
            position_mask = packet['position_mask'] if packet else 0
            supply_rows = tuple((row['position_age_ms'], row['voltage_decivolts']) for row in packet['joints']) if packet else ()
        host_age = max(0, (now-observed)*1000) if observed is not None else 0
        valid = [(W.MOTOR_IDS[i], voltage/10, row_age+host_age)
                 for i, (row_age, voltage) in enumerate(supply_rows)
                 if fresh and position_mask & (1 << i) and row_age != 65535 and row_age+host_age < 150 and voltage > 0]
        result.update(mode='wireless_calibration', app_code_version=VERSION, host_read_only=False,
                      no_rpc_writes=False, control_ready=False, commissioning_motion_locked=False,
                      mcu_motion_locked=False, **values)
        result['calibration'] = {'supported': True, 'schema': SCHEMA, 'version': VERSION,
                                 'mcu_contract_revision': values['mcu_contract_revision'],
                                 'snapshot_version': values['snapshot_version'], 'fresh': fresh, 'age_ms': age}
        result['servo_power'] = {'supported': True, 'source': 'servo_supply', 'percent': None,
                                 'voltage_v': min(v[1] for v in valid) if valid else None,
                                 'voltage_min_v': min(v[1] for v in valid) if valid else None,
                                 'voltage_max_v': max(v[1] for v in valid) if valid else None,
                                 'age_ms': max(v[2] for v in valid) if valid else None,
                                 'fresh': bool(valid), 'valid_ids': [v[0] for v in valid],
                                 'measurement_scope': 'Servo supply only; battery state of charge is not measured.'}
        return result

    def power_status(self):
        packet, valid, per_id = self.snapshot, [], {}
        host_age = max(0, (self.clock()-self.observed_at)*1000) if self.observed_at is not None else 0
        for i, sid in enumerate(W.MOTOR_IDS):
            row = packet['joints'][i] if packet else None
            age = row['position_age_ms']+host_age if row and row['position_age_ms'] != 65535 else None
            fresh = bool(packet and self.fresh() and packet['position_mask'] & (1<<i)
                         and age is not None and age < 150 and row['voltage_decivolts'] > 0)
            voltage = row['voltage_decivolts']/10 if fresh else None
            per_id[str(sid)] = {'voltage_v': voltage, 'temperature_c': row['temperature_c'] if fresh else None,
                               'age_ms': age, 'fresh': fresh}
            if fresh:
                valid.append((sid, voltage, age))
        return {'supported': True, 'source': 'servo_supply', 'percent': None,
                'voltage_v': min(v[1] for v in valid) if valid else None,
                'voltage_min_v': min(v[1] for v in valid) if valid else None,
                'voltage_max_v': max(v[1] for v in valid) if valid else None,
                'age_ms': max(v[2] for v in valid) if valid else None, 'fresh': bool(valid),
                'valid_ids': [v[0] for v in valid], 'per_id': per_id,
                'voltage_summary': 'minimum_of_fresh_servo_supply_readings',
                'measurement_scope': 'Servo supply only; battery-terminal voltage and state of charge are not measured.'}

    def authenticate(self, key):
        if not isinstance(key, str) or not hmac.compare_digest(key.encode('utf-8'), self._key.encode('ascii')):
            raise CalibrationError('unauthorized', '校准请求未鉴权。', 401)

    def send(self, op, sid=0, nonce=0, delta=0, flags=0):
        if not self.snapshot:
            raise CalibrationError('mcu_unknown', '尚无主控校准反馈。', 503)
        self.request_seq = (self.request_seq+1) & 0xffffffff
        request = self.request_seq
        payload = W.encode_command(op, sid, self.snapshot['boot_id'], request, nonce,
                                   self.snapshot['sample_us'], delta_ticks=delta, flags=flags)
        self.requests[request] = (op, sid, nonce, delta)
        if len(self.requests) > 128:
            del self.requests[next(iter(self.requests))]
        self.sender.put(request, op, payload, self.generation)
        self.sent += 1
        return request

    def send_failed(self, request, op, code='bridge_send_failed', *, boot=None, generation=None):
        with self.condition:
            if ((boot is not None and (not self.snapshot or self.snapshot['boot_id'] != boot))
                    or (generation is not None and self.generation != generation)):
                return  # A late failure from a retired epoch cannot cancel a new session.
            self.failed_requests.add(request)
            code = code if code in ('command_route_unavailable','command_delivery_unknown') else 'bridge_send_failed'
            self.delivery_errors[request] = code
            if len(self.delivery_errors) > 128:
                old = next(iter(self.delivery_errors)); del self.delivery_errors[old]
                self.failed_requests.discard(old)
            sid = self.requests.get(request, (None, 0, 0))[1]
            message = '命令通道已断开，等待自动恢复后重新操作。' if code=='command_route_unavailable' else '主控命令发送未确认。'
            self.errors.append({'code': code, 'ids': [sid] if sid else [], 'message': message})
            current = self._session
            if current and op in (W.ARM, W.STEP, W.HEARTBEAT):
                self._session = None; self.generation += 1
                try:
                    self.send(W.CANCEL, current['id'], current['nonce'])
                except Exception:
                    self.errors.append({'code': 'cancel_send_failed', 'ids': [current['id']]})
            self.condition.notify_all()

    def raise_delivery_error(self, request):
        code = self.delivery_errors.get(request, 'bridge_send_failed')
        message = '命令通道已断开，等待自动恢复后重新操作。' if code=='command_route_unavailable' else '主控命令发送未确认。'
        raise CalibrationError(code, message, 503)

    def wait(self, request, generation, session=None, timeout=1.5):
        deadline = time.monotonic()+timeout
        with self.condition:
            while True:
                if request in self.failed_requests:
                    self.raise_delivery_error(request)
                if generation != self.generation or (session is not None and self._session is not session):
                    raise CalibrationError('cancelled', '授权取消或主控重启，操作未完成。')
                for packet, observed in reversed(self.history):
                    if packet['operation_ack_seq'] == request and self.clock()-observed < FRESH_S:
                        if packet['op'] != self.requests[request][0]:
                            raise CalibrationError('ack_mismatch', '主控返回了不同操作的应答。')
                        if packet['result'] == W.OK:
                            return packet
                        if packet['result'] == W.ERROR:
                            sid = self.requests[request][1]
                            rows = self.rows()
                            if self.requests[request][0] == W.CAPTURE_GROUP:
                                selected = next(group[2] for group in W.GROUPS.values() if group[0] == sid)
                                ids = [row['id'] for row in rows if not row['torque_off_verified']
                                       or (row['id'] in selected and (not row['valid'] or not row['stable']))] or list(selected)
                            else:
                                ids = [sid] if sid else [row['id'] for row in rows if not row['valid'] or not row['torque_off_verified'] or not row['stable']]
                            raise CalibrationError('mcu_rejected', '主控拒绝操作，错误代码 '+str(packet['error'])+'。', ids=ids)
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    if session is not None and self._session is session:
                        self._session = None; self.generation += 1
                        try:
                            self.send(W.CANCEL, session['id'], session['nonce'])
                        except Exception:
                            self.errors.append({'code': 'timeout_cancel_failed', 'ids': [session['id']]})
                    raise CalibrationError('mcu_timeout', '主控未及时确认；不自动重试。', 503)
                self.condition.wait(min(remaining, .05))

    def require_fresh(self, motion=True):
        if not self.fresh() or not self.snapshot['flags'] & W.POLICY_LOCKED or (motion and not self.snapshot['flags'] & W.BOOT_READY):
            raise CalibrationError('not_ready', '主控反馈过期或策略锁未确认。')

    def require_idle(self):
        if self._home_session or self._home_unknown:
            raise CalibrationError('busy', '先停止归位并等卸力读回，再记录零位或开始新的操作。')
        if self.operation or self._session or self._home_session or self._home_unknown or (self.snapshot and (self.snapshot['state'] != W.IDLE or self.snapshot['flags'] & (W.TORQUE_MAY_BE_ON | W.SESSION_ACTIVE | W.CAPTURE_BUSY))):
            raise CalibrationError('busy', '先支撑后卸力并取消当前授权，确认主控扭矩状态。')

    def session(self, body, sid, require_client=True):
        current = self._session
        if not current or sid != current['id'] or not isinstance(body.get('session'), str) or not hmac.compare_digest(body['session'], current['token']):
            raise CalibrationError('session_mismatch', '关节授权不匹配。')
        if self.clock() >= current['deadline'] or (require_client and self.clock()-current['client_at'] > CLIENT_LEASE_S):
            raise CalibrationError('session_expired', '授权已超时。')
        if current['confirmed'] and not self.active():
            raise CalibrationError('session_inactive', '主控授权已失效。')
        return current

    def action(self, body, key):
        self.authenticate(key)
        if not isinstance(body, dict):
            raise CalibrationError('invalid_request', '无效请求。', 400)
        op, sid = body.get('op'), body.get('id', 0)
        if isinstance(op, str) and op.startswith('home_'):
            return self.home_action(body)
        if op not in ('read', 'off_all', 'capture_all', 'capture_group', 'arm', 'jog', 'cancel', 'heartbeat'):
            raise CalibrationError('invalid_operation', '不支持该操作。', 400)
        if type(sid) is not int or sid not in (0, *W.MOTOR_IDS):
            raise CalibrationError('invalid_id', '无效关节编号。', 400)
        if op == 'heartbeat':
            with self.lock:
                session = self.session(body, sid)
                session['client_at'] = self.clock()
                expires = max(0, int((session['deadline']-self.clock())*1000))
            return {'ok': True, 'op': op, 'expires_ms': expires, 'status': self.calibration_status()}
        if op in ('cancel', 'off_all') and (self._home_session or self._home_unknown):
            if op == 'off_all' and body.get('supported') is not True:
                raise CalibrationError('support_required', '先托稳机身再全部卸力。')
            response = self.home_action({'op':'home_stop'})
            response['op'] = op
            return response
        if op in ('cancel', 'off_all'):
            if op == 'off_all' and body.get('supported') is not True:
                raise CalibrationError('support_required', '先托稳机身再全部卸力。')
            with self.lock:
                current = self._session
                if op == 'cancel' and current and body.get('session') != current['token']:
                    raise CalibrationError('session_mismatch', '授权不匹配。')
                self._session = None; self.generation += 1; self.condition.notify_all()
                generation = self.generation
                request = self.send(W.OFF_ALL if op == 'off_all' else W.CANCEL,
                                    0 if op == 'off_all' else current['id'] if current else 0,
                                    current['nonce'] if current else 0, flags=1 if op == 'off_all' else 0)
            self.wait(request, generation)
            with self.lock:
                rows = self.rows()
                relevant = rows if op == 'off_all' or current is None else [row for row in rows if row['id'] == current['id']]
                if any(not row['fresh'] or row['torque_enabled'] is not False for row in relevant):
                    raise CalibrationError('release_unconfirmed', '卸力读回未确认；请保持支撑。')
            return {'ok': True, 'op': op, 'status': self.calibration_status()}
        if op == 'arm':
            if sid == 0 or body.get('supported') is not True or body.get('clear') is not True:
                raise CalibrationError('confirmation_required', '先托稳机身并确认无碰撞。')
            with self.lock:
                self.require_idle(); self.require_fresh()
                row = next(row for row in self.rows() if row['id'] == sid)
                if not row['motion_valid'] or not row['torque_off_verified']:
                    raise CalibrationError('joint_blocked', row['motion_blocked_reason'] or '卸力未确认。', ids=[sid])
                now = self.clock()
                current = {'id': sid, 'token': secrets.token_hex(24), 'nonce': secrets.randbelow(0xffffffff)+1,
                           'anchor': row['position'], 'deadline': now+SESSION_S, 'client_at': now, 'confirmed': False}
                self._session = current
                try:
                    request = self.send(W.ARM, sid, current['nonce'], flags=3)
                except Exception:
                    self._session = None
                    raise
                generation = self.generation
            try:
                packet = self.wait(request, generation, current)
                with self.lock:
                    if packet['session'] != current['nonce'] or packet['active_id'] != sid or packet['state'] != W.ARMED or not packet['flags'] & W.SESSION_ACTIVE:
                        raise CalibrationError('arm_unconfirmed', '主控授权身份未确认。')
                    current['confirmed'] = True
            except Exception:
                with self.lock:
                    if self._session is current:
                        self._session = None
                raise
            return {'ok': True, 'op': op, 'id': sid, 'session': current['token'],
                    'expires_ms': max(0, int((current['deadline']-self.clock())*1000)), 'status': self.calibration_status()}
        if op == 'jog':
            degrees = body.get('model_delta_deg')
            if type(degrees) not in (int, float) or not math.isfinite(degrees) or not 0 < abs(degrees) <= 1 or sid == 0:
                raise CalibrationError('invalid_step', '只允许约 1°以内的小步。', 400)
            ticks = round(degrees*4095/360*SIGNS[sid])
            if not 1 <= abs(ticks) <= 11:
                raise CalibrationError('invalid_step', '小步太小或超限。', 400)
            with self.lock:
                current = self.session(body, sid)
                self.require_fresh()
                if self.operation:
                    raise CalibrationError('busy', '上一操作尚未完成。')
                if self.snapshot['state'] != W.ARMED or self.snapshot['flags'] & W.TORQUE_MAY_BE_ON:
                    raise CalibrationError('busy', '先等上一小步到达并卸力。', ids=[sid])
                row = next(row for row in self.rows() if row['id'] == sid)
                if not row['motion_valid'] or not 0 <= row['position']+ticks <= 4095 or abs(row['position']+ticks-current['anchor']) > 114:
                    raise CalibrationError('joint_blocked', row['blocked_reason'] or '超出本次授权范围。', ids=[sid])
                self.operation = 'jog'
                try:
                    request = self.send(W.STEP, sid, current['nonce'], delta=ticks)
                except Exception:
                    self.operation = None
                    self._session = None; self.generation += 1
                    raise
                generation = self.generation
            try:
                self.wait(request, generation, current)
                reached = self.wait_result(request, generation, current)
                with self.lock:
                    row = next(row for row in self.rows() if row['id'] == sid)
                    if not row['fresh'] or row['torque_enabled'] is not False:
                        raise CalibrationError('release_unconfirmed', '小步后的卸力未确认。')
            finally:
                with self.lock:
                    self.operation = None
            return {'ok': True, 'op': op, 'id': sid, 'model_delta_deg': degrees, 'delta_ticks': ticks,
                    'arrival': {'position': reached['reached_position'], 'goal': reached['goal'],
                                'source_us': reached['reached_us'], 'within_tolerance': True},
                    'after_torque_off': {'position': reached['off_position'], 'source_us': reached['off_read_us'],
                                         'not_arrival_accuracy': True},
                    'physical_acceptance_verified': False,
                    'status': self.calibration_status()}
        if op == 'read':
            with self.lock:
                self.require_fresh(motion=False)
                if self.operation:
                    raise CalibrationError('busy', '上一操作尚未完成。')
                request = self.send(W.DIAG if sid else W.READ, sid)
                generation = self.generation
            ack = self.wait(request, generation)
            result = {'ok': True, 'op': op, 'status': self.calibration_status()}
            if sid and ack['op'] == W.DIAG and ack['diag_id'] == sid and ack['flags'] & W.DIAG_READY:
                result['diagnostic'] = {'id': sid, 'offset_steps': ack['diag_offset'], 'angle_limit_min': ack['diag_min'],
                                        'angle_limit_max': ack['diag_max'], 'operation_ack_seq': request,
                                        'source_us': ack['sample_us'], 'new_read_for_this_request': True,
                                        'physical_acceptance_verified': False}
            return result
        return self.capture(body)

    def capture(self, body):
        grouped = body.get('op') == 'capture_group'
        group = body.get('group') if grouped else None
        if grouped and (not isinstance(group, str) or group not in W.GROUPS):
            raise CalibrationError('invalid_group', '只允许左腿、右腿或头部固定分组。', 400)
        if body.get('id', 0) != 0:
            raise CalibrationError('invalid_id', '记录分组时不指定单颗关节。', 400)
        target, mask, ids = W.GROUPS[group] if grouped else (0, W.ALL_MASK, W.MOTOR_IDS)
        op = W.CAPTURE_GROUP if grouped else W.CAPTURE_ALL
        operation = 'capture_group' if grouped else 'capture_all'
        if body.get('supported') is not True or body.get('pose_confirmed') is not True:
            raise CalibrationError('pose_required', '先托稳并对齐本组实物零姿态。' if grouped else '先托稳并对齐全部 15 颗实物零姿态。')
        with self.lock:
            self.require_idle(); self.require_fresh()
            owner = object()
            self._capture_owner = owner
            self.operation = operation
            try:
                request = self.send(op, target, flags=1)
            except Exception:
                self._capture_owner = None
                self.operation = None
                raise
            generation = self.generation

        def evidence_ready():
            rows, packet = self.rows(), self.snapshot
            window = self.stable_window(ids, request, op)
            blocked = [row['id'] for row in rows if not row['torque_off_verified']
                       or (row['id'] in ids and (not row['valid'] or not row['stable']))]
            ready = bool(self.fresh() and packet['state'] == W.IDLE
                         and packet['capture_mask'] == mask and packet['metadata_mask'] == W.ALL_MASK
                         and packet['operation_ack_seq'] == request and packet['last_received_req_seq'] == request
                         and packet['op'] == op and packet['result'] == W.OK
                         and packet['flags'] & W.BOOT_READY
                         and packet['flags'] & W.ALL_TORQUE_OFF_VERIFIED
                         and not packet['flags'] & (W.TORQUE_MAY_BE_ON | W.SESSION_ACTIVE | W.CAPTURE_BUSY)
                         and not blocked and window)
            return ready, rows, window, packet, blocked

        try:
            ack = self.wait(request, generation)
            deadline = time.monotonic()+1.5
            with self.condition:
                while True:
                    if generation != self.generation or self._capture_owner is not owner:
                        raise CalibrationError('capture_changed', '记录期间授权改变，原文件未改。')
                    ready, rows, window, packet, blocked = evidence_ready()
                    if ready:
                        break
                    remaining = deadline-time.monotonic()
                    if remaining <= 0:
                        raise CalibrationError('capture_not_ready', '编码、模式、卸力或稳定性未通过，原文件未改。', ids=blocked)
                    self.condition.wait(min(.05, remaining))
            with self.lock:
                self.require_fresh()
                ready, rows, window, packet, blocked = evidence_ready()
                if not ready:
                    raise CalibrationError('capture_not_ready', '编码、模式、卸力或稳定性未通过，原零位文件未改。', ids=blocked)
                positions = {str(row['id']): row['position'] for row in rows if row['id'] in ids}
                audit = {'source': 'DWR1', 'host_version': VERSION, 'snapshot_version': W.SNAPSHOT_VERSION,
                         'boot_id': packet['boot_id'], 'sequence': packet['sequence'],
                         'sample_us': packet['sample_us'], 'operation_ack_seq': ack['operation_ack_seq'],
                         'user_confirmed_all_poses': not grouped, 'user_confirmed_group_pose': True,
                         'user_confirmed_support': True, 'stability': window,
                         'snapshot_payload_hex': packet['raw_payload_hex'],
                         'ack_snapshot_payload_hex': ack['raw_payload_hex'],
                         'torque_off_readback': True, 'all_15_metadata_read_for_this_request': True,
                         'coordinate_domain': NATIVE_DOMAIN, 'capture_domains': {'0': SINGLE_DOMAIN, '4': NATIVE_DOMAIN},
                         'model_deployment_ready': False, 'servo_supply_epoch_verified': False, 'joints': [row for row in rows if row['id'] in ids],
                         'all_15_torque_readback': [{'id': row['id'], 'mode': row['mode'],
                                                    'torque_enabled': row['torque_enabled'],
                                                    'metadata_age_ms': row['metadata_age_ms']} for row in rows],
                         'physical_acceptance_verified': False, 'policy_applied': False}
                first = W.decode_snapshot(bytes.fromhex(window['snapshot_payloads_hex'][-1]))
                audit['raw_signed_endpoints'] = {str(sid): {'mode': packet['joints'][W.MOTOR_IDS.index(sid)]['mode'],
                    'first_raw_word': first['joints'][W.MOTOR_IDS.index(sid)]['raw_word'],
                    'first_signed_steps': first['joints'][W.MOTOR_IDS.index(sid)]['signed_steps'],
                    'last_raw_word': packet['joints'][W.MOTOR_IDS.index(sid)]['raw_word'],
                    'last_signed_steps': packet['joints'][W.MOTOR_IDS.index(sid)]['signed_steps']} for sid in ids}
                source_rows = {row['id']: row for row in rows}
                groups = dict(self._groups) if grouped else {}
                saved_at = datetime.now(timezone.utc).isoformat()
                for name in ((group,) if grouped else tuple(W.GROUPS)):
                    selected_ids = W.GROUPS[name][2]
                    groups[name] = {'ids': list(selected_ids), 'saved_at': saved_at, 'audit': audit,
                                    'records': {str(sid): {'position': positions[str(sid)], 'group': name,
                                                'mode': source_rows[sid]['mode'], 'raw_word': source_rows[sid]['raw_word'],
                                                'signed_steps': source_rows[sid]['signed_steps'],
                                                'coordinate_domain': coordinate_domain(source_rows[sid]['mode']),
                                                'model_deployment_ready': False,
                                                'saved_at': saved_at, 'physical_acceptance_verified': False,
                                                'historical': True} for sid in selected_ids}}

                def before_commit():
                    with self.lock:
                        if (generation != self.generation or self._capture_owner is not owner
                                or not self.fresh() or self.snapshot['boot_id'] != packet['boot_id']):
                            raise CalibrationError('capture_changed', '记录期间反馈或授权改变，原文件未改。')
                        current_ready, latest, _, _, _ = evidence_ready()
                        if not current_ready:
                            raise CalibrationError('capture_changed', '记录期间有效性改变，原文件未改。')
                        if any(abs(row['position']-positions[str(row['id'])]) > MAX_SPREAD for row in latest if row['id'] in ids):
                            raise CalibrationError('capture_moved', '记录期间关节移动，原文件未改。')
            group_file, file = self.store.save_groups(groups, before_commit)
            with self.lock:
                self._group_summary, self._groups = dict(group_file), groups
                self._records = self.store.group_records(groups)
                # Arrival belonged to the previously published zero identity.
                # A successfully recorded replacement needs its own HOME proof.
                self._home_arrival = None
                if file is not None:
                    self._native_summary = dict(file)
                    self._native_published_records = {sid: dict(value) for sid, value in self._records.items()}
        except CalibrationError as error:
            if error.code == 'rollback_failed':
                with self.lock:
                    self._native_summary = {**self._native_summary, 'verified': False}
                    self._group_summary = {**self._group_summary, 'verified': False}
            raise
        except OSError:
            raise CalibrationError('save_failed', '零位保存或读回未通过，原文件未提交。', 503) from None
        finally:
            with self.lock:
                if self._capture_owner is owner:
                    self._capture_owner = None
                    if self.operation == operation:
                        self.operation = None
        return {'ok': True, 'op': operation, 'group': group, 'saved_count': len(ids),
                'captured_ids': [sid for sid in W.MOTOR_IDS if str(sid) in self._records],
                'missing_ids': [sid for sid in W.MOTOR_IDS if str(sid) not in self._records],
                'complete': len(self._records) == 15, 'published': file is not None,
                'file': dict(self._file_summary), 'native_file': dict(self._native_summary), 'group_file': group_file,
                'native_published': file is not None, 'native_only': True, 'coordinate_domain': NATIVE_DOMAIN,
                'model_deployment_ready': False,
                'policy_applied': False, 'physical_acceptance_verified': False, 'status': self.calibration_status()}

    def wait_result(self, request, generation, session, timeout=.8):
        deadline = time.monotonic()+timeout
        with self.condition:
            while True:
                if generation != self.generation or self._session is not session:
                    raise CalibrationError('cancelled', '小步结果等待已取消。')
                item = self.results.get((self.snapshot['boot_id'], request))
                if item and self.clock()-item[1] < FRESH_S:
                    result = item[0]
                    tolerance = 1 if abs(result['delta_ticks']) <= 3 else min(3, abs(result['delta_ticks'])//3)
                    if result['delta_ticks'] != self.requests[request][3] or result['goal'] != result['start']+result['delta_ticks']:
                        raise CalibrationError('result_mismatch', '小步结果与本次命令不一致。', ids=[result['target_id']])
                    if result['result'] != W.OK or result['op'] != W.STEP or abs(result['reached_position']-result['goal']) > tolerance:
                        raise CalibrationError('arrival_unconfirmed', '到达反馈未确认。')
                    if result['start'] == result['reached_position'] or (result['reached_position']-result['start'])*result['delta_ticks'] <= 0:
                        raise CalibrationError('movement_unconfirmed', '未确认本次小步的实际位置变化。')
                    if W.elapsed(result['off_read_us'], result['reached_us']) > 1000000:
                        raise CalibrationError('result_expired', '小步结果源时刻不一致。')
                    return result
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise CalibrationError('result_missing', '达到及卸力后两组位置结果尚未读到。', 503)
                self.condition.wait(min(.05, remaining))

    def tick(self):
        self.home_tick()
        with self.lock:
            current = self._session
            if not current:
                return
            if not self.active():
                self._session = None; self.generation += 1; self.condition.notify_all()
                try:
                    self.send(W.CANCEL, current['id'], current['nonce'])
                except Exception:
                    self.errors.append({'code': 'watchdog_cancel_unconfirmed'})
                return
            if current['confirmed'] and (self.last_heartbeat is None or self.clock()-self.last_heartbeat >= HEARTBEAT_S):
                try:
                    self.send(W.HEARTBEAT, current['id'], current['nonce'])
                    self.last_heartbeat = self.clock()
                except Exception:
                    self._session = None; self.generation += 1; self.condition.notify_all()
                    self.errors.append({'code': 'heartbeat_send_failed'})

    def stop(self):
        with self.lock:
            if self._home_session or self._home_unknown:
                self.home_stop_send()
        with self.lock:
            current, self._session = self._session, None
            self.generation += 1; self.condition.notify_all()
            if current:
                try:
                    self.send(W.CANCEL, current['id'], current['nonce'])
                except Exception:
                    self.errors.append({'code': 'shutdown_cancel_unconfirmed'})
        # Let an already queued release leave the process, without an unbounded join.
        deadline = time.monotonic()+.2
        while self.sender.queue.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(.005)
        self.sender.close()
