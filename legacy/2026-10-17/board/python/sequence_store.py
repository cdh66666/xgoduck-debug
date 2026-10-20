import hashlib
import json
import calibration_wire as calibration
import sequence_wire as sequence

FORMAT = 'sequential-axis-overrides-v1'


def validate_evidence(axis, evidence):
    selected = sequence.decode(bytes.fromhex(evidence['selected_payload_hex']))
    held = sequence.decode(bytes.fromhex(evidence['held_payload_hex']))
    index = calibration.MOTOR_IDS.index(axis)
    bit = 1 << index
    if (evidence.get('user_confirmed_axis_pose') is not True or evidence.get('user_confirmed_support') is not True
            or selected['selected_id'] != axis or selected['phase'] != 'selected' or selected['error']
            or held['selected_id'] != axis or held['phase'] != 'holding' or held['result'] != calibration.OK or held['error']
            or not selected['selection_seq'] or selected['selection_seq'] != held['selection_seq']
            or selected['boot_id'] != held['boot_id'] or not selected['session'] or selected['session'] != held['session']
            or not held['capture_seq'] or held['capture_seq'] != held['operation_ack_seq']
            or selected['held_mask'] & bit or held['held_mask'] != (selected['held_mask'] | bit)
            or selected['fresh_mask'] != calibration.ALL_MASK or selected['metadata_mask'] != calibration.ALL_MASK
            or selected['enabled_mask'] != selected['held_mask'] or selected['off_mask'] != calibration.ALL_MASK ^ selected['held_mask']
            or held['fresh_mask'] != calibration.ALL_MASK or held['metadata_mask'] != calibration.ALL_MASK
            or held['enabled_mask'] != held['held_mask'] or held['off_mask'] != calibration.ALL_MASK ^ held['held_mask']
            or held['capture_mode'] not in (0, 4) or held['capture_mode'] != selected['modes'][index]
            or held['capture_mode'] != held['modes'][index] or held['goals'][index] != held['capture_position']
            or not -32767 <= held['capture_position'] <= 32767
            or held['capture_mode'] == 0 and not 0 <= held['capture_position'] <= 4095
            or abs(held['capture_position'] - selected['positions'][index]) > 57
            or abs(held['positions'][index] - held['capture_position']) > 57
            or not 500000 <= calibration.elapsed(held['capture_us'], held['capture_start_us']) < 2000000
            or not 300000 <= calibration.elapsed(held['held_us'], held['capture_us']) < 2000000
            or calibration.elapsed(held['sample_us'], held['held_us']) >= 150000):
        raise ValueError('incomplete sequential capture/hold evidence')
    for packet in (selected, held):
        for offset in range(15):
            powered = bool(packet['held_mask'] & (1 << offset))
            if packet['modes'][offset] not in (0, 4) or packet['torques'][offset] != int(powered) or powered and abs(packet['positions'][offset] - packet['goals'][offset]) > 57:
                raise ValueError('sequential hold/off set differs')
    return held


def observed_record(axis, evidence, saved_at):
    packet = validate_evidence(axis, evidence)
    group = next(name for name, values in calibration.GROUPS.items() if axis in values[2])
    position, mode = packet['capture_position'], packet['capture_mode']
    return {'position': position, 'raw_word': calibration.native_word(position), 'signed_steps': position,
            'mode': mode, 'coordinate_domain': 'native-sign-magnitude-v1' if mode == 4 else 'single-turn-0-4095',
            'saved_at': saved_at, 'group': group, 'historical': True, 'model_deployment_ready': False,
            'physical_acceptance_verified': False}


def validate_group(store, name, group):
    if group.get('format') != FORMAT or group.get('ids') != list(calibration.GROUPS[name][2]):
        raise ValueError('invalid sequential group')
    baseline = group.get('baseline')
    expected = {}
    if baseline is not None:
        if baseline.get('format') == FORMAT:
            raise ValueError('nested sequential baseline')
        store.validate_group(name, baseline)
        expected = store.group_records({name: baseline})
    expected_hash = hashlib.sha256(store._json_bytes(baseline)).hexdigest()
    if group['audit'].get('baseline_sha256') != expected_hash or not isinstance(group.get('overrides'), dict) or not group['overrides']:
        raise ValueError('sequential baseline identity differs')
    for sid, override in group['overrides'].items():
        if sid not in {str(axis) for axis in calibration.GROUPS[name][2]}:
            raise ValueError('sequential axis is outside group')
        expected[sid] = observed_record(int(sid), override['evidence'], override['saved_at'])
    if store._json_bytes(expected) != store._json_bytes(group.get('records')):
        raise ValueError('sequential save altered other records')


def make_groups(store, groups, axis, evidence, saved_at):
    updated = json.loads(store._json_bytes(groups))
    name = next(name for name, values in calibration.GROUPS.items() if axis in values[2])
    old = updated.get(name)
    if old is not None:
        store.validate_group(name, old)
    baseline = old.get('baseline') if old and old.get('format') == FORMAT else old
    overrides = dict(old['overrides']) if old and old.get('format') == FORMAT else {}
    overrides[str(axis)] = {'evidence': evidence, 'saved_at': saved_at}
    records = store.group_records({name: old}) if old else {}
    records[str(axis)] = observed_record(axis, evidence, saved_at)
    updated[name] = {'format': FORMAT, 'ids': list(calibration.GROUPS[name][2]), 'baseline': baseline,
                     'overrides': overrides, 'records': records,
                     'audit': {'source': 'DZS1_sequential_axis_capture_hold_v1', 'user_confirmed_group_pose': False,
                               'baseline_sha256': hashlib.sha256(store._json_bytes(baseline)).hexdigest()}}
    validate_group(store, name, updated[name])
    return updated
