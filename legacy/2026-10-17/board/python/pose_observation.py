"""Cache-only IMU plus optional native joint observation; never writes or moves."""
import math
import re

import calibration_wire as W
from imu_transport.pose_codec import encode_pose

SHA = re.compile(r'[0-9a-f]{64}\Z')


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _u32(value):
    return type(value) is int and 0 <= value <= 0xffffffff


def _native_identity(host):
    summary, group_summary = host._native_summary, host._group_summary
    records, groups = host._native_published_records, host._groups
    if (summary.get('verified') is not True or group_summary.get('verified') is not True
            or summary.get('path') != 'data/zero_pos.native.json'
            or not isinstance(summary.get('sha256'), str) or not SHA.fullmatch(summary['sha256'])
            or summary.get('group_records_sha256') != group_summary.get('sha256')
            or not isinstance(group_summary.get('sha256'), str) or not SHA.fullmatch(group_summary['sha256'])
            or not isinstance(records, dict) or set(records) != {str(sid) for sid in W.MOTOR_IDS}
            or not isinstance(groups, dict) or set(groups) != set(W.GROUPS)):
        return None
    # Compare 15 small cached records, never serialize/hash their large audits.
    for name, (_, _, ids) in W.GROUPS.items():
        group = groups.get(name)
        if not isinstance(group, dict) or group.get('ids') != list(ids):
            return None
        selected = group.get('records')
        if not isinstance(selected, dict) or set(selected) != {str(sid) for sid in ids}:
            return None
        if any(selected[str(sid)] != records[str(sid)] for sid in ids):
            return None
    return summary['sha256']


def provider(host, telemetry):
    """Return telemetry.live_status() plus optional strictly encoded JP1 data.

    Header imu_age_ref_us floors the original IMU cache age. Frontends add
    max(0, total_feedback_age_ms - reference/1000) to each joint's stored age;
    this conservatively includes sensor lag, callbacks, RTT and relay queues.
    """
    return attach_pose(host, telemetry.live_status())


def attach_pose(host, observation):
    """Append optional cached JP1 to an existing DQS snapshot without rereading it."""
    result = dict(observation)
    result.pop('native_pose_compact', None)
    try:
        imu_age = result.get('feedback_age_ms')
        if not _finite(imu_age) or not 0 <= imu_age < 150:
            return result
        with host.lock:
            native_sha = _native_identity(host)
            packet, observed = host.snapshot, host.observed_at
            now = host.clock()
            if (native_sha is None or not isinstance(packet, dict)
                    or not _u32(packet.get('boot_id')) or not packet['boot_id']
                    or not _u32(packet.get('sequence')) or not _finite(now) or not _finite(observed)):
                return result
            cache_age = (now - observed) * 1000
            cache_valid = 0 <= cache_age < 150 and not host.protocol_fault
            position_mask, metadata_mask = packet.get('position_mask'), packet.get('metadata_mask')
            if type(position_mask) is not int or type(metadata_mask) is not int:
                return result
            positions, ages, valid_mask, mode4_mask = [0] * 15, [255] * 15, 0, 0
            rows = packet.get('joints')
            if not isinstance(rows, (list, tuple)) or len(rows) != 15:
                return result
            for index, row in enumerate(rows):
                if not cache_valid or not isinstance(row, dict):
                    continue
                mode, raw, signed = row.get('mode'), row.get('raw_word'), row.get('signed_steps')
                source_age, metadata_age = row.get('position_age_ms'), row.get('metadata_age_ms')
                if (type(mode) is not int or mode not in (0, 4)
                        or type(raw) is not int or not 0 <= raw <= 65535 or raw == 32768
                        or type(signed) is not int or not -32767 <= signed <= 32767
                        or signed != W.signed_word(raw) or mode == 0 and not 0 <= signed <= 4095
                        or type(source_age) is not int or not 0 <= source_age < 65535
                        or type(metadata_age) is not int or not 0 <= metadata_age < 65535
                        or not position_mask & (1 << index) or not metadata_mask & (1 << index)
                        or metadata_age + cache_age > 750):
                    continue
                age = math.ceil(source_age + cache_age)
                if age >= 150:
                    continue
                positions[index], ages[index] = signed, age
                valid_mask |= 1 << index
                if mode == 4:
                    mode4_mask |= 1 << index
            value = encode_pose(packet['boot_id'], packet['sequence'], math.floor(imu_age * 1000),
                                mode4_mask, valid_mask, positions, ages, native_sha)
        result['native_pose_compact'] = value
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        # This optional observation must not interrupt valid IMU telemetry.
        pass
    return result
