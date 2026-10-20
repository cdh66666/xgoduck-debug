"""JP1 optional read-only joint observation: fixed 96 bytes, canonical base64.

Offsets (little endian): magic0, boot3, sequence7, IMU age reference11,
mode4 mask15, valid mask17, signed positions19, ceil ages49, native SHA64.
Invalid joints have position0, age255 and neither mask bit. This is observation
data only and contains no authorization, command, torque or power state.
"""
import base64
import binascii
import struct

POSE = struct.Struct('<3sIIIHH15h15B32s')
POSE_SIZE = 96
ALL_MASK = 0x7fff
assert POSE.size == POSE_SIZE


def decode_pose(value):
    if not isinstance(value, str) or len(value) != 128:
        raise ValueError('JP1 requires canonical 128-character base64')
    try:
        raw = base64.b64decode(value, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError('Invalid JP1 base64') from exc
    if len(raw) != POSE_SIZE or base64.b64encode(raw).decode('ascii') != value:
        raise ValueError('Invalid JP1 length or encoding')
    magic, boot, sequence, reference, mode4, valid, *tail = POSE.unpack(raw)
    positions, ages, sha = tail[:15], tail[15:30], tail[30]
    if (magic != b'JP1' or not boot or reference >= 150000
            or (mode4 | valid) & ~ALL_MASK or mode4 & ~valid or sha == bytes(32)):
        raise ValueError('Invalid JP1 identity, masks or age reference')
    for index, (position, age) in enumerate(zip(positions, ages)):
        if valid & (1 << index):
            if age >= 150 or position == -32768 or not mode4 & (1 << index) and not 0 <= position <= 4095:
                raise ValueError('Invalid JP1 joint observation')
        elif position != 0 or age != 255:
            raise ValueError('Invalid JP1 unavailable joint marker')
    return {'boot_id': boot, 'sequence': sequence, 'imu_age_ref_us': reference,
            'mode4_mask': mode4, 'valid_mask': valid, 'positions': positions,
            'position_ages_ms': ages, 'native_sha256': sha.hex()}


def encode_pose(boot, sequence, reference, mode4, valid, positions, ages, native_sha):
    try:
        sha = bytes.fromhex(native_sha)
        if len(sha) != 32 or native_sha != sha.hex():
            raise ValueError('Invalid native SHA encoding')
        raw = POSE.pack(b'JP1', boot, sequence, reference, mode4, valid,
                        *positions, *ages, sha)
    except (ValueError, TypeError, struct.error) as exc:
        raise ValueError('Invalid JP1 fields') from exc
    value = base64.b64encode(raw).decode('ascii')
    decode_pose(value)
    return value
