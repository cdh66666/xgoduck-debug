import struct
import calibration_wire as calibration

VERSION = 1
BEGIN, SELECT, HOLD, HEARTBEAT, STOP = range(5)
PHASES = ('off', 'preparing', 'selected', 'capturing', 'enabling', 'holding', 'releasing', 'done', 'fault')
COMMAND = struct.Struct('<4sBBBB6IhBB')
SNAPSHOT = struct.Struct('<4sBBBB9I5H2BhBB30h30B3I')
PARAMETERS = struct.Struct('<4sBBBB5I7B12BB')
assert COMMAND.size == 36 and SNAPSHOT.size == 162 and PARAMETERS.size == 48


def encode(operation, target, boot, request, session, source, selection=0, position=0, mode=0):
    if type(operation) is not int or operation not in range(5):
        raise ValueError('invalid sequence operation')
    if target not in (calibration.MOTOR_IDS if operation in (SELECT, HOLD) else (0,)):
        raise ValueError('invalid sequence target')
    if not boot or not request or operation != STOP and not session:
        raise ValueError('missing sequence identity')
    for value, name in ((boot, 'boot'), (request, 'request'), (session, 'session'), (source, 'source'), (selection, 'selection')):
        calibration.uint32(value, name)
    if operation == HOLD:
        if not selection or mode not in (0, 4) or type(position) is not int or not -32767 <= position <= 32767 or mode == 0 and not 0 <= position <= 4095:
            raise ValueError('invalid capture identity')
    elif selection or position or mode:
        raise ValueError('unexpected capture identity')
    flags = 3 if operation in (BEGIN, HOLD) else 1 if operation == STOP else 0
    return COMMAND.pack(b'DZC1', VERSION, operation, target, flags, boot, request, session, source, 250000, selection, position, mode, 0)


def decode(payload):
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != SNAPSHOT.size:
        raise ValueError('expected162byte sequence snapshot')
    fields = SNAPSHOT.unpack(payload)
    if (fields[0] != b'DZS1' or fields[1] != VERSION or fields[2] not in range(9)
            or fields[3] not in range(3) or fields[4] not in range(19) or not fields[5]
            or fields[19] not in (0, *calibration.MOTOR_IDS) or fields[20] not in range(11) or fields[23]):
        raise ValueError('invalid sequence snapshot')
    if any(mask & ~calibration.ALL_MASK for mask in fields[14:19]) or fields[17] & fields[18]:
        raise ValueError('invalid sequence masks')
    packet = dict(zip(('boot_id', 'sequence', 'sample_us', 'operation_ack_seq', 'session', 'expires_us',
                      'last_received_req_seq', 'selection_seq', 'capture_seq'), fields[5:14]))
    packet.update(zip(('held_mask', 'fresh_mask', 'metadata_mask', 'off_mask', 'enabled_mask'), fields[14:19]))
    packet.update(version=fields[1], phase=PHASES[fields[2]], result=fields[3], error=fields[4],
                  selected_id=fields[19], stage=fields[20], capture_position=fields[21], capture_mode=fields[22],
                  goals=list(fields[24:39]), positions=list(fields[39:54]), modes=list(fields[54:69]), torques=list(fields[69:84]),
                  capture_start_us=fields[84], capture_us=fields[85], held_us=fields[86], raw_payload_hex=bytes(payload).hex())
    for index in range(15):
        bit = 1 << index
        if packet['fresh_mask'] & bit and packet['positions'][index] == -32768:
            raise ValueError('unknown fresh position')
        if packet['metadata_mask'] & bit and (packet['modes'][index] == 255 or packet['torques'][index] not in (0, 1)):
            raise ValueError('invalid fresh metadata')
    return packet


def decode_parameters(payload):
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != PARAMETERS.size:
        raise ValueError('expected48byte parameter snapshot')
    fields = PARAMETERS.unpack(payload)
    if fields[0] != b'DPC1' or fields[1] != VERSION or fields[2] not in range(3) or fields[3] not in range(19) or fields[4] not in calibration.MOTOR_IDS or not fields[5] or fields[-1]:
        raise ValueError('invalid parameter snapshot')
    return {'version': fields[1], 'result': fields[2], 'error': fields[3], 'id': fields[4], 'boot_id': fields[5],
            'request_seq': fields[6], 'sample_us': fields[7], 'parameters21_us': fields[8], 'parameters40_us': fields[9],
            'registers': {str(address): value for address, value in zip((*range(21, 28), *range(40, 52)), fields[10:-1])},
            'raw_payload_hex': bytes(payload).hex(), 'read_only': True}
