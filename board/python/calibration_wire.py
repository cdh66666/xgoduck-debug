"""Separate, bounded calibration ABI; legacy DQC/DQL/DQV are never emitted."""
import struct

MOTOR_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
ALL_MASK = 0x7fff
COMMAND = struct.Struct('<4sBBBB5IhH')
HEADER = struct.Struct('<4sBBBB7Ihh4HBBBBhHH')
ROW = struct.Struct('<HhHH4B')
ACTION_RESULT = struct.Struct('<4sBBBB4I6h2IBBBB')
SNAPSHOT_SIZE = HEADER.size + 15 * ROW.size
SNAPSHOT_VERSION = 3
READ, ARM, STEP, HEARTBEAT, CANCEL, CAPTURE_ALL, OFF_ALL, DIAG = range(8)
CAPTURE_GROUP = 8
GROUPS = {'left': (1, 0x001f, MOTOR_IDS[:5]),
          'right': (2, 0x03e0, MOTOR_IDS[5:10]),
          'head': (3, 0x7c00, MOTOR_IDS[10:])}
BOOT, IDLE, PREFLIGHT, ARMED, MOVING, VERIFY_OFF, CAPTURING, FAULT = range(8)
POLICY_LOCKED, SESSION_ACTIVE, TORQUE_MAY_BE_ON, CAPTURE_BUSY = 1, 2, 4, 8
ALL_TORQUE_OFF_VERIFIED, BOOT_READY, DIAG_READY = 16, 32, 64
TORQUE_ENABLE_EVER_ATTEMPTED = 128
PENDING, OK, ERROR = range(3)
assert COMMAND.size == 32 and HEADER.size == 58 and SNAPSHOT_SIZE == 238
assert ACTION_RESULT.size == 48


def uint32(value, name):
    if type(value) is not int or not 0 <= value <= 0xffffffff:
        raise ValueError('invalid ' + name)
    return value


def encode_command(op, target_id, boot_nonce, request_seq, session, source_us,
                   ttl_us=250000, delta_ticks=0, flags=0):
    if type(op) is not int or op not in range(9):
        raise ValueError('invalid calibration operation')
    if type(target_id) is not int or target_id not in ((1, 2, 3) if op == CAPTURE_GROUP else (0, *MOTOR_IDS)):
        raise ValueError('invalid calibration ID')
    if op in (ARM, STEP, HEARTBEAT, DIAG) and target_id not in MOTOR_IDS:
        raise ValueError('single calibration ID required')
    if op in (CAPTURE_ALL, OFF_ALL) and target_id != 0:
        raise ValueError('all-ID operation requires target zero')
    if type(flags) is not int or flags & ~3:
        raise ValueError('invalid calibration confirmations')
    expected_flags = 3 if op == ARM else 1 if op in (OFF_ALL, CAPTURE_ALL, CAPTURE_GROUP) else 0
    if flags != expected_flags:
        raise ValueError('calibration confirmations do not match operation')
    if type(delta_ticks) is not int or (op == STEP and not 1 <= abs(delta_ticks) <= 22):
        raise ValueError('invalid calibration step')
    if op != STEP and delta_ticks != 0:
        raise ValueError('unexpected calibration step')
    if type(ttl_us) is not int or not 0 < ttl_us <= 250000:
        raise ValueError('invalid calibration command lifetime')
    fields = [uint32(value, name) for value, name in (
        (boot_nonce, 'boot identity'), (request_seq, 'request sequence'),
        (session, 'session'), (source_us, 'source time'), (ttl_us, 'lifetime'))]
    if op == CAPTURE_GROUP and session != 0:
        raise ValueError('group capture does not carry an authorization session')
    return COMMAND.pack(b'DWC1', 1, op, target_id, flags, *fields, delta_ticks, 0)


def signed_word(word):
    return -(word & 0x7fff) if word & 0x8000 else word


def decode_snapshot(payload):
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != SNAPSHOT_SIZE:
        raise ValueError('expected 238-byte DWR1 calibration snapshot')
    h = HEADER.unpack_from(payload)
    if h[0] != b'DWR1' or h[1] != SNAPSHOT_VERSION:
        raise ValueError('invalid calibration snapshot ABI')
    if h[2] not in range(8) or h[18] not in range(3) or h[19] not in (*range(9), 10) or h[20] not in range(19):
        raise ValueError('invalid calibration snapshot state/result')
    if h[3] & ALL_TORQUE_OFF_VERIFIED and h[3] & TORQUE_MAY_BE_ON:
        raise ValueError('contradictory torque evidence')
    if h[2] == FAULT and h[3] & ALL_TORQUE_OFF_VERIFIED:
        raise ValueError('fault cannot claim all-off verification')
    if h[5] == 0:
        raise ValueError('unknown MCU boot identity')
    if h[4] not in (0, *MOTOR_IDS) or h[21] not in (0, *MOTOR_IDS):
        raise ValueError('invalid snapshot ID')
    if any(mask & ~ALL_MASK for mask in h[14:18]):
        raise ValueError('invalid snapshot mask')
    rows = []
    for index, sid in enumerate(MOTOR_IDS):
        raw, decoded, pos_age, meta_age, mode, torque, voltage, temperature = ROW.unpack_from(
            payload, HEADER.size + index * ROW.size)
        if h[14] & (1 << index):
            if pos_age > 150 or decoded != signed_word(raw):
                raise ValueError('invalid position source age/decoding')
        if h[15] & (1 << index):
            if meta_age > 750 or mode == 255 or torque not in (0, 1):
                raise ValueError('invalid metadata source age/state')
        rows.append(dict(id=sid, raw_word=raw, signed_steps=decoded,
                         position_age_ms=pos_age, metadata_age_ms=meta_age,
                         mode=mode, torque=torque, voltage_decivolts=voltage,
                         temperature_c=temperature))
    return dict(snapshot_version=h[1], raw_payload_hex=bytes(payload).hex(),
                state=h[2], flags=h[3], active_id=h[4], boot_id=h[5], sequence=h[6],
                sample_us=h[7], operation_ack_seq=h[8], session=h[9], expires_us=h[10],
                last_received_req_seq=h[11], anchor=h[12], goal=h[13], position_mask=h[14],
                metadata_mask=h[15], stable_mask=h[16], capture_mask=h[17], result=h[18],
                op=h[19], error=h[20], diag_id=h[21], diag_offset=h[22],
                diag_min=h[23], diag_max=h[24], joints=rows)


def elapsed(now, before):
    return (now - before) & 0xffffffff


def decode_action_result(payload):
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != ACTION_RESULT.size:
        raise ValueError('expected 48-byte DWA1 operation result')
    p = ACTION_RESULT.unpack(payload)
    if p[0] != b'DWA1' or p[1] != 1 or p[2] not in (*range(9), 10) or p[3] not in ((1, 2, 3) if p[2] == CAPTURE_GROUP else MOTOR_IDS if p[2] == 10 else (0, *MOTOR_IDS)) or p[4] not in range(3) or p[5] == 0 or p[17] not in range(19) or p[20] != 0:
        raise ValueError('invalid DWA1 result ABI')
    if p[2] == STEP and p[4] == OK:
        if p[3] not in MOTOR_IDS or any(not 0 <= v <= 4095 for v in p[9:14]) or not 1 <= abs(p[14]) <= 22 or p[15] == 0 or p[16] == 0:
            raise ValueError('incomplete successful step result')
        if elapsed(p[16], p[15]) >= 0x80000000:
            raise ValueError('off read precedes arrival')
    return dict(op=p[2], target_id=p[3], result=p[4], boot_id=p[5], request_seq=p[6], session=p[7],
                sample_us=p[8], anchor=p[9], start=p[10], goal=p[11], reached_position=p[12],
                off_position=p[13], delta_ticks=p[14], reached_us=p[15], off_read_us=p[16],
                error=p[17], voltage_decivolts=p[18], temperature_c=p[19])


# Independent full-native home protocol; old encode_command ID rules remain unchanged.
HOME_RUN = 9
HOME_COMMAND = struct.Struct('<4sBBH5I32s15h15B3B')
HOME_SNAPSHOT = struct.Struct('<4sBBBB6I32s7H2B45h30B')
HOME_PHASES = ('off','preparing','prepared','enabling','moving','holding','releasing','done','fault')
HOME_STAGES = ('idle', 'read_mode', 'read_torque', 'read_position', 'prime_echo',
               'enable_readback', 'prime_settle', 'ramp', 'final_settle', 'hold', 'release')
HOME_CONTRACT_VERSION = 5
assert HOME_COMMAND.size == 108 and HOME_SNAPSHOT.size == 200


def native_word(position):
    if type(position) is not int or not -32767 <= position <= 32767:
        raise ValueError('invalid native position')
    return (0x8000 | -position) if position < 0 else position


def encode_home_command(targets, modes, native_sha256, boot_nonce, request_seq, session, source_us, ttl_us=250000):
    if (len(targets) != 15 or len(modes) != 15 or any(type(m) is not int or m not in (0,4) for m in modes)
            or any(type(p) is not int or not -32767 <= p <= 32767 or m == 0 and not 0 <= p <= 4095 for p,m in zip(targets,modes))):
        raise ValueError('fixed fifteen native positions/modes required')
    if type(native_sha256) is not str or len(native_sha256) != 64 or any(c not in '0123456789abcdef' for c in native_sha256) or native_sha256 == '0'*64:
        raise ValueError('audited native SHA256 required')
    fields=[uint32(v,n) for v,n in ((boot_nonce,'boot'),(request_seq,'request'),(session,'home session'),(source_us,'source'),(ttl_us,'ttl'))]
    if not boot_nonce or not request_seq or not session or not 0 < ttl_us <= 250000:
        raise ValueError('home identity and lifetime required')
    return HOME_COMMAND.pack(b'DWH1',HOME_CONTRACT_VERSION,3,0,*fields,bytes.fromhex(native_sha256),*targets,*modes,0,0,0)


def encode_home_control(op,boot_nonce,request_seq,session,source_us,ttl_us=250000):
    if op not in (HOME_RUN,HEARTBEAT,CANCEL,OFF_ALL) or type(op) is not int:
        raise ValueError('invalid home control')
    if op in (HOME_RUN,HEARTBEAT) and not session:
        raise ValueError('home session required')
    fields=[uint32(v,n) for v,n in ((boot_nonce,'boot'),(request_seq,'request'),(session,'home session'),(source_us,'source'),(ttl_us,'ttl'))]
    if not boot_nonce or not request_seq or not 0 < ttl_us <= 250000:
        raise ValueError('home identity and lifetime required')
    return COMMAND.pack(b'DWC1',1,op,0,1 if op == OFF_ALL else 0,*fields,0,0)


def decode_home_snapshot(payload):
    if not isinstance(payload,(bytes,bytearray)) or len(payload)!=200:
        raise ValueError('expected fixed 200-byte DHS1 snapshot')
    p=HOME_SNAPSHOT.unpack(payload)
    if p[0]!=b'DHS1' or p[1]!=HOME_CONTRACT_VERSION or p[2] not in range(9) or p[3] not in range(3) or p[4] not in range(19) or p[5]==0 or p[19] not in (0,*MOTOR_IDS) or p[20] not in range(len(HOME_STAGES)):
        raise ValueError('invalid home snapshot')
    masks=p[12:18]
    if any(mask & ~ALL_MASK for mask in masks):raise ValueError('invalid home masks')
    targets=list(p[21:36]);goals=list(p[36:51]);positions=list(p[51:66]);modes=list(p[66:81]);torques=list(p[81:96])
    if p[2] in (2,3,4,5) and (not p[9] or not p[10] or p[12]!=ALL_MASK):raise ValueError('incomplete active home identity')
    return dict(raw_payload_hex=bytes(payload).hex(),version=p[1],phase=HOME_PHASES[p[2]],phase_code=p[2],result=p[3],error=p[4],
                boot_id=p[5],sequence=p[6],sample_us=p[7],operation_ack_seq=p[8],session=p[9],expires_us=p[10],
                native_sha256=p[11].hex(),target_mask=p[12],fresh_mask=p[13],arrived_mask=p[14],off_mask=p[15],
                primed_mask=p[16],enabled_mask=p[17],steps=p[18],active_id=p[19],stage=p[20],stage_name=HOME_STAGES[p[20]],
                targets=targets,goals=goals,positions=positions,modes=modes,torques=torques)
