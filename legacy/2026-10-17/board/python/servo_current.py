import struct
import calibration_wire as calibration

PACKET = struct.Struct('<4sBBH3I15H15H')
assert PACKET.size == 80


class ServoCurrent:
    def __init__(self, owner):
        self.owner = owner
        self.last_sequence = None
        self.values = {}

    def reset(self):
        self.last_sequence = None
        self.values.clear()

    def receive(self, payload):
        if not isinstance(payload, (bytes, bytearray)) or len(payload) != PACKET.size:
            return
        fields = PACKET.unpack(payload)
        if fields[0] != b'DIC1' or fields[1] != 1 or fields[2] or fields[3] & ~calibration.ALL_MASK:
            return
        with self.owner.lock:
            if not self.owner.snapshot or fields[4] != self.owner.snapshot['boot_id']:
                return
            if self.last_sequence is not None and not 0 < calibration.elapsed(fields[5], self.last_sequence) < 0x80000000:
                return
            self.last_sequence = fields[5]
            source_gap = calibration.elapsed(self.owner.snapshot['sample_us'], fields[6])
            source_gap_ms = source_gap / 1000 if source_gap < 0x80000000 else 0
            for index, sid in enumerate(calibration.MOTOR_IDS):
                age = fields[22 + index]
                if fields[3] & (1 << index) and age <= 150:
                    self.values[sid] = (fields[7 + index], age + source_gap_ms, self.owner.clock(), fields[4])

    def row(self, sid):
        value = self.values.get(sid)
        valid_boot = bool(value and self.owner.snapshot and value[3] == self.owner.snapshot['boot_id'])
        raw = value[0] if valid_boot else None
        age = max(0, value[1] + (self.owner.clock() - value[2]) * 1000) if valid_boot else None
        return {'current_raw': raw, 'current_ma': calibration.signed_word(raw) * 6.5 if raw is not None else None,
                'current_age_ms': round(age, 2) if age is not None else None,
                'current_fresh': bool(age is not None and age <= 500),
                'current_source': 'servo_phase_current/register69', 'current_unit': 'mA',
                'current_calibrated': False}

    @staticmethod
    def metadata():
        return {'source': 'servo_phase_current/register69', 'register': 69, 'width_bytes': 2,
                'encoding': 'HLS-family sign-magnitude bit15, 6.5mA per unit', 'unit': 'mA',
                'calibrated': False, 'input_max_age_ms': 150, 'display_ttl_ms': 500,
                'used_for_control': False, 'scope': 'Servo phase current; not bus supply current or register60 load'}
