"""Independent, pinned-SSH copy of the Host's passive current log files."""
import base64
import csv
import hashlib
import json
import io
from pathlib import Path
import shlex
import struct
import subprocess
import threading
import time

from model_transport import BOOTSTRAP, _configuration

HEADER = struct.Struct('<8sHHIQQ')
RECORD_SIZE = 336
RECORD = struct.Struct('<4sHHQQ312s')
CURRENT = struct.Struct('<4sBBH7I15HHI')
STATE = struct.Struct('<4sBBH10I6h45f')
IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)


def export_csv(folder, limit=20000):
    """Export the newest recorded generation; no access to the robot."""
    candidates = []
    for path in Path(folder).glob('current-??-????????????????.bin'):
        with path.open('rb') as stream:
            header = stream.read(32)
        if len(header) == 32 and HEADER.unpack(header)[:4] == (b'DQICLOG1', 1, 32, RECORD_SIZE):
            candidates.append((HEADER.unpack(header)[4], path))
    if not candidates:
        raise FileNotFoundError('尚无已保存的舵机电流记录')
    path = max(candidates)[1]
    output = io.StringIO(newline='')
    writer = csv.writer(output)
    writer.writerow(['host_unix_s', 'host_monotonic_s', 'mcu_boot', 'mcu_seq', 'current_start_us',
                     'current_end_us', 'attempted', 'current_mask', 'complete_15',
                     'servo_sum_abs_a_not_battery_total', 'servo_sum_signed_a',
                     *['id_%s_a' % sid for sid in IDS], *['id_%s_raw' % sid for sid in IDS],
                     'enabled', 'command_seq', 'mcu_period_us', 'mcu_work_us', 'mcu_overruns',
                     'mcu_report_drops', 'current_attempts', 'current_skipped'])
    with path.open('rb') as stream:
        count = (path.stat().st_size - 32) // RECORD_SIZE
        stream.seek(32 + max(0, count - limit) * RECORD_SIZE)
        for _ in range(min(count, limit)):
            raw = stream.read(RECORD_SIZE)
            if len(raw) != RECORD_SIZE:
                break
            magic, version, reserved, monotonic_ns, wall_ns, frame = RECORD.unpack(raw)
            if (magic, version, reserved) != (b'DQIR', 1, 0):
                raise ValueError('已保存记录格式不匹配')
            current, state = CURRENT.unpack(frame[240:]), STATE.unpack(frame[:240])
            if current[:2] != (b'DQI1', 1) or current[26] or current[6] != state[4]:
                raise ValueError('电流帧与姿态帧不匹配')
            values = [((-1 if value & 0x8000 else 1) * (value & 0x7fff) * .0065)
                      if current[3] & (1 << index) else None for index, value in enumerate(current[11:26])]
            complete = current[3] == 32767
            writer.writerow([wall_ns / 1e9, monotonic_ns / 1e9, '%08x%08x' % (current[5], current[4]),
                current[6], current[7], current[8], int(bool(current[2] & 1)), current[3], int(complete),
                sum(abs(value) for value in values) if complete else '', sum(values) if complete else '',
                *[value if value is not None else '' for value in values],
                *[value if current[3] & (1 << index) else '' for index, value in enumerate(current[11:26])],
                int(bool(state[2] & 2)), state[7], state[9], state[10], state[11], state[12], current[9], current[10]])
    return ('\ufeff' + output.getvalue()).encode('utf-8')


def current_series(folder, seconds=60, limit=6000):
    """Build a plot from local original samples, with no board/network access."""
    files = []
    for path in Path(folder).glob('current-??-????????????????.bin'):
        with path.open('rb') as stream:
            header = stream.read(32)
        if len(header) == 32 and HEADER.unpack(header)[:4] == (b'DQICLOG1', 1, 32, RECORD_SIZE):
            files.append((HEADER.unpack(header)[4], path))
    selected, remaining = [], limit
    for _, path in sorted(files, reverse=True):
        with path.open('rb') as stream:
            count = max(0, (path.stat().st_size - 32) // RECORD_SIZE)
            take = min(count, remaining)
            stream.seek(32 + (count - take) * RECORD_SIZE)
            selected.append(stream.read(take * RECORD_SIZE))
        remaining -= take
        if remaining <= 0:
            break
    samples = []
    for block in reversed(selected):
        for offset in range(0, len(block) - RECORD_SIZE + 1, RECORD_SIZE):
            magic, version, reserved, mono, wall, frame = RECORD.unpack_from(block, offset)
            if (magic, version, reserved) != (b'DQIR', 1, 0):
                raise ValueError('电流记录格式不匹配')
            x = CURRENT.unpack_from(frame, 240)
            if x[:2] != (b'DQI1', 1) or x[26] or x[3] & ~32767:
                raise ValueError('电流采样格式不匹配')
            values = [((-1 if value & 0x8000 else 1) * (value & 0x7fff) * .0065)
                      if x[3] & (1 << index) else None for index, value in enumerate(x[11:26])]
            samples.append(dict(t=wall / 1e9, seq=x[6], boot='%08x%08x' % (x[5], x[4]),
                mask=x[3], attempted=bool(x[2] & 1), start_us=x[7],
                sum_abs_a=sum(abs(v) for v in values) if x[3] == 32767 else None, values=values))
    samples.sort(key=lambda row: row['t'])
    latest = samples[-1]['t'] if samples else None
    if latest is not None:
        samples = [row for row in samples if row['t'] >= latest - seconds]
    full = [row for row in samples if row['mask'] == 32767]
    same_boot = [row for row in full if row['boot'] == full[-1]['boot']] if full else []
    span = ((same_boot[-1]['start_us'] - same_boot[0]['start_us']) & 0xffffffff) if len(same_boot) > 1 else 0
    hz = (len(same_boot) - 1) * 1e6 / span if 0 < span < 0x80000000 else None
    return dict(ok=True, source='servo_feedback', battery_total=False, calibrated=False,
        sample_hz=hz, samples=samples, latest_recorded_unix_s=latest,
        age_s=max(0, time.time() - latest) if latest is not None else None,
        stale=latest is None or time.time() - latest > 2,
        full_samples=len(full), partial_samples=sum(row['attempted'] and row['mask'] != 32767 for row in samples),
        skipped_samples=sum(not row['attempted'] for row in samples), ids=list(IDS))
REMOTE = r'''
import base64,json,os,struct,sys,time
from pathlib import Path
p=json.load(sys.stdin)
os.nice(19)
folder=Path('/home/arduino/ArduinoApps/xgoduck-commissioning/data/servo-current')
positions={}
for item in p.get('resume',[]):
 slot,offset=item['slot'],item['offset'];header=bytes.fromhex(item['header'])
 if slot in range(8) and len(header)==32 and header[:8]==b'DQICLOG1' and 32<=offset<=8*1024*1024 and (offset-32)%336==0:
  positions[(slot,header)]=offset
while True:
 candidates=[]
 for slot in range(8):
  path=folder/('current-%02d.bin'%slot)
  try:
   with path.open('rb') as stream:header=stream.read(32)
   if len(header)==32 and header[:8]==b'DQICLOG1':candidates.append((struct.unpack_from('<Q',header,16)[0],slot))
  except (FileNotFoundError,OSError):pass
 budget=64
 for _,slot in sorted(candidates,reverse=True):
  if budget<=0:break
  path=folder/('current-%02d.bin'%slot)
  try:
   with path.open('rb') as stream:
    header=stream.read(32)
    if len(header)!=32 or header[:8]!=b'DQICLOG1':continue
    key=(slot,header)
    offset=positions.get(key,32)
    stream.seek(offset)
    raw=stream.read(336*budget)
    raw=raw[:len(raw)//336*336]
    stream.seek(0)
    if stream.read(32)!=header:continue
   if raw:
    print(json.dumps(dict(slot=slot,header=header.hex(),offset=offset,
                          data=base64.b64encode(raw).decode())),flush=True)
    positions={k:v for k,v in positions.items() if k[0]!=slot}
    positions[key]=offset+len(raw)
    budget-=len(raw)//336
  except (FileNotFoundError,OSError):pass
 print('{"alive":true}',flush=True)
 time.sleep(.1)
'''


class CurrentRecorder:
    def __init__(self, root, *, start=True):
        self.root = Path(root).resolve()
        self.folder = self.root / 'current-recordings'
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.process = None
        self.received_at = 0.0
        self.bytes_received = self.blocks = 0
        self.error = '等待电流记录连接'
        self.worker = threading.Thread(target=self._run, name='servo-current-copy', daemon=True)
        if start:
            self.worker.start()

    def _args(self):
        identity, address, executable, null_config = _configuration(self.root)
        return [str(executable), '-F', null_config, '-T', '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
                '-o', 'IdentityAgent=none', '-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no',
                '-o', 'VerifyHostKeyDNS=no', '-o', 'CheckHostIP=no', '-o', 'PasswordAuthentication=no',
                '-o', 'KbdInteractiveAuthentication=no', '-o', 'ConnectTimeout=3', '-o', 'ConnectionAttempts=1',
                '-o', 'ServerAliveInterval=1', '-o', 'ServerAliveCountMax=3',
                '-o', 'Compression=no',
                '-o', 'GlobalKnownHostsFile=' + null_config,
                '-o', 'UserKnownHostsFile=' + str(identity['known_hosts_path']),
                '-o', 'HostKeyAlias=' + identity['host_key_alias'], '-i', str(identity['key_path']),
                'arduino@' + address, 'python3 -u -c ' + shlex.quote(BOOTSTRAP.replace(repr('__PORTABLE_DEVICE_SERIAL__'),repr(identity['serial'])))]

    def resume_positions(self):
        result=[]
        for path in self.folder.glob('current-??-????????????????.bin'):
            with path.open('rb') as stream:
                header=stream.read(32)
            if len(header)!=32 or HEADER.unpack(header)[:4]!=(b'DQICLOG1',1,32,RECORD_SIZE):
                continue
            result.append(dict(slot=int(path.name[8:10]),header=header.hex(),
                               offset=32+max(0,(path.stat().st_size-32)//RECORD_SIZE)*RECORD_SIZE))
        return result

    def accept(self, value):
        if value.get('alive') is True:
            return
        slot, offset = value['slot'], value['offset']
        if type(slot) is not int or slot not in range(8) or type(offset) is not int or offset < 32:
            raise ValueError('Invalid current log position')
        header = bytes.fromhex(value['header'])
        if len(header) != HEADER.size:
            raise ValueError('Invalid current log header length')
        magic, version, size, record_size, _, _ = HEADER.unpack(header)
        if (magic, version, size, record_size) != (b'DQICLOG1', 1, 32, RECORD_SIZE):
            raise ValueError('Unknown current log format')
        raw = base64.b64decode(value['data'], validate=True)
        if (not raw or len(raw) > RECORD_SIZE * 64 or len(raw) % RECORD_SIZE
                or (offset - 32) % RECORD_SIZE or offset + len(raw) > 8 * 1024 * 1024):
            raise ValueError('Invalid current log block')
        for index in range(0, len(raw), RECORD_SIZE):
            if raw[index:index + 8] != b'DQIR\x01\x00\x00\x00':
                raise ValueError('Invalid current log record')
        self.folder.mkdir(exist_ok=True)
        name = 'current-%02d-%s.bin' % (slot, hashlib.sha256(header).hexdigest()[:16])
        path = self.folder / name
        with path.open('r+b' if path.exists() else 'w+b') as stream:
            old = stream.read(32)
            if old and old != header:
                raise ValueError('Current log generation changed')
            stream.seek(0)
            stream.write(header)
            if offset > stream.seek(0, 2):
                raise ValueError('Missing current log prefix')
            stream.seek(offset)
            stream.write(raw)
            stream.flush()
        with self.lock:
            self.received_at = time.monotonic()
            self.bytes_received += len(raw)
            self.blocks += 1
            self.error = ''
        files = sorted(self.folder.glob('current-??-????????????????.bin'), key=lambda p: p.stat().st_mtime)
        total = sum(p.stat().st_size for p in files)
        for old in files:
            if total <= 256 * 1024 * 1024:
                break
            if old != path:
                total -= old.stat().st_size
                old.unlink()

    def _run(self):
        while not self.stop_event.is_set():
            process = None
            try:
                process = subprocess.Popen(self._args(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                self.process = process
                process.stdin.write(json.dumps({'source': REMOTE, 'payload': {'resume':self.resume_positions()}}).encode('utf-8'))
                process.stdin.close()
                while not self.stop_event.is_set():
                    line = process.stdout.readline(65537)
                    if not line:
                        raise ConnectionError('电流记录连接中断，已保存数据保留')
                    if len(line) > 65536:
                        raise ValueError('电流记录块超过上限')
                    self.accept(json.loads(line))
            except (OSError, ValueError, KeyError, TypeError, ConnectionError, RuntimeError) as error:
                with self.lock:
                    self.error = str(error)
            finally:
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                    process.stdout.close()
                self.process = None
            self.stop_event.wait(1)

    def snapshot(self):
        with self.lock:
            age = time.monotonic() - self.received_at if self.received_at else None
            return dict(recording=age is not None and age < 2, received_age_s=age,
                        copied_bytes=self.bytes_received, copied_blocks=self.blocks,
                        directory=str(self.folder), error=self.error,
                        source='servo_phase_current/register69', battery_total=False)

    def close(self):
        self.stop_event.set()
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
        if self.worker.is_alive():
            self.worker.join(timeout=4)
