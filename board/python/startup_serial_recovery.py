"""One first-Host-per-Linux-boot recovery of the fixed Router UART, never motion.

Only the official v0.5.3 $/serial/close and $/serial/open requests are encoded.
There is no MCU reset, Router restart, motor RPC, socket replacement or retry.
"""
from device_config import DEVICE_SERIAL
import json
import math
import os
from pathlib import Path
import re
import socket
import stat
import tempfile
import threading
import time

SERIAL = DEVICE_SERIAL
SOCKET_PATH = '/var/run/arduino-router.sock'
UART_PATH = '/dev/ttyHS1'
GUARD_FILE = 'startup_serial_guard.json'
LOCK_FILE = 'startup_serial_guard.lock'
BOOT_RE = re.compile(r'[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}')
START_DELAY_S = 8.0
START_WINDOW_S = 30.0


class StartupRecoveryBusy(RuntimeError):
    pass


def claim_first_host(data_dir, *, boot_id=None, serial=None, lock_module=None):
    """Durably consume this Linux boot before any recovery is eligible.

    The deployment seeds the record with the current boot, so installation or
    an ordinary Host restart cannot claim to be a new cold boot. A stable flock
    inode protects the atomic JSON replacement against concurrent Host starts.
    Missing, corrupt or inaccessible files disable recovery; never auto-seed.
    """
    if lock_module is None:
        import fcntl as lock_module
    directory = Path(data_dir)
    guard = directory / GUARD_FILE
    lock_path = directory / LOCK_FILE
    if directory.is_symlink() or guard.is_symlink() or not guard.is_file():
        raise ValueError('startup guard missing or invalid')
    if lock_path.is_symlink():
        raise ValueError('startup guard lock invalid')
    if boot_id is None:
        boot_id = Path('/proc/sys/kernel/random/boot_id').read_text('utf-8').strip()
    if serial is None:
        serial = Path('/sys/devices/soc0/serial_number').read_text('utf-8').strip()
    if serial != SERIAL or not isinstance(boot_id, str) or not BOOT_RE.fullmatch(boot_id):
        raise ValueError('startup identity mismatch')
    descriptor = os.open(lock_path, os.O_RDWR | os.O_CREAT | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    temporary = None
    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise ValueError('startup guard lock must be a regular file')
        lock_module.flock(descriptor, lock_module.LOCK_EX | lock_module.LOCK_NB)
        if guard.is_symlink() or guard.stat().st_size > 1024:
            raise ValueError('startup guard invalid')
        record = json.loads(guard.read_text('utf-8'))
        if (not isinstance(record, dict) or set(record) != {'version', 'serial', 'last_linux_boot_id'}
                or type(record['version']) is not int or record['version'] != 1
                or record['serial'] != SERIAL or not isinstance(record['last_linux_boot_id'], str)
                or not BOOT_RE.fullmatch(record['last_linux_boot_id'])):
            raise ValueError('startup guard invalid')
        if record['last_linux_boot_id'] == boot_id:
            return False
        record['last_linux_boot_id'] = boot_id
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=directory,
                                         prefix='.startup-serial-', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(json.dumps(record, separators=(',', ':')) + '\n')
            stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, guard)
        temporary = None
        # Make the claim durable before allowing a serial recovery.
        parent = os.open(directory, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
        try:
            os.fsync(parent)
        finally:
            os.close(parent)
        return True
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
        os.close(descriptor)


def request_bytes(method, request_id):
    expected_id = {'$/serial/close': 1, '$/serial/open': 2}.get(method)
    if type(request_id) is not int or request_id != expected_id:
        raise ValueError('Only the fixed serial close/open requests are supported')
    name, argument = method.encode('ascii'), UART_PATH.encode('ascii')
    return b'\x94\x00' + bytes((request_id, 0xa0 + len(name))) + name + b'\x91' + bytes((0xa0 + len(argument),)) + argument


class SerialLink:
    def __init__(self, *, socket_factory=None, clock=time.monotonic, stat_path=os.stat,
                 environment=None):
        self.socket_factory = socket_factory or (lambda: socket.socket(socket.AF_UNIX, socket.SOCK_STREAM))
        self.clock, self.stat_path = clock, stat_path
        self.environment = os.environ if environment is None else environment

    def identity(self):
        if self.environment.get('APP_SOCKET', 'unix://' + SOCKET_PATH) != 'unix://' + SOCKET_PATH:
            raise ValueError('Only the existing fixed Router socket is supported')
        info = self.stat_path(SOCKET_PATH, follow_symlinks=False)
        if not stat.S_ISSOCK(info.st_mode):
            raise ValueError('Router path must be an existing socket')
        return info.st_dev, info.st_ino

    def operation(self, method, request_id):
        payload = request_bytes(method, request_id)
        expected = b'\x94\x01' + bytes((request_id,)) + b'\xc0\xc3'
        deadline = self.clock() + 1.5
        stream = self.socket_factory()
        def remaining():
            value = deadline - self.clock()
            if value <= 0:
                raise TimeoutError('Startup serial operation deadline')
            stream.settimeout(value)
        try:
            remaining(); stream.connect(SOCKET_PATH)
            remaining(); stream.sendall(payload)
            reply = b''
            while len(reply) < len(expected):
                remaining(); chunk = stream.recv(6 - len(reply))
                if not chunk:
                    raise ConnectionError('Router closed before serial reply')
                reply += chunk
            if reply != expected or self.clock() >= deadline:
                raise ValueError('Unexpected or late serial response')
            return True
        finally:
            stream.close()

    def cycle(self):
        identity = self.identity()
        close_ok = False
        close_error = None
        # Open once even if the close response was lost: the close may have
        # executed. This is the second half of one cycle, never a close retry.
        try:
            close_ok = self.operation('$/serial/close', 1)
        except (OSError, ValueError) as error:
            close_error = type(error).__name__
        if self.identity() != identity:
            raise ValueError('Router socket identity changed; no request sent to replacement')
        open_ok = self.operation('$/serial/open', 2)
        if self.identity() != identity:
            raise ValueError('Router socket identity changed after serial open')
        if not close_ok:
            raise RuntimeError('Serial close was not confirmed: ' + str(close_error))
        return bool(open_ok)


class StartupSerialRecovery:
    def __init__(self, host, telemetry, data_dir, *, clock=time.monotonic,
                 claim=claim_first_host, cycle=None):
        self.host, self.telemetry, self.clock = host, telemetry, clock
        self.started = clock()
        self.lock = threading.Lock()
        self.feedback = threading.Event()
        self.attempts = 0
        self.busy = False
        self.cycle = cycle or SerialLink().cycle
        try:
            self.eligible = claim(data_dir) is True
            self.state = 'startup_wait' if self.eligible else 'disabled_same_linux_boot'
        except (OSError, ValueError, ImportError) as error:
            self.eligible = False
            self.state = 'disabled_guard_' + type(error).__name__

    def wrap_feedback(self, callback):
        def receive(*args, **kwargs):
            self.feedback.set()  # Any Bridge report ends the zero-feedback case.
            return callback(*args, **kwargs)
        return receive

    def before_action(self):
        with self.lock:
            self.eligible = False  # Permanent for this Host; guard already claims this boot.
            if self.busy:
                raise StartupRecoveryBusy('Startup serial recovery is in progress; no action was sent')
            if self.state == 'startup_wait':
                self.state = 'disabled_action_seen'

    def cancel(self):
        with self.lock:
            self.eligible = False
            if self.state == 'startup_wait':
                self.state = 'disabled_shutdown'

    def clean_start(self):
        with self.host.lock:
            host_clean = (self.host.snapshot is None and self.host.observed_at is None
                and self.host.sent == 0 and self.host.request_seq == 0
                and self.host.sender.attempted == 0 and self.host.sender.queue.unfinished_tasks == 0
                and self.host._session is None and self.host._home_session is None
                and self.host._home_snapshot is None and not self.host._home_unknown
                and self.host.operation is None and self.host._capture_owner is None
                and not self.host.requests and not self.host.results and not self.host.seen_boots)
        with self.telemetry.lock:
            telemetry_clean = (self.telemetry.received == 0 and self.telemetry.feedback is None
                and self.telemetry.raw is None and self.telemetry.servo_reply is None
                and not self.telemetry.boot_pd and not self.telemetry.probe_reports
                and not self.telemetry.mcu_enabled_seen)
        return host_clean and telemetry_clean and not self.feedback.is_set()

    def tick(self):
        with self.lock:
            if not self.eligible or self.busy:
                return
            elapsed = self.clock() - self.started
            if not math.isfinite(elapsed) or elapsed < 0 or elapsed > START_WINDOW_S:
                self.eligible = False; self.state = 'disabled_startup_window'
                return
            if not self.clean_start():
                self.eligible = False; self.state = 'disabled_feedback_or_activity'
                return
            if elapsed < START_DELAY_S:
                return
            self.eligible = False; self.busy = True; self.attempts = 1
            self.state = 'serial_recovery_running'
        outcome = 'serial_cycle_failed_internal'
        try:
            # No locks span the socket round trips. Reports continue to update
            # the ordinary safety state; incoming action requests fail promptly.
            if self.feedback.is_set() or not self.clean_start():
                outcome = 'disabled_feedback_or_activity'
            elif self.cycle() is True:
                outcome = 'serial_cycle_confirmed_waiting_feedback'
            else:
                outcome = 'serial_cycle_unconfirmed'
        except Exception as error:
            outcome = 'serial_cycle_failed_' + type(error).__name__
        finally:
            with self.lock:
                self.busy = False
                self.state = outcome

    def run(self, stop):
        while not stop.wait(.1):
            self.tick()
            with self.lock:
                if not self.eligible and not self.busy:
                    return

    def snapshot(self):
        with self.lock:
            state = self.state
            if state == 'serial_cycle_confirmed_waiting_feedback' and self.feedback.is_set():
                state = 'serial_cycle_confirmed_feedback_seen'
            return {'state': state, 'attempts': self.attempts, 'busy': self.busy,
                    'scope': 'First Host of a new Linux boot; zero-feedback serial link only',
                    'automatic_action_retry': False, 'motion_resumed': False}
