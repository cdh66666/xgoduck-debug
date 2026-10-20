"""Read-only servo phase-current recorder; never part of the motion loop.

This is NOT battery input current. The user's approximately 0.4 A stationary
power-supply observation is contextual information only, never an offset.
The receive callback only submits bounded queue items; disk I/O and decoding
of display summaries happen on this independent daemon thread.
"""
import collections
import queue
import struct
import threading
import time
from pathlib import Path
from wire import ALL_MASK, CURRENT, STATE, decode_current

FILE_HEADER = struct.Struct('<8sHHIQQ')
RECORD = struct.Struct('<4sHHQQ312s')
IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
MAX_FILE_BYTES = 8 * 1024 * 1024
FILE_COUNT = 8


def signed_amperes(raw):
    return (-1 if raw & 0x8000 else 1) * (raw & 0x7fff) * 0.0065


class CurrentTelemetry:
    def __init__(self, directory=None, *, queue_size=2048, file_bytes=MAX_FILE_BYTES,
                 file_count=FILE_COUNT, clock=time.monotonic, start_writer=True):
        self.directory = Path(directory) if directory is not None else Path(__file__).resolve().parents[1] / 'data' / 'servo-current'
        if not RECORD.size + FILE_HEADER.size <= file_bytes <= MAX_FILE_BYTES or not 1 <= file_count <= FILE_COUNT:
            raise ValueError('invalid bounded current log size')
        if not 1 <= queue_size <= 2048:
            raise ValueError('current log queue must be bounded to 1..2048 items')
        self.queue = queue.Queue(maxsize=queue_size)
        self.clock = clock
        self.file_bytes, self.file_count = file_bytes, file_count
        self.queue_drops = self.callback_errors = 0
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._boot = self._seq = None
        self._retired_boots = collections.deque(maxlen=16)
        self._full_clock_us = 0
        self._last_full_start = None
        self._last_full_host_time = None
        self._last_full_abs = None
        self._full_times = collections.deque(maxlen=1000)
        self._attempt_clock_us = 0
        self._last_attempt_start = None
        self._attempt_times = collections.deque(maxlen=1000)
        self._counts = dict(frames=0, legacy_frames=0, full=0, partial=0, unattempted=0,
                            attempts=0, duplicates=0, out_of_order=0, sequence_gaps=0,
                            boot_changes=0, disk_errors=0, records_written=0, malformed=0)
        self._stream = None
        self._bytes = 0
        self._slot = None
        self._disk_failed = False
        self._disk_error = ''
        self._thread = threading.Thread(target=self._run, name='servo-current-log', daemon=True)
        if start_writer:
            self._thread.start()

    def submit(self, raw, fb, host_time):
        """No disk, blocking wait or controller lock; accepted Feedback is unchanged."""
        try:
            self.queue.put_nowait((bytes(raw), fb.seq, host_time, time.time_ns()))
        except queue.Full:
            self.queue_drops += 1
        except Exception:
            self.callback_errors += 1

    @staticmethod
    def _rate(times):
        if len(times) < 2:
            return None
        span = times[-1] - times[0]
        return (len(times) - 1) * 1e6 / span if span > 0 else None

    def _consume(self, item):
        raw, seq, host_time, wall_ns = item
        if len(raw) == STATE.size:
            with self._lock:
                self._counts['legacy_frames'] += 1
            return
        try:
            value = decode_current(raw[STATE.size:], seq)
        except ValueError:
            with self._lock:
                self._counts['malformed'] += 1
            return
        with self._lock:
            boot = value['boot']
            if boot != self._boot:
                if boot in self._retired_boots:
                    self._counts['out_of_order'] += 1
                    return
                if self._boot is not None:
                    self._retired_boots.append(self._boot)
                    self._counts['boot_changes'] += 1
                self._boot, self._seq = boot, None
                self._full_times.clear(); self._attempt_times.clear()
                self._last_full_start = self._last_attempt_start = None
                self._last_full_host_time = self._last_full_abs = None
                self._full_clock_us = self._attempt_clock_us = 0
            if self._seq is not None:
                delta = (seq - self._seq) & 0xffffffff
                if delta == 0:
                    self._counts['duplicates'] += 1
                    return
                if delta >= 0x80000000:
                    self._counts['out_of_order'] += 1
                    return
                self._counts['sequence_gaps'] += delta - 1
            self._seq = seq
            self._counts['frames'] += 1
            amps = [signed_amperes(v) if value['mask'] & (1 << i) else None for i, v in enumerate(value['raw'])]
            full = value['mask'] == ALL_MASK
            if value['attempted']:
                self._counts['attempts'] += 1
                self._counts['full' if full else 'partial'] += 1
                if self._last_attempt_start is not None:
                    self._attempt_clock_us += (value['start_us'] - self._last_attempt_start) & 0xffffffff
                self._last_attempt_start = value['start_us']
                self._attempt_times.append(self._attempt_clock_us)
                if full:
                    self._last_full_host_time = host_time
                    self._last_full_abs = sum(abs(v) for v in amps)
                    if self._last_full_start is not None:
                        self._full_clock_us += (value['start_us'] - self._last_full_start) & 0xffffffff
                    self._last_full_start = value['start_us']
                    self._full_times.append(self._full_clock_us)
            else:
                self._counts['unattempted'] += 1
            self._latest = dict(value, host_time=host_time, per_servo_signed_a=amps,
                                sum_abs_a=sum(abs(v) for v in amps) if full else None,
                                sum_signed_a=sum(amps) if full else None,
                                sample_span_us=(value['end_us'] - value['start_us']) & 0xffffffff)
        if not self._disk_failed:
            try:
                self._write(RECORD.pack(b'DQIR', 1, 0, int(host_time * 1e9), wall_ns, raw))
            except Exception as exc:
                self._disk_failed = True
                with self._lock:
                    self._counts['disk_errors'] += 1
                    self._disk_error = type(exc).__name__ + ': ' + str(exc)
                self._close_stream()

    def _write(self, data):
        if self._stream is None or self._bytes + len(data) > self.file_bytes:
            self._close_stream()
            if self._disk_failed:
                return
            self.directory.mkdir(parents=True, exist_ok=True)
            if self._slot is None:
                slots = [(self.directory / ('current-%02d.bin' % i)) for i in range(self.file_count)]
                self._slot = min(range(self.file_count), key=lambda i: slots[i].stat().st_mtime_ns if slots[i].exists() else -1)
            else:
                self._slot = (self._slot + 1) % self.file_count
            path = self.directory / ('current-%02d.bin' % self._slot)
            self._stream = path.open('wb', buffering=64 * 1024)
            header = FILE_HEADER.pack(b'DQICLOG1', 1, FILE_HEADER.size, RECORD.size, time.time_ns(), time.monotonic_ns())
            self._stream.write(header)
            self._bytes = len(header)
        self._stream.write(data)
        self._bytes += len(data)
        with self._lock:
            self._counts['records_written'] += 1

    def _close_stream(self):
        if self._stream is not None:
            try:
                self._stream.close()
            except Exception as exc:
                self._disk_failed = True
                with self._lock:
                    self._counts['disk_errors'] += 1
                    self._disk_error = type(exc).__name__ + ': ' + str(exc)
            self._stream = None

    def _run(self):
        while not self._stop.is_set() or not self.queue.empty():
            # Queue arrivals do not wake this thread. Consume at most 128 raw
            # records per 20 Hz batch, keeping the original nonblocking submit.
            self._stop.wait(.05)
            for _ in range(128):
                try:
                    item = self.queue.get_nowait()
                except queue.Empty:
                    break
                try:
                    self._consume(item)
                except Exception as exc:
                    # A monitoring failure must never propagate to motion receive.
                    with self._lock:
                        self._counts['malformed'] += 1
                        self._disk_error = type(exc).__name__ + ': ' + str(exc)
                finally:
                    self.queue.task_done()
            if self._stream is not None:
                try:
                    self._stream.flush()
                except Exception as exc:
                    self._disk_failed = True
                    with self._lock:
                        self._counts['disk_errors'] += 1
                        self._disk_error = type(exc).__name__ + ': ' + str(exc)
                    self._close_stream()
        self._close_stream()

    def snapshot(self, now=None):
        now = self.clock() if now is None else now
        with self._lock:
            latest = dict(self._latest) if self._latest is not None else None
            counts = dict(self._counts)
            source_hz = self._rate(self._full_times)
            attempted_hz = self._rate(self._attempt_times)
            disk_error = self._disk_error
            full_age = None if self._last_full_host_time is None else max(0., now - self._last_full_host_time)
            last_full_abs = self._last_full_abs
            if full_age is None or full_age > .15:
                source_hz = None
        result = dict(source='servo_phase_current', battery_input_current=False,
                      calibrated=False, scale_source='family_register_reference_unverified_on_this_servo',
                      units='A', ids=list(IDS), scale_a_per_unit=.0065,
                      stationary_supply_observation_a=.4, observation_source='user_power_supply_approximate_not_offset',
                      queue_drops=self.queue_drops, callback_errors=self.callback_errors,
                      queue_depth=self.queue.qsize(), counts=counts, source_hz=source_hz,
                      attempted_hz=attempted_hz, disk_error=disk_error,
                      log_directory=str(self.directory), file_bytes=self._bytes,
                      max_total_bytes=self.file_bytes * self.file_count,
                      full_sample_age_ms=None if full_age is None else full_age * 1000,
                      logging=not self._disk_failed and not self._stop.is_set(),
                      stale=True, age_ms=None, sum_abs_a=None, sum_signed_a=None,
                      supported=latest is not None, available=False,
                      full_samples=counts['full'], partial_samples=counts['partial'],
                      skipped_samples=latest['skipped'] if latest else None,
                      sequence_gaps=counts['sequence_gaps'], log_error=disk_error,
                      last_sum_abs_a=last_full_abs,
                      per_servo_signed_a=[None] * 15)
        if latest is not None:
            age = max(0.0, now - latest.pop('host_time'))
            result.update(latest)
            result.pop('raw', None)
            result.update(stale=age > .15, age_ms=age * 1000)
            if age > .15:
                result.update(sum_abs_a=None, sum_signed_a=None, per_servo_signed_a=[None] * 15,
                              source_hz=None, attempted_hz=None)
            result['available'] = not result['stale'] and result['mask'] == ALL_MASK
        result['per_servo_a'] = {str(i): v for i, v in zip(IDS, result['per_servo_signed_a'])}
        return result

    def close(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=.25)
        else:
            self._close_stream()
