"""Opt-in host-only SSE -> authenticated UDP observation relay, standard library.

AppCLI still owns the bridge-network application. The sole upstream operation is
GET http://127.0.0.1:9527/api/live; no redirects, proxies, RPC or command route.
Container and host must share Linux CLOCK_MONOTONIC. source_monotonic is
sampled before the provider; checked_at is sampled after it. Known
age includes SSE queue, cache residence, MCU snapshot lag and downstream RTT;
unknown MCU-to-Linux pre-receive buffering remains outside this measured bound.
"""
from collections import Counter
from copy import deepcopy
import argparse
import http.client
import ipaddress
import json
import os
from pathlib import Path
import signal
import socket
import stat
import threading
import time

from imu_transport.host import HostEndpoint
from imu_transport.protocol import (HostProtocol, ProtocolError, MAX_AGE_MS,
                                    compact_status, finite, identity, derive_key)

UPSTREAM_HOST = '127.0.0.1'
UPSTREAM_PORT = 9527
UPSTREAM_PATH = '/api/live'
UDP_PORT = 9528
DATA_DIR = Path(__file__).resolve().parent.parent / 'data'
MAX_EVENT_BYTES = 65536
IO_TIMEOUT_S = .25
PRIVATE_NETWORKS = tuple(ipaddress.ip_network(x) for x in
                         ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))


def _private(value):
    try:
        address = ipaddress.IPv4Address(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError('Literal RFC1918 address required') from exc
    if str(address) != value or not any(address in network for network in PRIVATE_NETWORKS):
        raise ProtocolError('Literal RFC1918 address required')
    return value


def config_target(config, *, socket_factory=socket.socket):
    """Disabled is inert. Route selection sends no payload and performs no DNS."""
    if config is None:
        return None
    if not isinstance(config, dict) or type(config.get('enabled', False)) is not bool:
        raise ProtocolError('Explicit boolean enabled required')
    if not config.get('enabled', False):
        return None
    if set(config) != {'enabled', 'bind_ip', 'port'} or type(config['port']) is not int or config['port'] != UDP_PORT:
        raise ProtocolError('Relay requires enabled, bind_ip and fixed port 9528')
    address = config['bind_ip']
    if address == 'auto_private':
        with socket_factory(socket.AF_INET, socket.SOCK_DGRAM) as route:
            route.connect(('192.168.0.1', 9))
            address = route.getsockname()[0]
    return _private(address), UDP_PORT


def _read_bounded(path, maximum):
    """Read existing regular files only; do not create, chmod or follow links."""
    path = Path(path)
    if path.is_symlink() or path.parent.is_symlink():
        raise ValueError('Redirected configuration is not accepted')
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    descriptor = os.open(path, flags)
    with os.fdopen(descriptor, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > maximum:
            raise ValueError('Invalid configuration file')
        value = stream.read(maximum + 1)
    if len(value) > maximum:
        raise ValueError('Configuration exceeds limit')
    return value


def load_existing_key():
    key = _read_bounded(DATA_DIR / 'calibration_key', 128).decode('ascii').strip()
    derive_key(key)
    return key


def _json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError('Duplicate JSON field')
        result[key] = value
    return result


def strict_json(data):
    return json.loads(data, object_pairs_hook=_json_object,
                      parse_constant=lambda _: (_ for _ in ()).throw(ProtocolError('Nonfinite JSON')))


class LatestObservation:
    """One immutable latest frame; duplicates never renew cache age or progress."""
    def __init__(self, *, clock=time.monotonic, wall_clock=time.time):
        self.clock, self.wall_clock = clock, wall_clock
        self.lock = threading.Lock()
        self.latest = None
        self.source = None
        self.progress = None
        self.retired = set()
        self.stats = Counter()

    def invalidate(self):
        with self.lock:
            self.latest = None

    def ingest(self, raw):
        now = finite(self.clock(), 'relay monotonic time')
        if not isinstance(raw, dict):
            raise ProtocolError('SSE status object required')
        generated = finite(raw.get('source_monotonic'), 'source monotonic timestamp')
        checked = finite(raw.get('checked_at'), 'audit wall timestamp')
        if checked <= 0:
            raise ProtocolError('Invalid audit wall timestamp')
        mono_delta = now - generated
        wall_delta = finite(self.wall_clock(), 'relay wall timestamp') - checked
        # Same-kernel monotonic prevents NTP from reducing known age. A wall
        # jump may discard frames conservatively; the two delays are not summed.
        if mono_delta < -1e-6 or wall_delta < -1e-6:
            raise ProtocolError('Future SSE timestamp')
        queue_ms = max(mono_delta, wall_delta, 0.) * 1000
        if queue_ms >= MAX_AGE_MS:
            raise ProtocolError('Queued/future SSE observation')
        value = compact_status(raw)
        value['feedback_age_ms'] += queue_ms
        if value['feedback_age_ms'] + value['imu_snapshot_age_ms'] >= MAX_AGE_MS:
            raise ProtocolError('SSE observation exceeds known-age limit')
        source = identity(value)
        progress = tuple(value[name] for name in ('mcu_seq', 'mcu_sample_us', 'mcu_imu_us'))
        with self.lock:
            if source in self.retired:
                raise ProtocolError('Retired source identity')
            if source == self.source and self.progress is not None:
                if any(not 0 < ((new-old) & 0xffffffff) < 0x80000000
                       for new, old in zip(progress, self.progress)):
                    self.stats['duplicate_or_regressed'] += 1
                    return False
            elif self.source is not None:
                if len(self.retired) >= 64:
                    raise ProtocolError('Source identity history exhausted')
                self.retired.add(self.source)
            self.source, self.progress = source, progress
            self.latest = (value, now)
            self.stats['accepted'] += 1
            return True

    def live_status(self):
        with self.lock:
            if self.latest is None:
                raise ProtocolError('No current SSE observation')
            value, received = self.latest
            residence = (finite(self.clock(), 'relay monotonic time') - received) * 1000
            if residence < 0:
                self.latest = None
                raise ProtocolError('Relay clock regressed')
            result = deepcopy(value)
            result['feedback_age_ms'] += residence
            if result['feedback_age_ms'] + result['imu_snapshot_age_ms'] >= MAX_AGE_MS:
                self.latest = None
                raise ProtocolError('Relay cache is stale')
            return result


class SSEReader:
    """Bounded fixed GET reader. Worker failures clear cache and reconnect only GET."""
    def __init__(self, cache, *, connection_factory=http.client.HTTPConnection):
        self.cache, self.connection_factory = cache, connection_factory
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.connection = None
        self.thread = None
        self.stats = Counter()

    def start(self):
        if self.stop_event.is_set():
            raise RuntimeError('Reader closed')
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self._run, name='imu-relay-sse', daemon=True)
            self.thread.start()
        return self

    def _close_connection(self):
        with self.lock:
            connection, self.connection = self.connection, None
        if connection is not None:
            try:
                if connection.sock is not None:
                    connection.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            finally:
                connection.close()

    def _read_stream(self, response):
        data, size, event = [], 0, ''
        while not self.stop_event.is_set():
            line = response.readline(MAX_EVENT_BYTES + 1)
            if not line:
                raise EOFError('SSE disconnected')
            size += len(line)
            if size > MAX_EVENT_BYTES:
                raise ProtocolError('SSE event exceeds size limit')
            line = line.rstrip(b'\r\n')
            if not line:
                if data and event in ('', 'status'):
                    if self.cache.ingest(strict_json(b'\n'.join(data).decode('utf-8'))):
                        self.stats['accepted'] += 1
                data, size, event = [], 0, ''
            elif line.startswith(b'data:'):
                data.append(line[5:].lstrip(b' '))
            elif line.startswith(b'event:'):
                event = line[6:].lstrip(b' ').decode('ascii')

    def _run(self):
        retry = .05
        while not self.stop_event.is_set():
            accepted = self.stats['accepted']
            try:
                connection = self.connection_factory(UPSTREAM_HOST, UPSTREAM_PORT, timeout=IO_TIMEOUT_S)
                with self.lock:
                    self.connection = connection
                if self.stop_event.is_set():
                    break
                connection.request('GET', UPSTREAM_PATH, headers={'Accept': 'text/event-stream',
                                   'Accept-Encoding': 'identity', 'Cache-Control': 'no-cache'})
                response = connection.getresponse()
                if response.status != 200 or response.getheader('Content-Type', '').split(';')[0].strip() != 'text/event-stream':
                    raise ProtocolError('SSE endpoint unavailable')
                if response.getheader('Content-Encoding', 'identity') != 'identity':
                    raise ProtocolError('Compressed SSE is not supported')
                with response:
                    self._read_stream(response)
            except Exception:
                # Deliberate I/O worker boundary: neither a malformed upstream
                # nor an unexpected reader exception can strand a dead thread.
                # Never log exception text, URL parameters or credentials.
                self.stats['stream_failures'] += 1
            finally:
                self.cache.invalidate()
                self._close_connection()
            if self.stats['accepted'] > accepted:
                retry = .05
            if self.stop_event.wait(retry):
                break
            retry = min(1., retry * 2)

    def close(self):
        self.stop_event.set()
        self.cache.invalidate()
        self._close_connection()
        if self.thread is not None:
            self.thread.join(timeout=.6)


class RelayService:
    """Supervise both read-only workers; a dead/broken UDP socket is recreated."""
    def __init__(self, key, target, *, cache=None, reader=None, endpoint_factory=HostEndpoint, emit=None):
        self.key, self.target = key, target
        self.cache = cache if cache is not None else LatestObservation()
        self.reader = reader if reader is not None else SSEReader(self.cache)
        self.endpoint_factory, self.emit = endpoint_factory, emit or (lambda record: None)
        self.endpoint = None
        self.stop_event = threading.Event()
        self.thread = None

    def _bind(self):
        protocol = HostProtocol(self.key, self.cache.live_status)
        endpoint = self.endpoint_factory(protocol, self.target[1], bind=self.target[0], enabled=True)
        try:
            endpoint.start()
        except Exception:
            endpoint.close()
            raise
        self.endpoint = endpoint
        self.emit({'event': 'imu_relay_bound', 'bind_ip': endpoint.address[0], 'port': endpoint.address[1],
                   'read_only': True, 'upstream': 'http://127.0.0.1:9527/api/live'})

    def start(self):
        if self.thread is not None or self.stop_event.is_set():
            raise RuntimeError('Relay already started or closed')
        self._bind()
        self.reader.start()
        self.thread = threading.Thread(target=self._supervise, name='imu-relay-supervisor', daemon=True)
        self.thread.start()
        return self

    def _supervise(self):
        while not self.stop_event.wait(.1):
            try:
                self.reader.start()  # Detect dead reader threads without losing source progress.
                endpoint = self.endpoint
                if endpoint is not None and (not endpoint.thread.is_alive()
                        or endpoint.socket is None or endpoint.socket.fileno() < 0
                        or endpoint.protocol.stats['socket_error']):
                    endpoint.close()
                    self.endpoint = None
                if self.endpoint is None:
                    self._bind()
            except Exception:
                self.emit({'event': 'imu_relay_worker_retry', 'read_only': True})
                if self.stop_event.wait(.4):
                    break

    def close(self):
        self.stop_event.set()
        self.reader.close()
        if self.thread is not None:
            self.thread.join(timeout=.7)
        if self.endpoint is not None:
            self.endpoint.close()
            self.endpoint = None
        self.cache.invalidate()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DATA_DIR / 'imu_relay.json')
    args = parser.parse_args(argv)
    service = None
    old_handlers = {}
    emit = lambda record: print(json.dumps(record, separators=(',', ':')), flush=True)
    try:
        config = strict_json(_read_bounded(args.config, 4096).decode('utf-8')) if args.config.exists() else None
        target = config_target(config)
        if target is None:
            emit({'event': 'imu_relay_disabled', 'read_only': True})
            return 0
        key = load_existing_key()
        stopped = threading.Event()
        for number in (signal.SIGTERM, signal.SIGINT):
            old_handlers[number] = signal.signal(number, lambda *_: stopped.set())
        service = RelayService(key, target, emit=emit)
        service.start()
        while not stopped.wait(.2):
            if service.thread is None or not service.thread.is_alive():
                raise RuntimeError('Relay supervisor unavailable')
        return 0
    except Exception:
        emit({'event': 'imu_relay_unavailable', 'read_only': True})
        return 1
    finally:
        if service is not None:
            service.close()
        for number, handler in old_handlers.items():
            signal.signal(number, handler)


if __name__ == '__main__':
    raise SystemExit(main())
