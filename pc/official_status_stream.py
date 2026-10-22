"""Read-only latest-status stream from the currently selected SSH tunnel."""
from __future__ import annotations

import copy
from collections import deque
import http.client
import json
import math
import socket
import threading
import time


class _SourceBacklogError(ConnectionError):
    """Received lines lag behind this process's recently observed source clock."""


class OfficialStatusStream:
    def __init__(self, tunnel, *, freshness=.35, reconnect_delay=.1, timeout=2):
        self.tunnel = tunnel
        self.freshness = freshness
        self.reconnect_delay = reconnect_delay
        self.timeout = timeout
        self.read_at = 0
        self.epoch = 1
        self.last_uptime = None
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._value = None
        self._frames = deque(maxlen=64)
        self._generation = 0
        self._invalidated_generation = -1
        self._sequence = None
        self._failure = None
        self._connection = None
        self._socket = None
        self._port = tunnel.port
        self._source_offsets = deque()
        self._source_window_s = 10.
        self._started_at = time.perf_counter()
        self._connects = self._backlog_drops = self._relay_recoveries = 0
        self._worker = threading.Thread(target=self._run, name='official-status-stream', daemon=True)
        self._worker.start()

    @property
    def value(self):
        with self._condition:
            return copy.deepcopy(self._value)

    @value.setter
    def value(self, value):
        if value is not None:
            raise ValueError('Only cache invalidation with value=None is supported')
        with self._condition:
            self._invalidated_generation = self._generation
            self._condition.notify_all()

    def _disconnect_locked(self):
        connection, self._connection = self._connection, None
        stream_socket, self._socket = self._socket, None
        if stream_socket is not None:
            try:
                stream_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if connection is not None:
            connection.close()

    def _port_changed_locked(self):
        if self._port == self.tunnel.port:
            return False
        self._port = self.tunnel.port
        self._source_offsets.clear()
        self._disconnect_locked()
        self._failure = ConnectionError('主控无线端口已变化，等待新状态')
        self._invalidated_generation = self._generation
        self.tunnel.error = 'official_status_stream_unavailable'
        self.tunnel.official = False
        self._condition.notify_all()
        return True

    def get(self):
        deadline = time.perf_counter()+self.freshness
        with self._condition:
            while True:
                if self._stop.is_set():
                    raise ConnectionError('状态读取已关闭')
                self._port_changed_locked()
                if self._failure is not None:
                    raise copy.copy(self._failure)
                if self._value is not None and self._generation > self._invalidated_generation:
                    if time.perf_counter()-self.read_at <= self.freshness:
                        return copy.deepcopy(self._value)
                    # Wait within this request's existing deadline for a new
                    # received frame. Never return the stale cached snapshot.
                remaining = deadline-time.perf_counter()
                if remaining <= 0:
                    self.tunnel.error = 'official_status_stream_stale'
                    self.tunnel.official = False
                    raise TimeoutError('等待主控下一帧状态超时')
                self._condition.wait(remaining)

    def frames(self, *, after=None, after_observation=None, through_observation=None):
        """Actual received endpoints for display; this never asserts freshness."""
        with self._condition:
            self._trim_frames(time.perf_counter())
            if after_observation is not None:
                rows = (frame for frame in self._frames if frame['pc_observation_id'] > after_observation)
            else:
                rows = self._frames if after is None else (
                    frame for frame in self._frames if frame['pc_observed_monotonic_s'] > after)
            if through_observation is not None:
                rows = [frame for frame in rows if frame['pc_observation_id'] <= through_observation]
            return copy.deepcopy(list(rows))

    def _trim_frames(self, now):
        while self._frames and now-self._frames[0]['pc_observed_monotonic_s'] > 1:
            self._frames.popleft()

    def diagnostics(self):
        with self._condition:
            return dict(worker_alive=self._worker.is_alive(),http_connects=self._connects,
                        backlog_frames_skipped=self._backlog_drops,relay_recoveries=self._relay_recoveries,
                        last_received_age_ms=(time.perf_counter()-self.read_at)*1000 if self.read_at else None,
                        failure_type=type(self._failure).__name__ if self._failure else None,
                        receive_clock=time.get_clock_info('perf_counter').implementation,
                        receive_clock_resolution_s=time.get_clock_info('perf_counter').resolution)

    def _accept(self, state, port, received_at, received_unix):
        if (not isinstance(state, dict) or not isinstance(state.get('deployment'), dict) or
                state['deployment'].get('reference') != 'rl/raspi_deploy'):
            raise ValueError('不是当前官方运行链')
        with self._condition:
            if self._stop.is_set() or port != self.tunnel.port:
                return False
            uptime = state.get('uptime_s')
            uptime = (float(uptime) if isinstance(uptime, (int, float)) and
                      not isinstance(uptime, bool) and math.isfinite(uptime) and uptime >= 0 else None)
            restarted = (isinstance(uptime, (int, float)) and
                         isinstance(self.last_uptime, (int, float)) and uptime < self.last_uptime)
            if restarted:
                self.epoch += 1
                self._sequence = None
                self._frames.clear()
                self._source_offsets.clear()
            self.last_uptime = uptime
            sequence = ('mcu_seq', state['mcu_seq']) if 'mcu_seq' in state else (
                ('received', state['received']) if 'received' in state else None)
            if sequence is not None and sequence == self._sequence:
                return True
            if uptime is None:
                self._source_offsets.clear()
                progress = dict(basis='uptime_s unavailable', relative_backlog_lower_bound_s=None,
                                reference_window_s=self._source_window_s,
                                absolute_network_latency_measured=False)
            else:
                # Two monotonic clocks may have unrelated origins. Their offset
                # minimum only measures added lag relative to recent observations;
                # it cannot establish absolute network delay on the first frame.
                while self._source_offsets and received_at-self._source_offsets[0][0] > self._source_window_s:
                    self._source_offsets.popleft()
                offset = received_at-uptime
                reference = min([offset] + [item[1] for item in self._source_offsets])
                backlog = max(0., offset-reference)
                progress = dict(basis='uptime_s relative progress', relative_backlog_lower_bound_s=backlog,
                                reference_window_s=self._source_window_s,
                                absolute_network_latency_measured=False)
                if backlog > self.freshness:
                    error = _SourceBacklogError(f'主控状态来源进展滞后至少{backlog:.3f}秒，等待新状态流')
                    self._failure = error
                    self._invalidated_generation = self._generation
                    self.tunnel.error = 'official_status_stream_backlog'
                    self.tunnel.official = False
                    self._condition.notify_all()
                    # The reader skips this old endpoint and drains the same
                    # healthy connection. Reconnecting here adds an SSH/HTTP
                    # handshake delay to a temporary delivery burst.
                    raise error
                self._source_offsets.append((received_at, offset))
            self._sequence = sequence
            state['pc_host_epoch'] = self.epoch
            state['pc_observation_id'] = self._generation+1
            state['pc_observed_monotonic_s'] = received_at
            state['pc_observed_unix_s'] = received_unix
            state['pc_transport'] = getattr(self, 'transport_name', 'official-status-stream')
            state['pc_source_progress'] = progress
            self._value = state
            self._frames.append(state)
            self._trim_frames(received_at)
            self.read_at = received_at
            self._generation += 1
            self._failure = None
            self.tunnel.error = None
            self.tunnel.official = True
            self._condition.notify_all()
            return True

    def _run(self):
        failures=0;previous_owner=None
        while not self._stop.is_set():
            response = None
            connection = None
            port = self.tunnel.port
            process = getattr(self.tunnel, 'process', None)
            owner=(port,id(process))
            if owner != previous_owner:failures=0;previous_owner=owner
            try:
                with self._condition:
                    self._port_changed_locked()
                    if port is None:
                        raise ConnectionError('主控无线未连接')
                    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=self.timeout)
                    self._connects += 1
                    self._connection = connection
                connection.request('GET', '/api/status-stream')
                with self._condition:
                    if self._stop.is_set():
                        break
                    self._socket = connection.sock
                response = connection.getresponse()
                if response.status != 200:
                    raise http.client.HTTPException('官方状态流暂不可用')
                kind = response.getheader('Content-Type', '').split(';', 1)[0].strip().lower()
                if kind != 'application/x-ndjson':
                    raise ValueError('主控未返回NDJSON状态流')
                if self._socket is not None:
                    # Display freshness expires before the transport itself.
                    # A short delivery gap must not tear down a healthy stream.
                    self._socket.settimeout(self.timeout)
                while not self._stop.is_set():
                    raw = response.readline(256*1024+1)
                    # Python 3.12 on Windows uses a coarse 15.6ms clock for
                    # monotonic(). QPC preserves individual burst endpoints.
                    received_at, received_unix = time.perf_counter(), time.time()
                    if not raw:
                        raise ConnectionError('主控状态流已断开')
                    if len(raw) > 256*1024 or not raw.endswith(b'\n'):
                        raise ValueError('主控状态流帧过大或不完整')
                    try:
                        if not self._accept(json.loads(raw), port, received_at, received_unix):
                            break
                        failures=0
                    except _SourceBacklogError:
                        self._backlog_drops += 1
                        # Keep draining this healthy channel. Radio delivery
                        # can stall for longer than 150ms; reconnecting while
                        # bytes still arrive compounds the delay. Transport
                        # timeouts below handle a genuinely stuck relay.
                        # No cache timestamp, history or freshness is advanced
                        # by rejected frames. Only a genuinely caught-up frame
                        # can make get() succeed again.
                        continue
            except (OSError, ValueError, http.client.HTTPException) as error:
                with self._condition:
                    if not self._stop.is_set():
                        self._failure = copy.copy(error)
                        self.tunnel.error = ('official_status_stream_backlog' if isinstance(error, _SourceBacklogError)
                                             else 'official_status_stream_unavailable')
                        self.tunnel.official = False
                        self._condition.notify_all()
                if isinstance(error,OSError) and not isinstance(error,_SourceBacklogError):
                    failures += 1
                else:failures=0
                recover=getattr(self.tunnel,'invalidate_stalled',None)
                if (not self._stop.is_set() and failures>=3 and callable(recover)
                        and time.perf_counter()-(self.read_at or self._started_at)>2*self.timeout):
                    if recover(port,process):self._relay_recoveries += 1
                    failures=0
            finally:
                with self._condition:
                    self._disconnect_locked()
                if connection is not None:
                    connection.close()
                if response is not None:
                    response.close()
            self._stop.wait(self.reconnect_delay)

    def close(self):
        with self._condition:
            self._stop.set()
            self._disconnect_locked()
            self._condition.notify_all()
        if threading.current_thread() is not self._worker:
            self._worker.join(self.timeout+self.freshness+1)
            if self._worker.is_alive():
                raise RuntimeError('状态流读取线程未能停止')


# The workbench can use the same constructor name as the polling reader.
OfficialReader = OfficialStatusStream
