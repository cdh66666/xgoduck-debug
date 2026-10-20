"""Read-only latest-status stream from the currently selected SSH tunnel."""
from __future__ import annotations

import copy
from collections import deque
import http.client
import json
import socket
import threading
import time


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
        self._disconnect_locked()
        self._failure = ConnectionError('主控无线端口已变化，等待新状态')
        self._invalidated_generation = self._generation
        self.tunnel.error = 'official_status_stream_unavailable'
        self.tunnel.official = False
        self._condition.notify_all()
        return True

    def get(self):
        deadline = time.monotonic()+self.freshness
        with self._condition:
            while True:
                if self._stop.is_set():
                    raise ConnectionError('状态读取已关闭')
                self._port_changed_locked()
                if self._failure is not None:
                    raise copy.copy(self._failure)
                if self._value is not None and self._generation > self._invalidated_generation:
                    if time.monotonic()-self.read_at <= self.freshness:
                        return copy.deepcopy(self._value)
                    # Wait within this request's existing deadline for a new
                    # received frame. Never return the stale cached snapshot.
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    self.tunnel.error = 'official_status_stream_stale'
                    self.tunnel.official = False
                    raise TimeoutError('等待主控下一帧状态超时')
                self._condition.wait(remaining)

    def frames(self):
        """Actual received endpoints for display; this never asserts freshness."""
        with self._condition:
            self._trim_frames(time.monotonic())
            return copy.deepcopy(list(self._frames))

    def _trim_frames(self, now):
        while self._frames and now-self._frames[0]['pc_observed_monotonic_s'] > 1:
            self._frames.popleft()

    def _accept(self, state, port, received_at, received_unix):
        if (not isinstance(state, dict) or not isinstance(state.get('deployment'), dict) or
                state['deployment'].get('reference') != 'rl/raspi_deploy'):
            raise ValueError('不是当前官方运行链')
        with self._condition:
            if self._stop.is_set() or port != self.tunnel.port:
                return False
            uptime = state.get('uptime_s')
            restarted = (isinstance(uptime, (int, float)) and
                         isinstance(self.last_uptime, (int, float)) and uptime < self.last_uptime)
            if restarted:
                self.epoch += 1
                self._sequence = None
                self._frames.clear()
            self.last_uptime = uptime
            sequence = ('mcu_seq', state['mcu_seq']) if 'mcu_seq' in state else (
                ('received', state['received']) if 'received' in state else None)
            if sequence is not None and sequence == self._sequence:
                return True
            self._sequence = sequence
            state['pc_host_epoch'] = self.epoch
            state['pc_observed_monotonic_s'] = received_at
            state['pc_observed_unix_s'] = received_unix
            state['pc_transport'] = 'official-status-stream'
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
        while not self._stop.is_set():
            response = None
            connection = None
            port = self.tunnel.port
            try:
                with self._condition:
                    self._port_changed_locked()
                    if port is None:
                        raise ConnectionError('主控无线未连接')
                    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=self.timeout)
                    self._connection = connection
                connection.request('GET', '/api/status-stream')
                with self._condition:
                    if self._stop.is_set():
                        break
                    self._socket = connection.sock
                response = connection.getresponse()
                if response.status != 200:
                    raise ConnectionError('官方状态流暂不可用')
                kind = response.getheader('Content-Type', '').split(';', 1)[0].strip().lower()
                if kind != 'application/x-ndjson':
                    raise ValueError('主控未返回NDJSON状态流')
                if self._socket is not None:
                    # Display freshness expires before the transport itself.
                    # A short delivery gap must not tear down a healthy stream.
                    self._socket.settimeout(self.timeout)
                while not self._stop.is_set():
                    raw = response.readline(256*1024+1)
                    received_at, received_unix = time.monotonic(), time.time()
                    if not raw:
                        raise ConnectionError('主控状态流已断开')
                    if len(raw) > 256*1024 or not raw.endswith(b'\n'):
                        raise ValueError('主控状态流帧过大或不完整')
                    if not self._accept(json.loads(raw), port, received_at, received_unix):
                        break
            except (OSError, ValueError, http.client.HTTPException) as error:
                with self._condition:
                    if not self._stop.is_set():
                        self._failure = copy.copy(error)
                        self.tunnel.error = 'official_status_stream_unavailable'
                        self.tunnel.official = False
                        self._condition.notify_all()
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
