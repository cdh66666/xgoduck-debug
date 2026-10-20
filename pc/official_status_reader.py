"""Coalesced, read-only official status requests over the current SSH tunnel."""
from __future__ import annotations

import copy
import http.client
import json
import threading
import time


class OfficialReader:
    """Reuse one GET connection; share short-lived failures without stale success."""

    def __init__(self, tunnel, *, timeout=2, cache_ttl=.06, failure_ttl=.1,
                 connection_factory=None, monotonic=None, wall_clock=None):
        self.tunnel = tunnel
        self.lock = threading.Lock()
        self.value = None
        self.read_at = 0
        self.epoch = 1
        self.last_uptime = None
        self.timeout = timeout
        self.cache_ttl = cache_ttl
        self.failure_ttl = failure_ttl
        self._connection_factory = connection_factory or http.client.HTTPConnection
        self._monotonic = monotonic or time.monotonic
        self._wall_clock = wall_clock or time.time
        self._connection = None
        self._port = None
        self._failure = None
        self._failed_at = 0
        self._closed = False

    def _disconnect(self):
        connection, self._connection = self._connection, None
        if connection is not None:
            connection.close()

    def get(self):
        with self.lock:
            if self._closed:
                raise ConnectionError('状态读取已关闭')
            port = self.tunnel.port
            if port != self._port:
                self._disconnect()
                self._port = port
                self.value = None
                self._failure = None
            now = self._monotonic()
            if self.value is not None and now-self.read_at < self.cache_ttl:
                return copy.deepcopy(self.value)
            if self._failure is not None and now-self._failed_at < self.failure_ttl:
                raise copy.copy(self._failure)
            try:
                if port is None:
                    raise ConnectionError('主控无线未连接')
                if self._connection is None:
                    self._connection = self._connection_factory('127.0.0.1', port, timeout=self.timeout)
                connection = self._connection
                connection.request('GET', '/api/status')
                response = connection.getresponse()
                if response.status != 200:
                    raise ConnectionError('官方状态暂不可用')
                raw = response.read(256*1024+1)
                if len(raw) > 256*1024:
                    raise ValueError('状态回复过大')
                state = json.loads(raw)
                if not isinstance(state,dict):
                    raise ValueError('官方状态格式不正确')
                if state.get('deployment', {}).get('reference') != 'rl/raspi_deploy':
                    raise ValueError('不是当前官方运行链')
                uptime = state.get('uptime_s')
                if self.last_uptime is not None and isinstance(uptime, (int, float)) and uptime < self.last_uptime:
                    self.epoch += 1
                self.last_uptime = uptime
                observed = self._monotonic()
                state['pc_host_epoch'] = self.epoch
                state['pc_observed_monotonic_s'] = observed
                state['pc_observed_unix_s'] = self._wall_clock()
                self.value = state
                self.read_at = observed
                self._failure = None
                self.tunnel.error=None
                self.tunnel.official=True
                if response.will_close:
                    self._disconnect()
                return copy.deepcopy(state)
            except (OSError, ValueError, http.client.HTTPException) as error:
                self._disconnect()
                self.value = None
                self._failure = copy.copy(error)
                self._failed_at = self._monotonic()
                self.tunnel.error='wireless_status_unconfirmed'
                self.tunnel.official=False
                raise

    def close(self):
        with self.lock:
            self._closed = True
            self._disconnect()
            self.value = None
            self._failure = None
