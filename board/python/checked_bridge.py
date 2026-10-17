"""Confirm command dispatch without confusing RPC receipt with servo completion.

Only explicit caller requests are sent, once. There is no reconnect retry or
motion replay. The existing DWR/DHS acknowledgements remain authoritative.
"""
import threading
import time


class DispatchError(RuntimeError):
    def __init__(self, code):
        self.code = code
        super().__init__(code)


def missing_route(error):
    """Match the pinned SDK's exact code2 wrapper, never a generic timeout."""
    direct = (getattr(error, 'code', None) == 2
              and getattr(error, 'message', None) == 'method duck_calibration not available')
    wrapped = (type(error) is ValueError and str(error) ==
               "Request 'duck_calibration' failed: method duck_calibration not available (2)")
    return direct or wrapped


class CheckedBridge:
    TIMEOUT_S = .2

    def __init__(self, bridge, clock=time.monotonic):
        if not callable(getattr(bridge, 'call', None)):
            raise RuntimeError('A bounded command RPC adapter is required')
        self.bridge, self.clock = bridge, clock
        self.lock = threading.Lock()
        self.last_state = 'unknown'
        self.last_at = None

    def notify(self, method, payload):
        # Compatibility boundary for the existing FIFO sender. Despite this
        # historical method name, dispatch now uses a bounded RPC request.
        if method != 'duck_calibration' or type(payload) is not bytes or len(payload) not in (32, 108):
            raise DispatchError('invalid_command_envelope')
        try:
            self.bridge.call(method, payload, timeout=self.TIMEOUT_S)
        except Exception as error:
            code = 'command_route_unavailable' if missing_route(error) else 'command_delivery_unknown'
            with self.lock:
                self.last_state, self.last_at = code, self.clock()
            raise DispatchError(code) from None
        with self.lock:
            self.last_state, self.last_at = 'handler_received', self.clock()

    def status(self):
        with self.lock:
            age = None if self.last_at is None else max(0, (self.clock()-self.last_at)*1000)
            return {'state': self.last_state, 'age_ms': age,
                    'scope': 'RPC dispatch only; DWR/DHS required for completion',
                    'automatic_retry': False, 'motion_resumed': False}
