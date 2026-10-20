"""Opt-in application hooks. Disabled config performs no key validation or I/O.

This module is a read-only transport lifecycle only. It cannot discover devices,
load credentials, replace calibration sessions, deploy code or restore motion.
"""
from copy import deepcopy
import threading
import time

if __package__:
    from .host import HostEndpoint,private_literal
    from .pc import PCClient
    from .protocol import HostProtocol,ProtocolError,MAX_RTT_MS,finite
else:
    from host import HostEndpoint,private_literal
    from pc import PCClient
    from protocol import HostProtocol,ProtocolError,MAX_RTT_MS,finite


def _config(config, address_field):
    if config is None:
        return None
    if not isinstance(config,dict) or type(config.get('enabled',False)) is not bool:
        raise ProtocolError('Explicit boolean enabled flag required')
    if not config.get('enabled',False):
        return None
    if set(config) != {'enabled',address_field,'port'}:
        raise ProtocolError('Only enabled, literal address and port are supported')
    address=private_literal(config[address_field]);port=config['port']
    minimum=0 if address_field=='bind_ip' else 1
    if type(port) is not int or not minimum<=port<=65535:
        raise ProtocolError('Invalid UDP port')
    return address,port


def start_host_if_enabled(config=None, private_key=None, live_status=None, *, clock=time.monotonic):
    target=_config(config,'bind_ip')
    if target is None:
        return None
    protocol=HostProtocol(private_key,live_status,clock=clock)
    return HostEndpoint(protocol,target[1],bind=target[0],enabled=True).start()


class ReadOnlyReader:
    """Poll latest only; automatically renew read authentication, never control state.

    None means no new usable observation. A relay must not replay last_packet or
    refresh its sample age on None. Its existing stale timer remains authoritative.
    """
    def __init__(self, private_key, peer, *, clock=time.monotonic, wall_clock=time.time, client_factory=PCClient):
        self.clock=clock
        self.client=client_factory(private_key,peer,enabled=True,clock=clock,wall_clock=wall_clock)
        self.lock=threading.Lock()
        self.last_packet=None
        self.last_error=''
        self.failures=0
        self.retry_at=0.
        self.next_read_at=0.
        self.progress_identity=None
        self.progress=None
        self.retired_identities=set()
        self.closed=False

    def _clear_and_rebind(self, reason, now):
        self.last_packet=None
        self.last_error=str(reason)
        self.client.core.rebind_readonly_source()
        self.retry_at=now+.5  # At most two handshake attempts/s; no data backlog.
        self.failures=0

    def _handshake(self, now):
        self.last_packet=None
        result=self.client.connect()
        if not result:
            self._clear_and_rebind('Read-only handshake unavailable',now)
            return False
        source=self.client.core.source_identity
        if source in self.retired_identities:
            self._clear_and_rebind('Retired source identity refused',now)
            return False
        if self.progress_identity==source:
            # A new socket/session may never revive the same old MCU sample.
            self.client.core.last_source=self.progress
        else:
            if self.progress_identity is not None:
                if len(self.retired_identities)>=64:
                    raise ProtocolError('Source identity history exhausted; explicit new reader required')
                self.retired_identities.add(self.progress_identity)
            self.progress_identity=source
            self.progress=None
        self.retry_at=0.
        self.failures=0
        return True

    def poll(self):
        with self.lock:
            if self.closed:
                return None
            now=finite(self.clock(),'reader monotonic time')
            if now<self.retry_at or now<self.next_read_at:
                return None
            try:
                if self.client.socket is None:
                    self.client.open()
                session=self.client.core.session
                if session is None or now+MAX_RTT_MS/1000>=session['expires']:
                    if not self._handshake(now):return None
                self.next_read_at=finite(self.clock(),'reader monotonic time')+.010
                result=self.client.poll()
                if result is None:
                    self.last_packet=None
                    self.last_error='No fresh authenticated observation'
                    self.failures+=1
                    if self.failures>=3:
                        self._clear_and_rebind(self.last_error,finite(self.clock(),'reader monotonic time'))
                    return None
                self.progress_identity=self.client.core.source_identity
                self.progress=self.client.core.last_source
                self.last_packet=deepcopy(result)
                self.last_error='';self.failures=0
                return result
            except (ProtocolError,OSError,ValueError,TypeError) as exc:
                self._clear_and_rebind(exc,finite(self.clock(),'reader monotonic time'))
                if isinstance(exc,OSError):self.client.close()
                return None

    def close(self):
        with self.lock:
            self.closed=True
            self.last_packet=None
            self.client.core.discard_pending('reader_closed')
            self.client.close()


def reader_if_enabled(config=None, private_key=None, **kwargs):
    target=_config(config,'peer_ip')
    return None if target is None else ReadOnlyReader(private_key,target,**kwargs)
