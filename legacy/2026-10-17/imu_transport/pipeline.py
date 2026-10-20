"""Bounded 100 Hz read-request pipeline; Host wire/authentication contract unchanged.

Every observation is real and authenticated. There is no pose interpolation,
history playback, command route, device discovery or credential-file access.
"""
from copy import deepcopy
import secrets
import socket
import threading
import time

if __package__:
    from .protocol import (PCProtocol,ProtocolError,MAX_RTT_MS,MAX_AGE_MS,MAX_DATAGRAM,
                           VERSION,SAMPLE_FIELDS,compact_status,decode,encode,message,identity,finite)
    from .host import private_literal
else:
    from protocol import (PCProtocol,ProtocolError,MAX_RTT_MS,MAX_AGE_MS,MAX_DATAGRAM,
                          VERSION,SAMPLE_FIELDS,compact_status,decode,encode,message,identity,finite)
    from host import private_literal

WINDOW = 8
SEND_INTERVAL = .010


class PipelinePCProtocol(PCProtocol):
    """Single-owner pure state machine; one optional HELLO plus <=8 total requests."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.reads={}
        self.next_send=self.last_clock
        self.max_pending_seen=0
        # Background HELLO uses a second stable client identity. The Host
        # replaces sessions owned by that client on HELLO, so probing with the
        # active identity would silently destroy usable reads if the ACK is lost.
        # Alternate these two identities; never accumulate unbounded sessions.
        self._spare_client=secrets.token_hex(16)

    @property
    def outstanding(self):
        return len(self.reads)+(self.pending is not None)

    def expire(self):
        super().expire()
        now=self._time()
        expired=[rid for rid,record in self.reads.items() if (now-record['sent'])*1000>MAX_RTT_MS]
        for rid in expired:del self.reads[rid]
        self.stats['expired_reads']+=len(expired)

    def hello(self):
        self.expire()
        if self.outstanding>=WINDOW:
            raise ProtocolError('Pipeline window full')
        packet=super().hello()
        if self.session is not None:
            self.pending['payload']['client']=self._spare_client
            packet=encode(self.pending['payload'],self.key)
        self.max_pending_seen=max(self.max_pending_seen,self.outstanding)
        return packet

    def request_due(self):
        self.expire()
        now=self._time()
        if (not self.session or now>=self.session['expires']
                or now+1e-12<self.next_send or self.outstanding>=WINDOW):
            return None
        self.rid+=1
        if self.rid>=2**64:raise ProtocolError('Request counter exhausted')
        # The base class uses the same CSPRNG and exact request fields.
        import secrets
        payload=dict(v=VERSION,kind='read',client=self.client,rid=self.rid,nonce=secrets.token_hex(16),
                     host=self.session['host'],sid=self.session['sid'])
        self.reads[self.rid]=dict(payload=payload,sent=now,session=dict(self.session))
        self.next_send=now+SEND_INTERVAL  # No catch-up burst after a scheduler pause.
        self.max_pending_seen=max(self.max_pending_seen,self.outstanding)
        self.stats['sent_reads']+=1
        return encode(payload,self.key)

    def discard_read(self,rid):
        if self.reads.pop(rid,None) is not None:self.stats['unsent_reads']+=1

    def rebind_readonly_source(self):
        self.stats['abandoned_reads']+=len(self.reads)
        self.reads.clear()
        super().rebind_readonly_source()
        self.next_send=self._time()

    def accept(self,packet,peer):
        now=self._time()
        if peer!=self.peer:raise ProtocolError('Foreign datagram')
        payload=decode(packet,self.key)
        if payload.get('kind')=='hello_ack':
            request_client=self.pending['payload']['client'] if self.pending else None
            previous_client=self.client
            result=super().accept(packet,peer)
            if request_client!=previous_client:
                self.client=request_client
                self._spare_client=previous_client
            return result
        message(payload,SAMPLE_FIELDS,'sample')
        record=self.reads.get(payload['rid'])
        if record is None:raise ProtocolError('Unsolicited/expired read response')
        request=record['payload'];session=record['session']
        if any(payload[name]!=request[name] for name in ('client','rid','nonce','host','sid')):
            raise ProtocolError('Wrong pipelined request nonce/session')
        rtt_ms=(now-record['sent'])*1000
        if rtt_ms>MAX_RTT_MS or now>=session['expires']:
            del self.reads[payload['rid']]
            raise ProtocolError('Late pipelined datagram')
        if not self.session or any(payload[name]!=self.session[name] for name in ('host','sid')):
            del self.reads[payload['rid']]
            raise ProtocolError('Old Host/session epoch')
        status=compact_status(payload['status'])
        if identity(status)!=self.source_identity:
            del self.reads[payload['rid']]
            raise ProtocolError('Source boot/instance changed')
        status['feedback_age_ms']+=status['imu_snapshot_age_ms']+rtt_ms
        if status['feedback_age_ms']>=MAX_AGE_MS:
            del self.reads[payload['rid']]
            raise ProtocolError('Known observation age is stale')
        source=tuple(status[name] for name in ('mcu_seq','mcu_sample_us','mcu_imu_us'))
        if self.last_source is not None and any(not 0<((current-previous)&0xffffffff)<0x80000000
                                                for current,previous in zip(source,self.last_source)):
            del self.reads[payload['rid']]
            raise ProtocolError('Duplicate/regressed source; no backlog replay')
        checked_at=finite(self.wall_clock(),'PC wall clock')
        if checked_at<=0:raise ProtocolError('Invalid local assessment timestamp')
        del self.reads[payload['rid']]
        self.last_source=source
        self.stats['accepted_samples']+=1
        return {'status':status,'read_only':True,'checked_at':checked_at}


class PipelineReader:
    """One background UDP I/O thread; poll is nonblocking and returns latest once.

    Construction is inert. Calling poll/start explicitly starts an enabled reader.
    Queuing time between acceptance and poll is added to known age exactly once.
    """
    def __init__(self,private_key,peer,*,enabled=False,clock=time.monotonic,wall_clock=time.time):
        if (not isinstance(peer,tuple) or len(peer)!=2 or type(peer[1]) is not int or not 1<=peer[1]<=65535):
            raise ProtocolError('Explicit private peer required')
        private_literal(peer[0])
        self.enabled=enabled is True
        self.clock,self.wall_clock=clock,wall_clock
        self.core=PipelinePCProtocol(private_key,peer,clock=clock,wall_clock=wall_clock)
        self.lock=threading.RLock()
        self.stop_event=threading.Event()
        self.thread,self.socket=None,None
        self.last_error=''
        self._latest=None;self._revision=0;self._delivered=0
        self._retry_at=0.;self._next_hello_at=0.;self._renewing=False
        self._session_at=None;self._last_accept=None
        self.progress_identity=None;self.progress=None;self.retired_identities=set()
        self.closed=False

    @property
    def last_packet(self):
        with self.lock:
            return None if self._latest is None else deepcopy(self._latest['packet'])

    def start(self):
        with self.lock:
            if not self.enabled:raise ProtocolError('Pipeline reader disabled')
            if self.closed:raise ProtocolError('Pipeline reader closed')
            if self.thread is None:
                self.thread=threading.Thread(target=self._run,name='xgo-imu-udp-pipeline',daemon=True)
                self.thread.start()
        return self

    def _clear_session(self,reason,now):
        self._latest=None
        self.last_error=str(reason)
        self.core.rebind_readonly_source()
        self._retry_at=now+.5;self._renewing=False;self._session_at=None;self._last_accept=None

    def _on_hello(self,now):
        source=self.core.source_identity
        if source in self.retired_identities:
            raise ProtocolError('Retired source identity refused')
        if source==self.progress_identity:
            self.core.last_source=self.progress
        else:
            if self.progress_identity is not None:
                if len(self.retired_identities)>=64:raise ProtocolError('Source identity history exhausted')
                self.retired_identities.add(self.progress_identity)
            self.progress_identity=source;self.progress=None
        # A validated new ACK is the atomic switch point. Old-session replies
        # can no longer publish after it, even if their nonce and RTT are valid.
        current=self.core.session
        old=[rid for rid,record in self.core.reads.items()
             if any(record['session'][name]!=current[name] for name in ('host','sid'))]
        for rid in old:del self.core.reads[rid]
        self.core.stats['retired_session_reads']+=len(old)
        self._renewing=False;self._retry_at=0.;self._session_at=now

    def _issue_due(self,now):
        """Called by the sole I/O owner while holding lock; never waits for ACK."""
        if now<self._retry_at:return
        session=self.core.session
        valid=bool(session and now<session['expires'])
        progress_at=self._last_accept if self._last_accept is not None else self._session_at
        probe=bool(valid and progress_at is not None and now-progress_at>.30)
        hello_due=not valid or now+.5>=session['expires'] or probe
        if (hello_due and self.core.pending is None and now>=self._next_hello_at
                and self.core.outstanding<WINDOW):
            self._renewing=True
            self._send(self.core.hello());self._next_hello_at=now+.5
        if valid:
            self._send(self.core.request_due())

    def _send(self,packet):
        if packet is None:return
        try:self.socket.sendto(packet,self.core.peer)
        except (BlockingIOError,InterruptedError):
            if self.core.pending and self.core.pending['payload']['rid']==self.core.rid:
                self.core.discard_pending('socket_send_error')
            else:self.core.discard_read(self.core.rid)
            self.core.stats['socket_send_error']+=1
        except OSError:
            self.core.stats['socket_send_error']+=1
            raise  # The relay owns the bounded retry/replacement policy.

    def _receive(self,packet,peer):
        """Single-owner receive step, shared with deterministic network QA."""
        try:
            result=self.core.accept(packet,peer);now=finite(self.clock(),'pipeline clock')
            if result.get('handshake'):
                self._on_hello(now)
            else:
                self.progress_identity=self.core.source_identity;self.progress=self.core.last_source
                self._latest=dict(packet=result,at=now);self._revision+=1;self._last_accept=now
                self.last_error=''
            return result
        except ProtocolError as error:
            self.core.stats[str(error)]+=1
            if str(error) in ('Source identity changed; new explicit client required','Retired source identity refused'):
                self._clear_session(error,finite(self.clock(),'pipeline clock'))
            return None

    def _run(self):
        try:
            self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.socket.bind(('127.0.0.1' if self.core.peer[0].startswith('127.') else '0.0.0.0',0))
            self.socket.setblocking(False)
            while not self.stop_event.is_set():
                with self.lock:
                    now=finite(self.clock(),'pipeline clock')
                    self.core.expire()
                    if self._latest and self._latest['packet']['status']['feedback_age_ms']+(now-self._latest['at'])*1000>=MAX_AGE_MS:
                        self._latest=None
                    self._issue_due(now)
                # Nonblocking receive plus Python's high-resolution sleep avoids
                # Windows select() rounding a1ms timeout to ~15.6ms (~64Hz).
                # A bounded drain cannot starve the next send deadline.
                for _ in range(32):
                    if self.stop_event.is_set():break
                    try:packet,peer=self.socket.recvfrom(MAX_DATAGRAM+1)
                    except (BlockingIOError,InterruptedError):break
                    except OSError:
                        with self.lock:self.core.stats['socket_receive_error']+=1
                        raise
                    with self.lock:
                        self._receive(packet,peer)
                time.sleep(.001)
        except (OSError,ProtocolError) as error:
            with self.lock:
                self.last_error=str(error);self._latest=None
                self.core.rebind_readonly_source()
        finally:
            if self.socket is not None:self.socket.close()
            self.socket=None

    def poll(self):
        if self.closed:return None
        self.start()
        with self.lock:
            if self.thread is not None and not self.thread.is_alive():
                raise OSError('IMU UDP worker stopped; recreate the read-only connection')
            if self._latest is None or self._revision==self._delivered:return None
            now=finite(self.clock(),'pipeline poll clock')
            elapsed=(now-self._latest['at'])*1000
            if elapsed<0:
                self._clear_session('Clock regressed',now);return None
            packet=deepcopy(self._latest['packet'])
            packet['status']['feedback_age_ms']+=elapsed
            if packet['status']['feedback_age_ms']>=MAX_AGE_MS:
                self._latest=None;return None
            packet['checked_at']=finite(self.wall_clock(),'pipeline poll wall clock')
            if packet['checked_at']<=0:raise ProtocolError('Invalid local assessment timestamp')
            self._delivered=self._revision
            return packet

    def close(self):
        self.closed=True;self.stop_event.set()
        if self.thread is not None:self.thread.join(timeout=1)
        with self.lock:
            self._latest=None
            self.core.rebind_readonly_source()


def pipeline_reader_if_enabled(config=None,private_key=None,**kwargs):
    if __package__:
        from .integration import _config
    else:
        from integration import _config
    peer=_config(config,'peer_ip')
    return None if peer is None else PipelineReader(private_key,peer,enabled=True,**kwargs)
