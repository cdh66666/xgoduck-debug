"""Shared real-state relay: board SSE, or persistent HTTP on old probe1.

All network requests are GET against already cached private board addresses.
It never renews a calibration lease or issues a Bridge/servo command.
"""
import http.client
from collections import deque
import json
import math
import socket
import threading
import time
from urllib.parse import urlsplit
from commissioning_support import sanitize_status

MAX_BYTES=131072


def configured_target(root):
    """Address selection only; cached identity is never promoted to live proof."""
    import sys
    from pathlib import Path
    path=str(Path(root)/'tools')
    if path not in sys.path:sys.path.insert(0,path)
    import discover_unoq
    choices=discover_unoq.cached_candidates(root)
    return choices[0] if choices else None


def trusted_target(root, value):
    import sys
    from pathlib import Path
    path=str(Path(root)/'tools')
    if path not in sys.path:sys.path.insert(0,path)
    import discover_unoq
    target=discover_unoq.literal_url(value)
    if not target or target not in discover_unoq.cached_candidates(root):
        raise ValueError('无线地址尚未通过发现确认')
    return target


class Source:
    def __init__(self,url,endpoint=None):
        self.url=url;self.condition=threading.Condition();self.clients=0;self.counter=0
        self.packet=None;self.key=None;self.last_client=time.monotonic();self.published_at=0;self.thread=None
        self.pending=deque(maxlen=32)
        self.endpoint=endpoint

    def acquire(self):
        with self.condition:
            self.clients+=1;self.last_client=time.monotonic()
            if not self.thread or not self.thread.is_alive():
                self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def release(self):
        with self.condition:self.clients=max(0,self.clients-1);self.last_client=time.monotonic()

    def publish(self, raw):
        if not isinstance(raw,dict):return
        s=sanitize_status(raw)
        seq=s.get('mcu_seq');age=s.get('feedback_age_ms')
        if type(seq) is not int or not 0<=seq<=0xffffffff or type(age) not in (int,float) or not 0<=age<150:return
        # Display scheduling only: the MCU oscillator is not the Linux clock.
        # Preserve the board observation time before adding PC queue age. This
        # metadata is not an end-to-end freshness proof or control permission.
        source_clock=raw.get('source_monotonic')
        if type(source_clock) in (int,float) and math.isfinite(source_clock) and source_clock>0:
            observed_ms=source_clock*1000-age
            if math.isfinite(observed_ms) and observed_ms>0:
                s['display_observed_monotonic_ms']=observed_ms
        key=(s.get('app_instance_id'),s.get('mcu_restarts'),seq,s.get('mcu_sample_us'))
        with self.condition:
            if key==self.key:return
            self.key=key;self.counter+=1;self.published_at=time.monotonic();self.packet={'status':s,'read_only':True,'checked_at':time.time()};self.condition.notify_all()
            self.pending.append((self.counter,self.packet,self.published_at))

    def offline(self):
        with self.condition:
            if self.packet and self.packet.get('error'):return
            self.packet={'error':True};self.counter+=1;self.pending.append((self.counter,self.packet,time.monotonic()));self.condition.notify_all()

    def wait(self,cursor,timeout=1):
        with self.condition:
            self.condition.wait_for(lambda:self.counter!=cursor,timeout)
            packet=self.packet
            if packet and isinstance(packet.get('status'),dict):
                s=dict(packet['status']);s['feedback_age_ms']+=max(0,(time.monotonic()-self.published_at)*1000)
                packet={**packet,'status':s,'checked_at':time.time()} if s['feedback_age_ms']<150 else {'error':True}
            return self.counter,packet

    def wait_batch(self,cursor,timeout=1):
        """Drain only actual arrivals, bounded to 32 and still source-fresh.

        A socket read can contain several real frames. One latest slot discarded
        them before the HTTP thread could acquire the GIL. Never replay stale
        backlog or create a frame merely because the consumer timer fired.
        """
        with self.condition:
            self.condition.wait_for(lambda:self.counter!=cursor,timeout)
            now=time.monotonic();checked_at=time.time();batch=[]
            for counter,packet,received_at in self.pending:
                if counter<=cursor:continue
                if packet.get('error'):
                    batch=[(counter,packet)];continue
                status=dict(packet['status'])
                status['feedback_age_ms']+=max(0,(now-received_at)*1000)
                if status['feedback_age_ms']<150:batch.append((counter,{**packet,'status':status,'checked_at':checked_at}))
            if self.counter!=cursor and not batch:batch=[(self.counter,{'error':True})]
            return self.counter,batch

    def running(self):
        with self.condition:return self.clients>0 or time.monotonic()-self.last_client<3

    def run(self):
        target=urlsplit(self.url);connection=None;retiring=False
        try:
            while self.running():
                try:
                    endpoint=urlsplit(self.endpoint() if self.endpoint else self.url)
                    connection=http.client.HTTPConnection(endpoint.hostname,endpoint.port or 9527,timeout=2)
                    connection.connect();connection.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    connection.request('GET','/api/live',headers={'Accept':'text/event-stream'})
                    response=connection.getresponse()
                    if response.status==200 and response.getheader('Content-Type','').startswith('text/event-stream'):
                        data=[];size=0
                        while self.running():
                            line=response.fp.readline(MAX_BYTES+1)
                            if not line:raise OSError('stream ended')
                            if len(line)>MAX_BYTES:raise ValueError('oversize stream')
                            if line.startswith(b'data:'):
                                piece=line[5:].strip();size+=len(piece)
                                if size>MAX_BYTES:raise ValueError('oversize event')
                                data.append(piece)
                            if line.strip()==b'':
                                if data:
                                    value=json.loads(b'\n'.join(data))
                                    if not isinstance(value,dict):raise ValueError('invalid stream event')
                                    status=value.get('status',value)
                                    if not isinstance(status,dict):raise ValueError('invalid stream status')
                                    self.publish(status)
                                data=[];size=0
                    elif response.status==404:
                        response.read(MAX_BYTES+1)
                        # Old firmware: reuse a keep-alive connection, no 200ms tick skips.
                        while self.running():
                            began=time.monotonic();connection.request('GET','/api/status',headers={'Accept':'application/json'})
                            reply=connection.getresponse();raw=reply.read(MAX_BYTES+1)
                            if reply.status!=200 or len(raw)>MAX_BYTES:raise ValueError('invalid status')
                            self.publish(json.loads(raw));time.sleep(max(0,.04-(time.monotonic()-began)))
                    else:raise OSError('live unavailable')
                except (OSError,ValueError,http.client.HTTPException):
                    self.offline();time.sleep(.5)
                finally:
                    if connection:connection.close();connection=None
            retiring=True
        finally:
            if connection:
                try:connection.close()
                except OSError:pass
            with self.condition:
                self.thread=None
                if retiring and self.clients>0:
                    self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()


_sources={};_sources_lock=threading.Lock()
def source_for(root,value):
    target=trusted_target(root,value)
    with _sources_lock:
        if target not in _sources:
            from wireless_calibration_proxy import proxy
            _sources[target]=Source(target,endpoint=lambda:proxy.readonly_endpoint(target))
        return _sources[target]
