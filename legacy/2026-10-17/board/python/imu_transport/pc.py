"""Explicit loopback-only client; no device address or private-key loader exists."""
import argparse
import json
import socket
import time

if __package__:
    from .protocol import PCProtocol, ProtocolError, MAX_DATAGRAM, MAX_RTT_MS
    from .host import SYNTHETIC_KEY,private_literal
else:
    from protocol import PCProtocol, ProtocolError, MAX_DATAGRAM, MAX_RTT_MS
    from host import SYNTHETIC_KEY,private_literal


class PCClient:
    def __init__(self, private_key, peer, *, enabled=False, clock=time.monotonic, wall_clock=time.time):
        if not isinstance(peer,tuple) or len(peer)!=2 or type(peer[1]) is not int or not 1<=peer[1]<=65535:
            raise ProtocolError('Explicit private IPv4:port required')
        private_literal(peer[0])
        self.enabled=enabled is True
        self.core=PCProtocol(private_key,peer,clock=clock,wall_clock=wall_clock)
        self.socket=None

    def open(self):
        if not self.enabled:
            raise ProtocolError('UDP endpoint is disabled')
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        # An ephemeral local IPv4 source port. No listener/control route exists.
        self.socket.bind(('127.0.0.1' if self.core.peer[0].startswith('127.') else '0.0.0.0',0))
        return self

    def _exchange(self, packet):
        self.socket.sendto(packet,self.core.peer)
        while self.core.pending:
            remaining=MAX_RTT_MS/1000-(self.core.clock()-self.core.pending['sent'])
            if remaining<=0:
                self.core.discard_pending()
                return None
            self.socket.settimeout(remaining)
            try:
                response,peer=self.socket.recvfrom(MAX_DATAGRAM+1)
                return self.core.accept(response,peer)
            except socket.timeout:
                self.core.discard_pending()
                return None
            except ProtocolError as exc:
                self.core.stats[str(exc)]+=1
        return None

    def connect(self):
        return self._exchange(self.core.hello())

    def poll(self):
        return self._exchange(self.core.request())

    def close(self):
        if self.socket:
            self.socket.close()
        self.socket=None

    def __enter__(self):
        return self.open()

    def __exit__(self,*args):
        self.close()


class LoopbackPC(PCClient):
    def __init__(self,private_key,peer,**kwargs):
        if not isinstance(peer,tuple) or len(peer)!=2 or peer[0]!='127.0.0.1':
            raise ProtocolError('Loopback demo accepts only 127.0.0.1:port')
        super().__init__(private_key,peer,enabled=True,**kwargs)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--loopback-demo',action='store_true')
    parser.add_argument('--port',type=int)
    parser.add_argument('--samples',type=int,default=50)
    args=parser.parse_args()
    if not args.loopback_demo:
        print('DISABLED: no network or private-key access. Use --loopback-demo for synthetic data only.')
        return 2
    if args.port is None or not 1<=args.samples<=1000:
        parser.error('port and samples 1..1000 are required')
    with LoopbackPC(SYNTHETIC_KEY,('127.0.0.1',args.port)) as pc:
        if not pc.connect():
            raise ProtocolError('Loopback handshake timed out')
        for _ in range(args.samples):
            result=pc.poll()
            if result: print(json.dumps(result),flush=True)
            time.sleep(.011)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
