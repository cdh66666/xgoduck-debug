"""Network-disabled by default. Explicit demo binds only 127.0.0.1 with synthetic data."""
import argparse
import json
import socket
import threading
import time
import ipaddress

if __package__:
    from .protocol import HostProtocol, MAX_DATAGRAM, ProtocolError
else:  # Direct, explicitly synthetic CLI; application imports use the package.
    from protocol import HostProtocol, MAX_DATAGRAM, ProtocolError

SYNTHETIC_KEY = '91'*32  # Public test fixture; never used for a real installation.


class SyntheticTelemetry:
    def __init__(self):
        self.started = time.monotonic()

    def live_status(self):
        index = int((time.monotonic()-self.started)*100)+1
        return dict(app_instance_id='ab'*16,mcu_restarts=0,mcu_seq=index,mcu_sample_us=index*10000,
                    mcu_imu_us=index*10000+2000,mcu_work_us=6000,received=index,feedback_hz=100.,feedback_age_ms=1.,
                    imu_ok=True,acc=[0.,0.,-9.80665],gyro=[0.,0.,0.],servo_ids=[])


def private_literal(value):
    try: address=ipaddress.IPv4Address(value)
    except (ValueError,TypeError,ipaddress.AddressValueError) as exc:raise ProtocolError('Literal private IPv4 required') from exc
    networks=('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16','127.0.0.0/8')
    if str(address)!=value or not any(address in ipaddress.ip_network(network) for network in networks):
        raise ProtocolError('Explicit private/loopback IPv4 required; no wildcard/DNS/public bind')
    return value


class HostEndpoint:
    """Configured application endpoint. Inert construction; enabled must be explicit."""
    def __init__(self, protocol, port=0, *, bind='127.0.0.1', enabled=False):
        if not isinstance(protocol,HostProtocol) or type(port) is not int or not 0 <= port <= 65535:
            raise ProtocolError('Invalid host arguments')
        self.protocol,self.port = protocol,port
        self.bind,self.enabled=private_literal(bind),enabled is True
        self.socket,self.thread = None,None
        self.stop_event = threading.Event()

    def start(self):
        if not self.enabled:
            raise ProtocolError('UDP endpoint is disabled')
        if self.socket is not None:
            raise ProtocolError('Host already started')
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.bind((self.bind,self.port))
        self.socket.settimeout(.02)
        self.address = self.socket.getsockname()
        self.thread = threading.Thread(target=self._serve,daemon=True)
        self.thread.start()
        return self

    def _serve(self):
        while not self.stop_event.is_set():
            try:
                packet,peer = self.socket.recvfrom(MAX_DATAGRAM+1)
                response = self.protocol.handle(packet,peer)
                if response is not None:
                    self.socket.sendto(response,peer)
            except socket.timeout:
                continue
            except OSError:
                if not self.stop_event.is_set():
                    self.protocol.stats['socket_error'] += 1

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1)
        if self.socket is not None:
            self.socket.close()
        self.socket = None

    def __enter__(self):
        return self.start()

    def __exit__(self,*args):
        self.close()


class LoopbackHost(HostEndpoint):
    """Explicit test harness; binds only 127.0.0.1."""
    def __init__(self,protocol,port=0):
        super().__init__(protocol,port,bind='127.0.0.1',enabled=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--loopback-demo',action='store_true')
    parser.add_argument('--port',type=int,default=0)
    parser.add_argument('--seconds',type=float,default=10.)
    args=parser.parse_args()
    if not args.loopback_demo:
        print('DISABLED: no network or private-key access. Use --loopback-demo for synthetic data only.')
        return 2
    if not 0 < args.seconds <= 60:
        parser.error('seconds must be 0..60')
    protocol=HostProtocol(SYNTHETIC_KEY,SyntheticTelemetry().live_status)
    with LoopbackHost(protocol,args.port) as host:
        print(json.dumps({'scope':'synthetic loopback only','address':host.address}),flush=True)
        host.stop_event.wait(args.seconds)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
