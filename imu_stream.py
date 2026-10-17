"""Opt-in IMU-only relay. Does not replace calibration/control status or sessions."""
from device_config import DEVICE_SERIAL
from copy import deepcopy
import json
import ipaddress
import math
from pathlib import Path
import re
import threading
import time
from urllib.parse import urlsplit

from imu_transport.pipeline import pipeline_reader_if_enabled as reader_if_enabled
from imu_transport.host import private_literal

SERIAL = DEVICE_SERIAL
ADDRESS_CHECK_S = .5
PRIVATE_NETWORKS = tuple(ipaddress.ip_network(x) for x in
                         ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))


def _cached_peer(root):
    """Local discovery hint only. A UDP HMAC handshake must still authenticate it.

    Match the existing cached-candidate newest-file ordering, but reject an
    invalid newest record instead of silently reviving an older DHCP address.
    No discovery scan, DNS, SSH command or control session is performed here.
    """
    directory = Path(root) / 'qa'
    if directory.is_symlink():raise ValueError('Invalid discovery cache directory')
    paths = [directory / name for name in
             ('wireless-connection.json', 'provisioning-state.json', 'discovery-last.json')]
    existing = [path for path in paths if path.exists()]
    if not existing:raise ValueError('No cached private IMU address')
    path = max(existing, key=lambda item: item.stat().st_mtime_ns)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > 65536:
        raise ValueError('Invalid discovery cache')
    value = json.loads(path.read_text('utf-8-sig'))
    if not isinstance(value,dict) or value.get('serial') != SERIAL:
        raise ValueError('Cached IMU serial mismatch')
    url = value.get('network_url')
    if url is None and isinstance(value.get('board_ip'),str):
        url = 'http://' + value['board_ip'] + ':9527'
    if not isinstance(url,str) or len(url)>256:raise ValueError('Invalid cached IMU address')
    try:
        parsed = urlsplit(url)
        address = ipaddress.IPv4Address(parsed.hostname)
        if (parsed.scheme != 'http' or parsed.port != 9527 or parsed.path not in ('','/')
                or parsed.username or parsed.password or parsed.query or parsed.fragment
                or not any(address in network for network in PRIVATE_NETWORKS)):
            raise ValueError('Invalid cached IMU address')
        if value.get('board_ip') is not None and value['board_ip'] != str(address):
            raise ValueError('Conflicting cached IMU addresses')
        return str(address)
    except (TypeError,ValueError) as exc:
        raise ValueError('Invalid cached IMU address') from exc


def effective_config(root):
    config = config_state(root)
    if not config.get('enabled'):return None
    if set(config) != {'enabled','peer_ip','port'}:
        raise ValueError('Invalid IMU configuration fields')
    port = config['port']
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('Invalid IMU UDP port')
    address = _cached_peer(root) if config['peer_ip'] == 'auto_cached' else private_literal(config['peer_ip'])
    return {'enabled':True,'peer_ip':address,'port':port}


def config_state(root):
    path = Path(root) / 'tools/.private/imu-transport.json'
    if not path.exists():return {'enabled':False}
    if path.is_symlink() or path.stat().st_size > 4096:raise ValueError('Invalid IMU configuration')
    value = json.loads(path.read_text('utf-8'))
    if not isinstance(value,dict) or type(value.get('enabled',False)) is not bool:
        raise ValueError('Invalid IMU configuration')
    return value


def _reader_factory(root, config):
    if not config.get('enabled'):raise ValueError('IMU datagrams are disabled')
    auth_path=Path(root)/'tools/.private/wireless-calibration-auth.json'
    if auth_path.is_symlink() or auth_path.stat().st_size>4096:raise ValueError('Invalid local IMU credential')
    auth=json.loads(auth_path.read_text('utf-8'))
    if (not isinstance(auth,dict) or type(auth.get('version')) is not int
            or auth.get('version')!=1 or str(auth.get('serial'))!=DEVICE_SERIAL
            or not isinstance(auth.get('key'),str) or not re.fullmatch('[0-9a-f]{64}',auth['key'])):
        raise ValueError('Invalid local IMU credential')
    reader=reader_if_enabled(config,auth['key'])
    if reader is None:raise ValueError('IMU datagrams are disabled')
    return reader


class ImuStream:
    def __init__(self, reader_factory, *, target_resolver=None, clock=time.monotonic, wall=time.time):
        self.reader_factory,self.clock,self.wall=reader_factory,clock,wall
        self.target_resolver=target_resolver
        self.condition=threading.Condition()
        self.clients=0;self.last_client=clock();self.thread=None
        self.counter=0;self.packet=None;self.received_at=0.;self.key=None
        self.reader_generation=0
        self.source_identity=None;self.source_progress=None;self.retired_identities=set()

    def acquire(self):
        with self.condition:
            self.clients+=1;self.last_client=self.clock()
            if self.thread is None or not self.thread.is_alive():
                self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def release(self):
        with self.condition:
            self.clients=max(0,self.clients-1);self.last_client=self.clock()

    def active(self):
        with self.condition:return self.clients>0 or self.clock()-self.last_client<.5

    def offline(self):
        with self.condition:
            if self.packet and self.packet.get('error'):return
            self.packet={'error':True};self.counter+=1;self.condition.notify_all()

    def publish(self, packet, *, generation=None):
        status=packet.get('status') if isinstance(packet,dict) else None
        checked=packet.get('checked_at') if isinstance(packet,dict) else None
        if not isinstance(status,dict) or type(checked) not in (int,float) or not math.isfinite(checked):return False
        age=status.get('feedback_age_ms');transit=self.wall()-checked
        if (type(age) not in (int,float) or not math.isfinite(age) or not math.isfinite(transit)
                or transit < -.001 or age<0 or age+max(0,transit)*1000>=150):return False
        key=(status.get('app_instance_id'),status.get('mcu_restarts'),status.get('mcu_seq'),status.get('mcu_imu_us'))
        if (not isinstance(key[0],str) or not re.fullmatch('[0-9a-f]{32}',key[0])
                or any(type(value) is not int or not 0<=value<=0xffffffff for value in key[1:])):
            return False
        source,progress=key[:2],key[2:]
        with self.condition:
            if generation is not None and generation != self.reader_generation:return False
            if source in self.retired_identities:return False
            if source==self.source_identity and self.source_progress is not None:
                if any(not 0<((new-old)&0xffffffff)<0x80000000
                       for new,old in zip(progress,self.source_progress)):return False
            elif self.source_identity is not None:
                if len(self.retired_identities)>=64:return False
                self.retired_identities.add(self.source_identity)
            value=deepcopy(status);value['feedback_age_ms']=age+max(0,transit)*1000
            self.source_identity,self.source_progress=source,progress
            self.key=key;self.counter+=1;self.received_at=self.clock()
            self.packet={'status':value,'read_only':True,'checked_at':self.wall()}
            self.condition.notify_all();return True

    def wait(self,cursor,timeout=.1):
        with self.condition:
            self.condition.wait_for(lambda:self.counter!=cursor,timeout)
            packet=self.packet
            if packet and isinstance(packet.get('status'),dict):
                status=deepcopy(packet['status'])
                status['feedback_age_ms']+=max(0,(self.clock()-self.received_at)*1000)
                packet=({'status':status,'read_only':True,'checked_at':self.wall()}
                        if status['feedback_age_ms']<150 else {'error':True})
            return self.counter,packet

    def run(self):
        reader=None;retry_at=0.;retiring=False
        applied=None;next_address_check=0.;generation=self.reader_generation
        def close_reader(value):
            if value is not None:
                try:value.close()
                except (OSError,ValueError):self.offline()
        try:
            while self.active():
                try:
                    if self.target_resolver is not None and self.clock()>=next_address_check:
                        next_address_check=self.clock()+ADDRESS_CHECK_S
                        target=self.target_resolver()
                        if target != applied:
                            self.offline()
                            with self.condition:self.reader_generation+=1
                            close_reader(reader);reader=None
                            applied=deepcopy(target);retry_at=0.
                    if self.target_resolver is not None and applied is None:
                        self.offline();time.sleep(.01);continue
                    if reader is None:
                        if self.clock()<retry_at:
                            time.sleep(.01);continue
                        reader=self.reader_factory(applied) if self.target_resolver is not None else self.reader_factory()
                        generation=self.reader_generation
                    packet=reader.poll()
                    if packet is not None:self.publish(packet,generation=generation)
                    _,current=self.wait(self.counter,0)
                    if current is None or current.get('error'):self.offline()
                except (OSError,ValueError):
                    self.offline()
                    with self.condition:self.reader_generation+=1
                    close_reader(reader)
                    reader=None;retry_at=self.clock()+.5
                    if self.target_resolver is not None:
                        applied=None;next_address_check=retry_at
                time.sleep(.002)
            retiring=True
        finally:
            close_reader(reader)
            # acquire() may arrive while the retiring worker closes its socket.
            # Transfer ownership under the same lock, so clients never lose it.
            with self.condition:
                self.thread=None
                if retiring and self.clients>0:
                    self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()


_sources={};_lock=threading.Lock()
def source_for_imu(root):
    config=config_state(root)
    if not config.get('enabled'):raise ValueError('IMU datagrams are disabled')
    identity=str(Path(root).resolve())
    with _lock:
        if identity not in _sources:
            _sources[identity]=ImuStream(lambda config:_reader_factory(root,config),
                                         target_resolver=lambda:effective_config(root))
        return _sources[identity]
