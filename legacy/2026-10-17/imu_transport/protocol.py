"""Authenticated latest-IMU datagrams. Pure state machines: no sockets or files.

Only challenge/response and read-latest messages exist. A calibration_key supplied
by the caller is domain-separated for this protocol; it is never loaded here.
"""
from collections import Counter
from dataclasses import dataclass, field
import hashlib
import hmac
import json
import math
import re
import secrets
import threading
import time

if __package__:
    from .pose_codec import decode_pose
else:
    from pose_codec import decode_pose

VERSION = 1
MAX_DATAGRAM = 1200  # Below ordinary IPv4/IPv6 Ethernet fragmentation thresholds.
MAX_RTT_MS = 80.0
MAX_AGE_MS = 150.0
SESSION_TTL_MS = 30000
MAX_SESSIONS = 8
MAX_REQUESTS_PER_SESSION = 4096
MOTOR_IDS = (10,11,12,13,14,20,21,22,23,24,30,31,32,33,34)
TOKEN = re.compile(r'[0-9a-f]{32}\Z')
HELLO_FIELDS = {'v','kind','client','rid','nonce'}
READ_FIELDS = HELLO_FIELDS | {'host','sid'}
ACK_FIELDS = READ_FIELDS | {'instance','restart','ttl_ms'}
SAMPLE_FIELDS = READ_FIELDS | {'status'}


class ProtocolError(ValueError):
    pass


def derive_key(private_key):
    if not isinstance(private_key, str) or not re.fullmatch(r'[0-9a-f]{64}', private_key):
        raise ProtocolError('Expected caller-supplied 64-hex calibration key')
    return hmac.new(bytes.fromhex(private_key), b'XGO-IMU-LATEST-UDP-v1', hashlib.sha256).digest()


def canonical(value):
    try:
        return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('ascii')
    except (TypeError, ValueError, UnicodeError, OverflowError, RecursionError) as exc:
        raise ProtocolError('Invalid JSON value') from exc


def encode(payload, key):
    body = canonical(payload)
    packet = canonical({'payload':payload,'mac':hmac.new(key,body,hashlib.sha256).hexdigest()})
    if not 1 <= len(packet) <= MAX_DATAGRAM:
        raise ProtocolError('Datagram exceeds 1200 bytes')
    return packet


def _object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ProtocolError('Duplicate JSON field')
        value[key] = item
    return value


def decode(packet, key):
    if not isinstance(packet, bytes) or not 1 <= len(packet) <= MAX_DATAGRAM:
        raise ProtocolError('Invalid datagram size')
    try:
        envelope = json.loads(packet.decode('ascii'), object_pairs_hook=_object,
                              parse_constant=lambda _: (_ for _ in ()).throw(ProtocolError('Nonfinite JSON')))
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ProtocolError('Malformed datagram') from exc
    if not isinstance(envelope,dict) or set(envelope) != {'payload','mac'}:
        raise ProtocolError('Invalid authentication envelope')
    payload, mac = envelope['payload'], envelope['mac']
    if not isinstance(payload,dict) or not isinstance(mac,str) or not re.fullmatch(r'[0-9a-f]{64}',mac):
        raise ProtocolError('Invalid authentication fields')
    expected = hmac.new(key,canonical(payload),hashlib.sha256).hexdigest()
    if not hmac.compare_digest(expected,mac):
        raise ProtocolError('Invalid HMAC')
    return payload


def integer(value, maximum, label):
    if type(value) is not int or not 0 <= value <= maximum:
        raise ProtocolError(f'Invalid {label}')
    return value


def finite(value, label):
    if type(value) not in (int,float):
        raise ProtocolError(f'Invalid {label}')
    try:
        result = float(value)
    except (OverflowError,ValueError) as exc:
        raise ProtocolError(f'Invalid {label}') from exc
    if not math.isfinite(result):
        raise ProtocolError(f'Invalid {label}')
    return result


def token(value):
    if not isinstance(value,str) or not TOKEN.fullmatch(value):
        raise ProtocolError('Invalid identity/nonce')
    return value


def message(payload, fields, kind):
    if set(payload) != fields or type(payload.get('v')) is not int or payload['v'] != VERSION or payload.get('kind') != kind:
        raise ProtocolError('Unknown protocol message or fields')
    token(payload['client']); token(payload['nonce'])
    if integer(payload['rid'],2**64-1,'request sequence') == 0:
        raise ProtocolError('Zero request sequence')
    if 'host' in fields:
        token(payload['host']); token(payload['sid'])


def compact_status(raw):
    """Whitelist real live_status data. Do not carry calibration/control authority."""
    if not isinstance(raw,dict):
        raise ProtocolError('Status provider did not return an object')
    result = {}
    for name in ('mcu_seq','mcu_sample_us','mcu_imu_us','mcu_restarts','mcu_work_us'):
        result[name] = integer(raw.get(name),0xffffffff,name)
    work = result['mcu_work_us']
    # sample_us is MCU cycle START, so current-cycle imu_us may be later.
    # Require the real work_us instead of guessing a permitted future skew.
    lag = (result['mcu_sample_us']+work-result['mcu_imu_us'])&0xffffffff
    if work >= MAX_AGE_MS*1000 or lag >= MAX_AGE_MS*1000:
        raise ProtocolError('IMU is stale or beyond the known snapshot end')
    result['app_instance_id'] = token(raw.get('app_instance_id'))
    result['received'] = integer(raw.get('received'),2**64-1,'received')
    if result['received'] == 0 or raw.get('imu_ok') is not True:
        raise ProtocolError('No valid IMU observation')
    age = finite(raw.get('feedback_age_ms'),'source feedback age')
    if not 0 <= age or age+lag/1000 >= MAX_AGE_MS:
        raise ProtocolError('Source feedback is stale')
    result.update(feedback_age_ms=age,imu_snapshot_age_ms=lag/1000,imu_ok=True)
    for name in ('acc','gyro'):
        value = raw.get(name)
        if not isinstance(value,(list,tuple)) or len(value) != 3:
            raise ProtocolError(f'Invalid {name} vector')
        result[name] = [finite(component,name) for component in value]
    hz = finite(raw.get('feedback_hz'),'feedback_hz')
    if not 0 <= hz <= 1000:
        raise ProtocolError('Invalid feedback frequency')
    ids = raw.get('servo_ids')
    if (not isinstance(ids,(list,tuple)) or len(ids)>15 or any(type(sid) is not int or sid not in MOTOR_IDS for sid in ids)
            or len(set(ids)) != len(ids)):
        raise ProtocolError('Invalid observed servo IDs')
    result.update(feedback_hz=hz,servo_ids=list(ids))
    if 'native_pose_compact' in raw:
        try:
            decode_pose(raw['native_pose_compact'])
        except (ValueError, TypeError):
            pass  # A bad optional pose cannot interrupt otherwise valid IMU.
        else:
            result['native_pose_compact'] = raw['native_pose_compact']
    # Labels cannot grant motion permission; all authority stays on existing SSH.
    result.update(mode='read_only',host_read_only=True,no_rpc_writes=True)
    return result


def identity(status):
    return status['app_instance_id'],status['mcu_restarts']


@dataclass
class Bucket:
    rate: float
    capacity: float
    tokens: float
    at: float

    def allow(self, now):
        if now < self.at:
            raise ProtocolError('Monotonic clock regressed')
        self.tokens = min(self.capacity,self.tokens+(now-self.at)*self.rate)
        self.at = now
        if self.tokens < 1:
            return False
        self.tokens -= 1
        return True


@dataclass
class Session:
    client: str
    peer: tuple
    identity: tuple
    expires: float
    last_rid: int
    bucket: Bucket
    nonces: set = field(default_factory=set)


class HostProtocol:
    """Only dependency on the application is the existing telemetry.live_status callable."""
    def __init__(self, private_key, live_status, *, clock=time.monotonic):
        if not callable(live_status):
            raise ProtocolError('A read-only live_status callable is required')
        self.key, self.live_status, self.clock = derive_key(private_key),live_status,clock
        self.host = secrets.token_hex(16)
        now = finite(clock(),'host monotonic time')
        self.last_clock = now
        self.global_bucket = Bucket(200,8,8,now)
        self.hello_bucket = Bucket(2,2,2,now)
        self.sessions, self.hellos, self.stats = {},{},Counter()
        self.lock = threading.Lock()

    def _time(self):
        now = finite(self.clock(),'host monotonic time')
        if now < self.last_clock:
            raise ProtocolError('Host monotonic clock regressed')
        self.last_clock = now
        return now

    def _snapshot(self):
        before = self._time()
        try:
            raw = self.live_status()
        except Exception as exc:
            # This is an application callback boundary; its failure must not kill
            # a reader thread or create a synthetic replacement observation.
            raise ProtocolError('Read-only provider unavailable') from exc
        status = compact_status(raw)
        # Include callback/serialization preparation time. PC adds conservative
        # full monotonic RTT, so no remote wall-clock synchronisation is needed.
        status['feedback_age_ms'] += (self._time()-before)*1000
        if status['feedback_age_ms']+status['imu_snapshot_age_ms'] >= MAX_AGE_MS:
            raise ProtocolError('Provider became stale')
        return status

    def handle(self, packet, peer):
        with self.lock:
            try:
                now = self._time()
                payload = decode(packet,self.key)
                if not self.global_bucket.allow(now):
                    raise ProtocolError('Global rate limit')
                self.sessions = {sid:s for sid,s in self.sessions.items() if now < s.expires}
                self.hellos = {client:h for client,h in self.hellos.items() if now < h['expires']}
                if payload.get('kind') == 'hello':
                    message(payload,HELLO_FIELDS,'hello')
                    previous = self.hellos.get(payload['client'])
                    if previous and (payload['rid'] <= previous['rid'] or payload['nonce'] in previous['nonces']):
                        raise ProtocolError('Replayed/reordered handshake')
                    if not self.hello_bucket.allow(now):
                        raise ProtocolError('Handshake rate limit')
                    owned = [sid for sid,s in self.sessions.items() if s.client == payload['client']]
                    if len(self.sessions)-len(owned) >= MAX_SESSIONS or (not previous and len(self.hellos)>=MAX_SESSIONS*2):
                        raise ProtocolError('Session capacity')
                    status = self._snapshot()
                    for sid in owned:
                        del self.sessions[sid]
                    sid = secrets.token_hex(16)
                    self.sessions[sid] = Session(payload['client'],peer,identity(status),now+SESSION_TTL_MS/1000,
                        payload['rid'],Bucket(100,8,8,now))
                    nonces = previous['nonces'] if previous else set()
                    if len(nonces) >= MAX_REQUESTS_PER_SESSION:
                        raise ProtocolError('Handshake nonce capacity')
                    nonces.add(payload['nonce'])
                    self.hellos[payload['client']] = dict(rid=payload['rid'],nonces=nonces,expires=now+2*SESSION_TTL_MS/1000)
                    response = {**payload,'kind':'hello_ack','host':self.host,'sid':sid,
                        'instance':status['app_instance_id'],'restart':status['mcu_restarts'],'ttl_ms':SESSION_TTL_MS}
                elif payload.get('kind') == 'read':
                    message(payload,READ_FIELDS,'read')
                    session = self.sessions.get(payload['sid'])
                    if payload['host'] != self.host or not session or session.peer != peer or session.client != payload['client']:
                        raise ProtocolError('Unknown/expired/foreign session')
                    if payload['rid'] <= session.last_rid or payload['nonce'] in session.nonces:
                        raise ProtocolError('Replayed/reordered request')
                    if len(session.nonces) >= MAX_REQUESTS_PER_SESSION or not session.bucket.allow(now):
                        raise ProtocolError('Session rate/nonce limit')
                    # Consume authentication identity even if the source is stale.
                    session.last_rid = payload['rid']; session.nonces.add(payload['nonce'])
                    status = self._snapshot()
                    if identity(status) != session.identity:
                        raise ProtocolError('Source rebooted; new handshake required')
                    response = {**payload,'kind':'sample','status':status}
                else:
                    raise ProtocolError('Only hello/read messages are supported')
                result = encode(response,self.key)
                self.stats['responses'] += 1
                return result
            except (ProtocolError,TypeError,ValueError,OverflowError) as exc:
                self.stats[str(exc)] += 1
                return None


class PCProtocol:
    """One outstanding read, fresh nonce each round, no retransmission/backlog replay."""
    def __init__(self, private_key, peer, *, clock=time.monotonic, wall_clock=time.time):
        self.key,self.peer,self.clock,self.wall_clock = derive_key(private_key),peer,clock,wall_clock
        self.client = secrets.token_hex(16)
        self.rid,self.pending,self.session,self.source_identity = 0,None,None,None
        self.last_source = None
        self.last_clock = finite(clock(),'PC monotonic time')
        self.stats = Counter()

    def _time(self):
        now = finite(self.clock(),'PC monotonic time')
        if now < self.last_clock:
            raise ProtocolError('PC monotonic clock regressed')
        self.last_clock = now
        return now

    def expire(self):
        now = self._time()
        if self.pending and (now-self.pending['sent'])*1000 > MAX_RTT_MS:
            self.pending = None
            self.stats['request_timeout'] += 1

    def discard_pending(self, reason='request_timeout'):
        if self.pending is not None:
            self.pending = None
            self.stats[reason] += 1

    def rebind_readonly_source(self):
        """Explicit reader recovery only. Preserve client/rid; next nonce/sid is new.

        A coordinating reader must retain same-identity source progress and retire
        old epochs. This cannot restore any calibration or actuator session.
        """
        self.discard_pending('abandoned_for_readonly_rebind')
        self.session = None
        self.source_identity = None
        self.last_source = None

    def _request(self, kind):
        self.expire()
        if self.pending:
            raise ProtocolError('Only one request may be outstanding')
        now = self._time()
        if kind == 'read' and (not self.session or now >= self.session['expires']):
            raise ProtocolError('Handshake required')
        self.rid += 1
        if self.rid >= 2**64:
            raise ProtocolError('Request counter exhausted; new client required')
        payload = dict(v=VERSION,kind=kind,client=self.client,rid=self.rid,nonce=secrets.token_hex(16))
        if kind == 'read':
            payload.update(host=self.session['host'],sid=self.session['sid'])
        self.pending = dict(payload=payload,sent=now)
        return encode(payload,self.key)

    def hello(self):
        return self._request('hello')

    def request(self):
        return self._request('read')

    def accept(self, packet, peer):
        now = self._time()
        if peer != self.peer or not self.pending:
            raise ProtocolError('Unsolicited/foreign datagram')
        rtt_ms = (now-self.pending['sent'])*1000
        if rtt_ms > MAX_RTT_MS:
            self.pending = None
            raise ProtocolError('Late datagram')
        payload = decode(packet,self.key)
        request = self.pending['payload']
        kind = 'hello_ack' if request['kind'] == 'hello' else 'sample'
        message(payload,ACK_FIELDS if kind == 'hello_ack' else SAMPLE_FIELDS,kind)
        if any(payload[name] != request[name] for name in ('client','rid','nonce')):
            raise ProtocolError('Wrong request nonce/sequence')
        if kind == 'hello_ack':
            source = token(payload['instance']),integer(payload['restart'],0xffffffff,'source restart epoch')
            if type(payload['ttl_ms']) is not int or payload['ttl_ms'] != SESSION_TTL_MS:
                raise ProtocolError('Unknown session TTL')
            if self.source_identity is not None and source != self.source_identity:
                raise ProtocolError('Source identity changed; new explicit client required')
            self.source_identity = source
            self.session = dict(host=payload['host'],sid=payload['sid'],expires=self.pending['sent']+SESSION_TTL_MS/1000)
            self.pending = None
            self.stats['handshakes'] += 1
            return {'handshake':True,'source_identity':source}
        if not self.session or now >= self.session['expires'] or any(payload[name] != request[name] for name in ('host','sid')):
            raise ProtocolError('Old/expired host session')
        status = compact_status(payload['status'])
        if identity(status) != self.source_identity:
            raise ProtocolError('Source boot/instance changed')
        # Known age components only: MCU->Linux pre-receive buffering is not
        # measured by this contract. No physical end-to-end latency claim.
        status['feedback_age_ms'] += status['imu_snapshot_age_ms']+rtt_ms
        if status['feedback_age_ms'] >= MAX_AGE_MS:
            raise ProtocolError('Total observation age is stale')
        source = tuple(status[name] for name in ('mcu_seq','mcu_sample_us','mcu_imu_us'))
        if self.last_source is not None and any(not 0 < ((current-previous)&0xffffffff) < 0x80000000
                                                for current,previous in zip(source,self.last_source)):
            self.pending = None
            raise ProtocolError('Duplicate/regressed source sequence or sensor clock')
        checked_at = finite(self.wall_clock(),'PC wall clock')
        if checked_at <= 0:
            raise ProtocolError('Invalid local assessment timestamp')
        self.last_source = source
        self.pending = None
        self.stats['accepted_samples'] += 1
        return {'status':status,'read_only':True,'checked_at':checked_at}
