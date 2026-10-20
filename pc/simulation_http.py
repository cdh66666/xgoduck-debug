"""Local virtual-physics HTTP boundary, separate from every hardware API."""
import json
from pathlib import Path
import re
import threading
from urllib.parse import parse_qs, urlsplit

from simulation_service import SimulationService

_services = {}
_lock = threading.Lock()
_ID = r'[a-z0-9][a-z0-9-]{0,79}'
_FIELDS = {
    'reset': set(), 'policy': {'enabled'}, 'pause': {'paused'},
    'drop': {'seed', 'orientation_quat'}, 'grab': {'position'},
    'release': set(), 'push': {'force', 'duration', 'body', 'point'}, 'close': set(),
    'command': {'twist'},
}


def service(root):
    key = str(Path(root).resolve())
    with _lock:
        if key not in _services:
            _services[key] = SimulationService(key)
        return _services[key]


def geometry(root, model_id):
    from simulation_catalog import geometry as current_geometry
    return current_geometry(root, model_id)


def close(root):
    with _lock:
        instance = _services.pop(str(Path(root).resolve()), None)
    if instance:
        instance.close_all()


def handle_get(handler, path, root, token):
    match = re.fullmatch(r'/api/models/(' + _ID + r')/geometry', path)
    if match:
        try:
            handler.send(200, geometry(root, match[1]))
        except (ValueError, KeyError, OSError) as error:
            handler.send(404, {'ok': False, 'error': str(error)})
        return True
    if path not in ('/api/simulation/events', '/api/simulation/state'):
        return False
    try:
        query = parse_qs(urlsplit(handler.path).query, strict_parsing=True)
        if set(query) != {'session'} or len(query['session']) != 1:
            raise ValueError('仿真会话无效')
        session = service(root).get(query['session'][0])
        if path.endswith('/state'):
            handler.send(200, {'ok': True, 'state': session.snapshot()})
            return True
        session.subscribe()
    except (ValueError, KeyError, OSError) as error:
        handler.send(404, {'ok': False, 'error': str(error)})
        return True
    old_timeout = handler.connection.gettimeout()
    try:
        handler.connection.settimeout(2)
        handler.send_response(200)
        handler.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        handler.send_header('Cache-Control', 'no-store')
        handler.send_header('Connection', 'close')
        handler.send_header('X-Accel-Buffering', 'no')
        handler.end_headers()
        seq = -1
        while not session.closed:
            state = session.wait_state(seq, timeout=1)
            if state.get('seq', -1) > seq:
                raw = ('event: state\ndata: ' + json.dumps(state, separators=(',', ':'), allow_nan=False) + '\n\n').encode('utf-8')
                seq = state.get('seq', -1)
            else:
                raw = b': keepalive\n\n'
            handler.wfile.write(raw)
            handler.wfile.flush()
    except (OSError, ValueError):
        pass
    finally:
        session.unsubscribe()
        handler.close_connection = True
        try:
            handler.connection.settimeout(old_timeout)
        except OSError:
            pass
    return True


def handle_post(handler, path, root, token):
    if path != '/api/simulation':
        return False
    if handler.headers.get('X-Token') != token:
        handler.send(403, {'ok': False, 'code': 'local_session_expired', 'error': '页面连接已更新，请重新打开模型'})
        return True
    try:
        length = int(handler.headers.get('Content-Length', '0'))
        if not 0 < length <= 4096:
            raise ValueError('请求大小无效')
        payload = json.loads(handler.rfile.read(length))
        if not isinstance(payload, dict):
            raise ValueError('请求格式无效')
        op = payload.get('op')
        if op == 'create':
            if set(payload) != {'op', 'model_id'}:
                raise ValueError('仿真创建字段无效')
            session = service(root).create(payload['model_id'])
            handler.send(200, {'ok': True, 'session': session.id, 'state': session.snapshot()})
        else:
            if op not in _FIELDS or set(payload) - {'session', 'op'} - _FIELDS[op]:
                raise ValueError('仿真操作或字段无效')
            if op in ('policy', 'pause') and type(payload.get('enabled' if op == 'policy' else 'paused')) is not bool:
                raise ValueError('运行状态必须是布尔值')
            session_id = payload.get('session')
            if op == 'close':
                service(root).close(session_id)
                handler.send(200, {'ok': True})
            else:
                session = service(root).get(session_id)
                state = session.command({key: value for key, value in payload.items() if key != 'session'})
                handler.send(200, {'ok': True, 'state': state})
    except (ValueError, KeyError, TypeError) as error:
        handler.send(400, {'ok': False, 'error': str(error)[:500]})
    except (OSError, RuntimeError, TimeoutError) as error:
        handler.send(503, {'ok': False, 'error': str(error)[:500]})
    return True
