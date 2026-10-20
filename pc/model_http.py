"""Local model-library HTTP boundary; no device action on GET or page navigation."""
import json
import mimetypes
import re
import threading
from email import policy
from email.parser import BytesParser
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

MAX_UPLOAD = 96 * 1024 * 1024
_libraries = {}
_lock = threading.Lock()
_ID = r'[a-zA-Z0-9_-]{1,80}'


def library(root):
    from model_library import ModelLibrary
    key = str(Path(root).resolve())
    with _lock:
        if key not in _libraries:
            _libraries[key] = ModelLibrary(Path(root))
        return _libraries[key]


def multipart(content_type, body):
    if not content_type.startswith('multipart/form-data;') or len(content_type) > 512:
        raise ValueError('请选择模型文件后导入')
    if not body or len(body) > MAX_UPLOAD:
        raise ValueError('上传文件超过大小限制')
    head = ('Content-Type: ' + content_type + '\r\nMIME-Version: 1.0\r\n\r\n').encode('ascii')
    message = BytesParser(policy=policy.default).parsebytes(head + body)
    if not message.is_multipart():
        raise ValueError('上传格式无效')
    fields, files = {}, {}
    for index, part in enumerate(message.iter_parts()):
        if index >= 5 or part.is_multipart():
            raise ValueError('上传字段过多')
        name = part.get_param('name', header='content-disposition')
        if name not in ('model', 'preview', 'name', 'description') or name in fields or name in files:
            raise ValueError('上传字段无效或重复')
        raw = part.get_payload(decode=True)
        if not isinstance(raw, bytes):
            raise ValueError('文件内容无效')
        filename = part.get_filename()
        if name in ('model', 'preview'):
            if not filename or not raw:
                if name == 'preview':
                    continue
                raise ValueError('请选择 ONNX 模型')
            files[name] = {'filename': filename, 'data': raw}
        else:
            if len(raw) > 4096:
                raise ValueError('名称或说明过长')
            fields[name] = raw.decode('utf-8', errors='strict')
    return fields, files


def _file(handler, file_path, *, byte_range=True):
    file_path = Path(file_path)
    size = file_path.stat().st_size
    start, end, status = 0, size - 1, 200
    requested = handler.headers.get('Range') if byte_range else None
    if requested:
        match = re.fullmatch(r'bytes=(\d*)-(\d*)', requested)
        if not match or not any(match.groups()):
            return handler.send(416, {'error': '无效的读取范围'})
        first, last = match.groups()
        if first:
            start, end = int(first), min(int(last), size - 1) if last else size - 1
        else:
            start, end = max(0, size - int(last)), size - 1
        if start > end or start >= size:
            handler.send_response(416)
            handler.send_header('Content-Range', f'bytes */{size}')
            handler.send_header('Content-Length', '0')
            handler.end_headers()
            return
        status = 206
    handler.send_response(status)
    handler.send_header('Content-Type', mimetypes.guess_type(file_path.name)[0] or 'application/octet-stream')
    handler.send_header('Content-Length', str(max(0, end - start + 1)))
    handler.send_header('Accept-Ranges', 'bytes')
    handler.send_header('X-Content-Type-Options', 'nosniff')
    handler.send_header('Cache-Control', 'private, max-age=3600')
    if status == 206:
        handler.send_header('Content-Range', f'bytes {start}-{end}/{size}')
    handler.end_headers()
    if handler.command == 'HEAD':
        return
    with file_path.open('rb') as source:
        source.seek(start)
        remaining = end - start + 1
        while remaining > 0:
            block = source.read(min(remaining, 65536))
            if not block:
                break
            handler.wfile.write(block)
            remaining -= len(block)


def replay(root, model_id):
    root = Path(root).resolve()
    info = library(root).replay_info(model_id)
    if not info:
        raise ValueError('此模型尚未添加训练回放')
    preview = Path(root).parent / 'training/runs/r5-weighed-825p5g-20261017/preview'
    model = json.loads((preview / 'model.json').read_text('utf-8'))
    records = []
    for record in model['records']:
        # The mesh references are from the fixed, saved training geometry.
        name = Path(record['file']).name
        records.append({key: value for key, value in record.items() if key != 'source'} | {
            'file': '/model-replay-assets/assets/' + name})
    trajectory = json.loads(Path(info['trajectory_path']).read_text('utf-8'))
    metrics = json.loads(Path(info['metrics_path']).read_text('utf-8'))
    frames = [{'t': frame['t'], 'body_poses': frame['body_poses']} for frame in trajectory['frames']]
    return {'label': info.get('label', model_id), 'evidence_kind': 'simulation',
            'model': {'version': model['version'], 'records': records}, 'frames': frames,
            'metrics': metrics, 'contains_reset': trajectory.get('contains_reset', False),
            'mass_kg': trajectory.get('mass_kg'), 'source_sha256': info.get('source_sha256')}


def handle_get(handler, path, root, token):
    if path in ('/models', '/models.html'):
        handler.send(200, (Path(root) / 'models.html').read_text('utf-8').replace('__TOKEN__', token), 'text/html; charset=utf-8')
        return True
    if path.startswith('/model-replay-assets/'):
        relative = path.removeprefix('/model-replay-assets/')
        base = (Path(root).parent / 'training/runs/r5-weighed-825p5g-20261017/preview').resolve()
        target = (base / relative).resolve()
        if (not relative.startswith(('assets/', 'vendor/')) or '..' in Path(relative).parts
                or not target.is_relative_to(base) or target.suffix not in ('.stl', '.obj', '.js') or not target.is_file()):
            handler.send(404, {'error': '预览资源不存在'})
        else:
            _file(handler, target)
        return True
    if path == '/api/models':
        try:
            handler.send(200, library(root).list_models())
        except (ValueError, OSError) as error:
            handler.send(503, {'error': str(error)})
        return True
    match = re.fullmatch(r'/api/models/(' + _ID + r')/(preview|replay)', path)
    if match:
        try:
            if match[2] == 'replay':
                handler.send(200, replay(root, match[1]))
            else:
                media = library(root).media_path(match[1])
                if not media:
                    raise ValueError('此模型尚未添加训练动态图')
                _file(handler, media)
        except (ValueError, KeyError, OSError) as error:
            handler.send(404, {'error': str(error)})
        return True
    return False


def handle_simulation_asset(handler, path, root):
    """Serve only the physical viewer's meshes/vendor; no old deployment API."""
    if not path.startswith('/model-replay-assets/'):
        return False
    from simulation_catalog import RUN
    relative=path.removeprefix('/model-replay-assets/')
    if relative.startswith('assets/'):
        base=Path(root).parent/'simulation-model/assets'
        name=relative.removeprefix('assets/')
        allowed={'.stl','.obj'}
    elif relative.startswith('vendor/'):
        base=Path(root)/'model-assets/vendor'
        name=relative.removeprefix('vendor/')
        allowed={'.js'}
    else:
        handler.send(404, {'error':'仿真资源不存在'})
        return True
    base=base.resolve()
    target=(base/name).resolve()
    if '..' in Path(name).parts or not target.is_relative_to(base) or target.suffix.lower() not in allowed or not target.is_file():
        handler.send(404, {'error':'仿真资源不存在'})
    else:
        _file(handler,target)
    return True


def handle_post(handler, path, root, token):
    if not path.startswith('/api/models/'):
        return False
    if handler.headers.get('X-Token') != token:
        handler.send(403, {'code': 'local_session_expired', 'error': '页面连接已更新，请重新操作'})
        return True
    try:
        length = int(handler.headers.get('Content-Length', '0'))
        limit = MAX_UPLOAD if path == '/api/models/import' else 4096
        if not 0 < length <= limit:
            raise ValueError('请求大小无效')
        if path == '/api/models/import':
            fields, files = multipart(handler.headers.get('Content-Type', ''), handler.rfile.read(length))
            result = library(root).import_model(fields, files)
        else:
            payload = json.loads(handler.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError('请求格式无效')
            if path == '/api/models/stop':
                result = library(root).stop()
            else:
                match = re.fullmatch(r'/api/models/(' + _ID + r')/deploy', path)
                if not match:
                    handler.send(404, {'error': '模型操作不存在'})
                    return True
                result = library(root).deploy(match[1])
        handler.send(200 if result.get('ok') else 409, result)
    except (ValueError, KeyError) as error:
        handler.send(400, {'ok': False, 'error': str(error)})
    except (OSError, RuntimeError, TimeoutError, AssertionError) as error:
        handler.send(503, {'ok': False, 'error': '模型操作未完成：' + str(error)[:600]})
    return True
