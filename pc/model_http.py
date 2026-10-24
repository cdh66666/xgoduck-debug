"""Read-only mesh/vendor assets for the current local physics viewer."""
import mimetypes
import re
from pathlib import Path


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


def handle_simulation_asset(handler, path, root):
    """Serve only the physical viewer's meshes/vendor; no old deployment API."""
    if not path.startswith('/model-replay-assets/'):
        return False
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
