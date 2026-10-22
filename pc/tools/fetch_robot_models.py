"""Read the four release-bound ONNX files from the configured, pinned duck."""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
NAMES = {'xgoduck_walk.onnx', 'xgoduck_getup.onnx', 'xgoduck_pick.onnx', 'xgoduck_roulade.onnx'}
MAX_BYTES = 4*1024*1024
CHUNK_BYTES = 32*1024
REMOTE_APP = '/home/arduino/ArduinoApps/xgoduck-commissioning'
REMOTE_SOURCE = r'''import base64,hashlib,json,sys
from pathlib import Path
APP=Path('/home/arduino/ArduinoApps/xgoduck-commissioning')
NAMES={'xgoduck_walk.onnx','xgoduck_getup.onnx','xgoduck_pick.onnx','xgoduck_roulade.onnx'}
MAX_BYTES=4*1024*1024
p=json.load(sys.stdin)
def model(name):
    if name not in NAMES:raise ValueError('model filename is not allowed')
    file=APP/'python'/name
    if file.resolve().parent!=(APP/'python').resolve():raise ValueError('model path escapes app')
    size=file.stat().st_size
    if not 0<size<=MAX_BYTES:raise ValueError('model size is outside 1..4MiB')
    return file,size
if p.get('operation')=='metadata':
    result={}
    for name in sorted(NAMES):
        file,size=model(name);digest=hashlib.sha256()
        with file.open('rb') as stream:
            for block in iter(lambda:stream.read(131072),b''):digest.update(block)
        result[name]={'sha256':digest.hexdigest(),'size':size}
    print(json.dumps({'models':result}))
elif p.get('operation')=='read_chunk':
    file,size=model(p.get('name'))
    offset,length=p.get('offset'),p.get('length')
    if (type(offset) is not int or type(length) is not int or not 0<=offset<size
            or not 0<length<=32768 or size!=p.get('size')):raise ValueError('invalid model chunk request')
    with file.open('rb') as stream:stream.seek(offset);data=stream.read(length)
    print(json.dumps({'offset':offset,'size':size,'data':base64.b64encode(data).decode('ascii')}))
else:raise ValueError('only metadata and read_chunk are allowed')
'''


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fetch_models(repo_root=ROOT.parent, execute=None):
    """Use pinned model_transport, or an injected offline test transport."""
    repo = Path(repo_root).resolve()
    external = json.loads((repo/'RELEASE_RUNTIME.json').read_text('utf-8')).get('external_models', {})
    require(set(external) == {'python/'+name for name in NAMES}, 'Release must bind exactly the four supported models')
    for name, row in external.items():
        require(isinstance(row, dict) and row.get('path') == 'robot-app/'+name
                and isinstance(row.get('sha256'), str) and re.fullmatch('[0-9a-f]{64}', row['sha256']),
                'Invalid release model path/SHA256: '+name)
    if execute is None:
        sys.path.insert(0, str(repo/'pc'))
        from model_transport import execute

    def remote(payload):
        return execute(repo/'pc', REMOTE_SOURCE, payload, timeout=20)

    metadata = remote({'operation': 'metadata'}).get('models', {})
    require(set(metadata) == NAMES, 'Duck metadata did not contain exactly the four supported models')
    # Check every actual model before downloading any bytes.
    for name, row in metadata.items():
        require(isinstance(row, dict) and type(row.get('size')) is int and 0 < row['size'] <= MAX_BYTES,
                'Duck model exceeds the 4MiB size limit: '+name)
        require(row.get('sha256') == external['python/'+name]['sha256'],
                'Duck model SHA256 differs from this release: '+name)
    results = {}
    for name in sorted(NAMES):
        row = external['python/'+name]
        destination = repo/row['path']
        destination.parent.mkdir(parents=True, exist_ok=True)
        size = metadata[name]['size']
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=destination.parent, prefix='.'+name+'.', suffix='.part', delete=False) as stream:
                temporary = Path(stream.name)
                digest = hashlib.sha256()
                for offset in range(0, size, CHUNK_BYTES):
                    length = min(CHUNK_BYTES, size-offset)
                    reply = remote({'operation': 'read_chunk', 'name': name, 'offset': offset, 'length': length, 'size': size})
                    require(reply.get('offset') == offset and reply.get('size') == size,
                            'Duck model chunk offset/size changed: '+name)
                    encoded = reply.get('data')
                    require(isinstance(encoded, str) and len(encoded) <= 65536, 'Duck model chunk exceeds 64KiB base64')
                    block = base64.b64decode(encoded, validate=True)
                    require(len(block) == length, 'Duck model chunk was truncated: '+name)
                    stream.write(block)
                    digest.update(block)
                require(digest.hexdigest() == row['sha256'], 'Downloaded model SHA256 mismatch: '+name)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, destination)
            temporary = None
            results['python/'+name] = {'path': row['path'], 'sha256': row['sha256'], 'bytes': size}
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fetch', action='store_true', help='Read and SHA256-verify the four models using configured pinned SSH')
    args = parser.parse_args(argv)
    if not args.fetch:
        parser.print_help()
        return 0
    try:
        print(json.dumps({'ok': True, 'models': fetch_models()}, indent=2))
    except (OSError, ValueError, RuntimeError, TimeoutError) as error:
        print('Model download failed: '+str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
