"""Read-only candidate dependency check. Does not patch global Arduino caches."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root/'component-origin.json').read_text('utf-8'))
count = 0
for name, item in manifest.items():
    if not name.replace('\\', '/').startswith('firmware_dependencies/libraries/'):
        continue
    candidate = root/name
    if candidate.is_symlink() or not candidate.is_file():
        raise SystemExit('Missing or redirected candidate dependency: '+name)
    if hashlib.sha256(candidate.read_bytes()).hexdigest() != item['sha256']:
        raise SystemExit('Candidate dependency hash mismatch: '+name)
    count += 1
if count == 0:
    raise SystemExit('No pinned candidate dependencies found')
print(json.dumps({'verified_dependency_files': count, 'global_cache_modified': False,
                  'arm_compile_verified': False, 'flash_performed': False}))