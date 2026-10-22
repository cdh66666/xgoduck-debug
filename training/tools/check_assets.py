"""Verify physics assets and the exact explicit whitelist of bundled weights."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
manifest=json.loads((ROOT/'ASSET_MANIFEST.json').read_text('utf-8'))
for rel,binding in manifest['bindings'].items():
 p=ROOT/binding['path'];assert p.is_file() and hashlib.sha256(p.read_bytes()).hexdigest()==binding['sha256'],rel
files=[p for p in ROOT.rglob('*') if p.is_file() and not set(p.relative_to(ROOT).parts)&{'outputs','.venv','__pycache__'}]
weights=[p.relative_to(ROOT).as_posix() for p in files if p.suffix.lower() in {'.pt','.pth','.onnx','.ckpt','.safetensors','.npz','.npy'}]
declared=json.loads((ROOT/'checkpoints/MANIFEST.json').read_text('utf-8'))['files']
expected={'checkpoints/'+name for name in declared}
assert set(weights)==expected,(weights,sorted(expected))
for name,item in declared.items():
 p=ROOT/'checkpoints'/name
 assert p.stat().st_size==item['bytes'] and hashlib.sha256(p.read_bytes()).hexdigest()==item['sha256'],name
print(json.dumps({'asset_bindings':len(manifest['bindings']),'files':len(files),'weight_files':weights,'weight_whitelist_verified':True,'passed':True}))
