"""Portable local source bindings. No machine receipts, network or hardware IO."""
import copy,hashlib,json
from pathlib import Path

def sha256(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def _root(root):return Path(root).resolve().parent
def runtime_manifest(root,**kwargs):
    repo=_root(root);value=json.loads((repo/'RELEASE_RUNTIME.json').read_text('utf-8'))
    if value.get('schema')!='xgoduck-portable-release-runtime-v1':raise ValueError('Wrong release source manifest')
    entries={**value['runtime_sources'],**value.get('external_models',{})}
    sources={name:item['sha256'] for name,item in entries.items()}
    return {'new_source_manifest':sources,'changed_files':{},'zero_sha256':None,
            'portable_release':True,'release_verified_on_hardware':False,
            'integrated_calibration':{'portable':True},'display_telemetry':{'portable':True},
            'current_telemetry':{'portable':True} if 'python/current_telemetry.py' in sources else None,
            'stand_profile':None,
            'basic_keyboard_release':{'portable':True} if value.get('release_profile',{}).get('schema')=='xgoduck-basic-keyboard-publication-v2' else None,
            'calibration_home':{'portable':True} if 'python/controller.py' in sources else None,
            'source_paths':{name:item['path'] for name,item in entries.items()},
            'external_models':value.get('external_models',{}),
            'snapshot_source_count':value['snapshot_source_count'],'excluded_models':value['excluded_models']}
def runtime_source(root,name,manifest=None):
    repo=_root(root);manifest=manifest or runtime_manifest(root)
    relative=Path(manifest['source_paths'][name])
    if relative.is_absolute() or '..' in relative.parts:raise ValueError('Release path is not relative')
    result=(repo/relative).resolve();result.relative_to(repo)
    if result.is_symlink():raise ValueError('Linked release source')
    return result
def verified_sources(root):
    manifest=runtime_manifest(root)
    for name,digest in manifest['new_source_manifest'].items():
        path=runtime_source(root,name,manifest)
        # The console talks to the already-installed robot; local ONNX files
        # are only needed for optional local simulation, never its inference.
        if name in manifest['external_models'] and not path.is_file():continue
        if sha256(path)!=digest:raise ValueError('Release source changed: '+name)
    return manifest
def baseline_manifest(root):return runtime_manifest(root)
def control_manifest(root):return runtime_manifest(root)
def publish_overlay(*args,**kwargs):raise RuntimeError('Portable publication contains no deployment authority; use a separately reviewed installer')
