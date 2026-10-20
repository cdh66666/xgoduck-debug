"""Portable local source bindings. No machine receipts, network or hardware IO."""
import copy,hashlib,json
from pathlib import Path

def sha256(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def _root(root):return Path(root).resolve().parent
def runtime_manifest(root,**kwargs):
    repo=_root(root);value=json.loads((repo/'RELEASE_RUNTIME.json').read_text('utf-8'))
    if value.get('schema')!='xgoduck-portable-release-runtime-v1':raise ValueError('Wrong release source manifest')
    sources={name:item['sha256'] for name,item in value['runtime_sources'].items()}
    return {'new_source_manifest':sources,'changed_files':{},'zero_sha256':None,
            'portable_release':True,'release_verified_on_hardware':False,
            'integrated_calibration':{'portable':True},'display_telemetry':{'portable':True},
            'current_telemetry':{'portable':True} if 'python/current_telemetry.py' in sources else None,
            'stand_profile':{'portable':True} if 'python/release_profile.py' in sources else None,
            'calibration_home':{'portable':True} if 'python/controller.py' in sources else None,
            'source_paths':{name:item['path'] for name,item in value['runtime_sources'].items()},
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
        if sha256(runtime_source(root,name,manifest))!=digest:raise ValueError('Release source changed: '+name)
    return manifest
def baseline_manifest(root):return runtime_manifest(root)
def control_manifest(root):return runtime_manifest(root)
def publish_overlay(*args,**kwargs):raise RuntimeError('Portable publication contains no deployment authority; use a separately reviewed installer')
