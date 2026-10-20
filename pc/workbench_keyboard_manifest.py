"""Resolve the explicitly deployed keyboard extension without network or IO writes."""
import copy
import hashlib
import json
from pathlib import Path

POINTER = 'qa/noeva-velocity-20261020/runtime-extension/active-deployment.json'
CHANGES = {'python/controller.py', 'python/keyboard_runtime.py', 'python/xgoduck_fast_forward.onnx'}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def keyboard_manifest(root, baseline):
    pointer = Path(root)/POINTER
    if not pointer.exists():
        return baseline
    active = json.loads(pointer.read_text('utf-8'))
    if active.get('schema') != 'xgoduck-keyboard-extension-active-v1':
        raise ValueError('Unrecognized keyboard deployment pointer')
    receipt = Path(active['receipt_path'])
    if not receipt.is_absolute() or digest(receipt) != active.get('receipt_sha256'):
        raise ValueError('Keyboard deployment receipt does not match its binding')
    deployed = json.loads(receipt.read_text('utf-8'))
    if (deployed.get('schema') != 'xgoduck-keyboard-extension-deployment-v1' or
            deployed.get('ok') is not True or deployed.get('phase') != 'activated' or
            deployed.get('zero_unchanged') is not True or deployed.get('motion_started') is not False or
            deployed.get('firmware_uploads') != 0):
        raise ValueError('Keyboard extension has no verified disarmed activation')
    previous = baseline['new_source_manifest']
    if deployed.get('base_sources_sha256') != previous:
        raise ValueError('Keyboard extension belongs to another control baseline')
    overrides = active.get('overrides', {})
    changes = set(overrides)
    if changes != CHANGES:
        raise ValueError('Keyboard extension exceeded its declared control/model files')
    admission_path = Path(active['admission_path'])
    admission_sha = active.get('admission_sha256')
    if (not admission_path.is_absolute() or digest(admission_path) != admission_sha or
            deployed.get('admission_sha256') != admission_sha):
        raise ValueError('Keyboard navigation admission does not match its deployed binding')
    admission = json.loads(admission_path.read_text('utf-8'))
    navigation = admission.get('navigation_scope', {})
    if (admission.get('schema') != 'xgoduck-keyboard-navigation-admission-v1' or
            admission.get('result') != 'PASS_LIMITED_KEYBOARD_SCOPE' or
            admission.get('limited_forward_deployable_offline') is not True or
            admission.get('manual_pick_deployable_offline') is not True or
            admission.get('navigation_deployable_offline') is not True or
            admission.get('all_core_deployable_offline') is not False or
            navigation.get('mode') != 'retained_official' or
            navigation.get('artifact_sha256') != previous['python/xgoduck_walk.onnx'] or
            admission.get('original_walk_sha256') != previous['python/xgoduck_walk.onnx'] or
            navigation.get('command_points') != [[-.3,0,0],[0,0,1],[0,0,-1]]):
        raise ValueError('The deployed keyboard scope lacks the admitted manual/forward/navigation behavior')
    expected = dict(previous)
    for name, artifact in overrides.items():
        source = Path(artifact['source_path'])
        if not source.is_absolute() or digest(source) != artifact.get('sha256'):
            raise ValueError('Keyboard runtime source changed: '+name)
        expected[name] = artifact['sha256']
    if (deployed.get('sources_sha256') != expected or
            deployed.get('original_walk_sha256') != previous['python/xgoduck_walk.onnx']):
        raise ValueError('Keyboard activation does not prove the complete source set')
    result = copy.deepcopy(baseline)
    result['new_source_manifest'] = expected
    result['changed_files'].update({name:expected[name] for name in changes})
    result['zero_sha256'] = deployed['zero_sha256']
    result['keyboard_extension'] = active
    result['keyboard_admission'] = admission
    return result


def keyboard_source(name, manifest):
    artifact = manifest.get('keyboard_extension', {}).get('overrides', {}).get(name)
    return Path(artifact['source_path']) if artifact else None
