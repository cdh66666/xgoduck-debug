"""Bind the installed read-only current extension after the keyboard runtime."""
import copy
import hashlib
import json
from pathlib import Path

CHANGED = frozenset({'python/controller.py', 'python/wire.py', 'python/current_telemetry.py',
                     'sketch/sketch.ino', 'sketch/scs_bus.h', 'sketch/current_telemetry.h'})
ADDED = frozenset({'python/current_telemetry.py', 'sketch/current_telemetry.h'})


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def current_manifest(root, baseline):
    folder = Path(root) / 'qa/current-monitor-20261020'
    receipt = folder / 'activate-result.json'
    if not receipt.exists():
        return baseline
    value = json.loads(receipt.read_text('utf-8'))
    if not (value.get('ok') is True and value.get('phase') == 'activated'
            and value.get('motion_started') is False and value.get('zero_unchanged') is True
            and value.get('existing_data_unchanged') is True and value.get('firmware_uploads') == 1):
        raise ValueError('Current telemetry installation is not verified')
    before, after = baseline['new_source_manifest'], value.get('sources_sha256', {})
    if value.get('sources_before') != before or value.get('zero_sha256') != baseline['zero_sha256']:
        raise ValueError('Current telemetry belongs to a different installed runtime')
    changes = {n for n in set(before) | set(after) if before.get(n) != after.get(n)}
    if changes != CHANGED or set(after) - set(before) != ADDED or set(before) - set(after):
        raise ValueError('Current telemetry changed unrelated sources')
    audit_path = folder / 'arm-build/build-audit.json'
    if value.get('arm_audit_sha256') != sha(audit_path):
        raise ValueError('Current telemetry build audit changed')
    audit = json.loads(audit_path.read_text('utf-8'))
    if audit.get('passed') is not True or audit.get('dangerous_relocations'):
        raise ValueError('Current telemetry ARM build failed')
    for key, name in [('elf_sha256', 'sketch.ino.elf'), ('zsk_sha256', 'sketch.ino.elf-zsk.bin')]:
        digest = audit['artifacts'][name]['sha256']
        if value.get('build', {}).get(key) != digest or sha(audit_path.parent / 'build' / name) != digest:
            raise ValueError('Current telemetry binary binding changed')
    candidate = folder / 'candidate'
    for name, digest in after.items():
        if audit['source_sha256'].get(name) != digest:
            raise ValueError('Current telemetry build source differs: ' + name)
    for name in CHANGED:
        if sha(candidate / name) != after[name]:
            raise ValueError('Current telemetry source changed: ' + name)
    result = copy.deepcopy(baseline)
    result['new_source_manifest'] = after
    result['changed_files'].update({n: after[n] for n in CHANGED})
    result['current_telemetry'] = dict(candidate_root=str(candidate), receipt=str(receipt), sha256=sha(receipt))
    hostfix = folder / 'hostfix-result.json'
    if hostfix.exists():
        fixed = json.loads(hostfix.read_text('utf-8'))
        module = 'python/current_telemetry.py'
        updated = fixed.get('sources_sha256', {})
        if not (fixed.get('ok') is True and fixed.get('phase') == 'activated'
                and fixed.get('base_receipt_sha256') == sha(receipt)
                and fixed.get('sources_before') == after and fixed.get('zero_unchanged') is True
                and fixed.get('motion_started') is False and fixed.get('firmware_uploads') == 0
                and fixed.get('zero_sha256') == baseline['zero_sha256']
                and set(updated) == set(after)
                and {n for n in after if updated[n] != after[n]} == {module}):
            raise ValueError('Current logger Host-only update is not verified')
        source = folder / 'r2-candidate' / module
        if sha(source) != updated[module]:
            raise ValueError('Current logger batching source changed')
        result['new_source_manifest'] = updated
        result['changed_files'][module] = updated[module]
        result['current_telemetry'].update(hostfix_source=str(source), hostfix_receipt=str(hostfix),
                                            hostfix_sha256=sha(hostfix))
    return result


def current_source(name, manifest):
    binding = manifest.get('current_telemetry')
    if binding and name == 'python/current_telemetry.py' and binding.get('hostfix_source'):
        return Path(binding['hostfix_source'])
    return Path(binding['candidate_root']) / name if binding and name in CHANGED else None
