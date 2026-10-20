"""Bindings for the installed read-only display telemetry extension."""
import copy
import hashlib
import json
from pathlib import Path

CHANGED=frozenset({'python/main.py','python/controller.py','python/display_telemetry.py',
                   'sketch/sketch.ino','sketch/scs_bus.h','sketch/display_telemetry.h'})
ADDED=frozenset({'python/display_telemetry.py','sketch/display_telemetry.h'})


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def telemetry_manifest(root, baseline):
    root=Path(root).resolve()
    folder=root/'qa/telemetry-r5'
    receipt=folder/'apply-result.json'
    if not receipt.exists():return baseline
    value=json.loads(receipt.read_text('utf-8'))
    if (value.get('ok') is not True or value.get('phase')!='activated' or
            value.get('motion_started') is not False or value.get('zero_unchanged') is not True or
            value.get('all_data_unchanged') is not True or value.get('firmware_uploads')!=1):
        raise ValueError('Display telemetry installation is not verified')
    before=baseline['new_source_manifest'];after=value.get('sources_sha256',{})
    revision=value.get('display_revision','r3')
    if revision not in ('r3','r4'):raise ValueError('Unknown display telemetry revision')
    if value.get('base_sources_sha256')!=before or value.get('zero_sha256')!=baseline['zero_sha256']:
        raise ValueError('Display telemetry belongs to a different runtime or zero')
    if revision=='r3':
        if value.get('sources_before')!=before:raise ValueError('Display telemetry base differs')
    else:
        previous=folder/'r3-activation.json'
        if value.get('previous_activation_sha256')!=sha(previous):raise ValueError('Previous display installation changed')
        installed=json.loads(previous.read_text('utf-8'))
        if (installed.get('ok') is not True or installed.get('phase')!='activated' or
                installed.get('base_sources_sha256')!=before or value.get('sources_before')!=installed['sources_sha256'] or
                installed.get('zero_sha256')!=baseline['zero_sha256']):
            raise ValueError('Display bias update belongs to a different installed runtime')
        delta={name for name in set(value['sources_before'])|set(after) if value['sources_before'].get(name)!=after.get(name)}
        if delta!={'python/display_telemetry.py','sketch/sketch.ino','sketch/display_telemetry.h'}:
            raise ValueError('Display bias update changed unrelated runtime sources')
    if value.get('base_wireless_receipt_sha256')!=sha(root/'integrated-calibration-20261020/wireless-host-result.json'):
        raise ValueError('Display telemetry wireless base changed')
    changes={name for name in set(before)|set(after) if before.get(name)!=after.get(name)}
    if changes!=CHANGED or set(after)-set(before)!=ADDED or set(before)-set(after):
        raise ValueError('Display telemetry exceeded its six source changes')
    audit=folder/('arm-build-r4' if revision=='r4' else 'arm-build-r3')/'build-audit.json'
    if value.get('arm_audit_sha256')!=sha(audit):
        raise ValueError('Display telemetry ARM build changed')
    built=json.loads(audit.read_text('utf-8'))
    if built.get('passed') is not True or built.get('dangerous_relocations'):
        raise ValueError('Display telemetry ARM build did not pass')
    for key,name in [('elf_sha256','sketch.ino.elf'),('zsk_sha256','sketch.ino.elf-zsk.bin')]:
        digest=built['artifacts'][name]['sha256']
        if value.get('build',{}).get(key)!=digest or sha(audit.parent/'build'/name)!=digest:
            raise ValueError('Display telemetry installed binary differs from ARM build')
    candidate=folder/('r4-candidate' if revision=='r4' else 'candidate')
    for name in CHANGED:
        if after.get(name)!=sha(candidate/name) or built['source_sha256'].get(name)!=after[name]:
            raise ValueError('Display telemetry source changed: '+name)
    result=copy.deepcopy(baseline)
    result['new_source_manifest']=after
    result['changed_files'].update({name:after[name] for name in CHANGED})
    result['display_telemetry']={'receipt':str(receipt),'sha256':sha(receipt),
                                 'candidate_root':str(candidate)}
    return result


def telemetry_source(name, manifest):
    bound=manifest.get('display_telemetry')
    return Path(bound['candidate_root'])/name if bound and name in CHANGED else None
