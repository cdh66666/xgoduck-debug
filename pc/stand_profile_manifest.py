"""Bind the installed zero-speed standing/getup HTTP admission profile."""
import copy
import hashlib
import json
from pathlib import Path

FILES={'python/main.py','python/release_profile.py'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stand_manifest(root,baseline):
    root=Path(root);folder=root/'qa/release-stand-getup-20261020';receipt=folder/'activate-result.json'
    if not receipt.exists():return baseline
    value=json.loads(receipt.read_text('utf-8'))
    before,after=baseline['new_source_manifest'],value.get('sources_sha256',{})
    if not (value.get('ok') is True and value.get('phase')=='activated'
            and value.get('sources_before')==before
            and value.get('base_receipt_sha256')==sha(root/'qa/performance-release-20261020/activate-result.json')
            and {n for n in set(before)|set(after) if before.get(n)!=after.get(n)}==FILES
            and set(after)-set(before)=={'python/release_profile.py'} and not set(before)-set(after)
            and value.get('zero_sha256')==baseline['zero_sha256'] and value.get('zero_unchanged') is True
            and value.get('motion_started') is False and value.get('firmware_uploads')==0):
        raise ValueError('Standing/getup publication profile is not verified')
    candidate=folder/'candidate'
    for name in FILES:
        if sha(candidate/name)!=after[name]:raise ValueError('Standing profile source changed: '+name)
    result=copy.deepcopy(baseline);result['new_source_manifest']=after
    result['changed_files'].update({n:after[n] for n in FILES})
    result['stand_profile']=dict(candidate_root=str(candidate),receipt=str(receipt),sha256=sha(receipt))
    return result


def stand_source(name,manifest):
    value=manifest.get('stand_profile')
    return Path(value['candidate_root'])/name if value and name in FILES else None
