"""Verify the installed Host-only control scheduler correction."""
import copy
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def performance_manifest(root, baseline):
    root = Path(root)
    folder = root / 'qa/performance-release-20261020'
    receipt = folder / 'activate-result.json'
    if not receipt.exists():
        return baseline
    value = json.loads(receipt.read_text('utf-8'))
    before, after = baseline['new_source_manifest'], value.get('sources_sha256', {})
    parent = root / 'qa/current-monitor-20261020/hostfix-result.json'
    if not (value.get('ok') is True and value.get('phase') == 'activated'
            and value.get('sources_before') == before and set(after) == set(before)
            and {n for n in before if after[n] != before[n]} == {'python/controller.py'}
            and value.get('base_receipt_sha256') == sha(parent)
            and value.get('zero_sha256') == baseline['zero_sha256']
            and value.get('zero_unchanged') is True and value.get('motion_started') is False
            and value.get('firmware_uploads') == 0):
        raise ValueError('Host scheduler installation is not verified')
    source = folder / 'candidate/python/controller.py'
    if sha(source) != after['python/controller.py']:
        raise ValueError('Host scheduler source changed after installation')
    result = copy.deepcopy(baseline)
    result['new_source_manifest'] = after
    result['changed_files']['python/controller.py'] = after['python/controller.py']
    result['performance_release'] = dict(receipt=str(receipt), sha256=sha(receipt), source=str(source))
    return result


def performance_source(name, manifest):
    value = manifest.get('performance_release')
    return Path(value['source']) if value and name == 'python/controller.py' else None
