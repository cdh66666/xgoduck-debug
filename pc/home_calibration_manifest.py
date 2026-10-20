"""Verified hold-only return to the operator's calibrated mechanical middle."""
import copy,hashlib,json
from pathlib import Path

def home_manifest(root,baseline):
 root=Path(root);folder=root/'qa/calibration-dialog-home-20261020';receipt=folder/'activate-result.json'
 if not receipt.exists():return baseline
 v=json.loads(receipt.read_text('utf-8'));before=baseline['new_source_manifest'];after=v.get('sources_sha256',{})
 source=folder/'candidate/python/controller.py'
 if not (v.get('ok') is True and v.get('phase')=='activated' and v.get('sources_before')==before
  and set(after)==set(before) and {n for n in before if before[n]!=after[n]}=={'python/controller.py'}
  and v.get('zero_unchanged') is True and v.get('motion_started') is False and v.get('firmware_uploads')==0
  and hashlib.sha256(source.read_bytes()).hexdigest()==after['python/controller.py']):
  raise ValueError('Calibrated-middle HOME installation is not verified')
 result=copy.deepcopy(baseline);result['new_source_manifest']=after
 result['changed_files']['python/controller.py']=after['python/controller.py']
 result['zero_sha256']=v['zero_sha256']
 result['calibration_home']={'source':str(source),'receipt':str(receipt)}
 return result

def home_source(name,manifest):
 v=manifest.get('calibration_home')
 return Path(v['source']) if v and name=='python/controller.py' else None
