"""Versioned V7 delta metadata. Legacy installers cannot overwrite this contract."""
import hashlib
import json
from pathlib import Path

VERSION='2026-10-15-wireless-calibration-v7'
ARCHIVE='xgoduck-slow-home-hold-v7-20261015.zip'
DESCRIPTOR='xgoduck-slow-home-hold-v7.json'
LEGACY_UPDATE_PATHS=frozenset(('/api/wireless-calibration/update','/api/wireless-calibration/usb',
                              '/api/provision/start','/api/provision/update'))

def program_release_info(root):
    root=Path(root);ready=False
    try:
        manifest=json.loads((root/'downloads'/DESCRIPTOR).read_text('utf-8'))
        archive=root/'downloads'/ARCHIVE
        ready=(manifest.get('version')==VERSION and manifest.get('home_contract_revision')==5
               and manifest.get('archive')==ARCHIVE and archive.is_file()
               and archive.stat().st_size==manifest.get('bytes')
               and hashlib.sha256(archive.read_bytes()).hexdigest()==manifest.get('sha256'))
    except (OSError,ValueError,TypeError):
        pass
    return {'latest_version':VERSION,'mcu_contract_revision':3,'home_contract_revision':5,
            'package_ready':False,'reviewed_bundle_ready':bool(ready),
            'install_method':'reviewed_delta','download_url':'/downloads/'+ARCHIVE if ready else None,
            'legacy_installer_allowed':False}

def legacy_update_blocked(path):
    return path in LEGACY_UPDATE_PATHS
