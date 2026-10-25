"""Capture source identity at process startup and bind it to live MCU build ID."""
import hashlib
import json
import time

class RuntimeIdentity:
    def __init__(self,root,servo_health):
        self.health=servo_health;self.started=time.time()
        self.files={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in list((root/'python').glob('*.py'))+list((root/'python').glob('*.onnx'))}
        elf=root/'.cache/sketch/sketch.ino.elf'
        self.elf=hashlib.sha256(elf.read_bytes()).hexdigest() if elf.exists() else None
        descriptor=root/'data/firmware-release.json'
        self.release=json.loads(descriptor.read_text()) if descriptor.exists() else {}
    def snapshot(self):
        health=self.health.status();build=health.get('build');age=health.get('age_s')
        fresh=age is not None and age<3
        expected=self.release.get('mcu_build')
        file_match=all(self.files.get(n)==h for n,h in self.release.get('python_sha256',{}).items())
        verified=bool(expected and fresh and build==expected and self.elf==self.release.get('elf_sha256') and file_match)
        return {'release':self.release.get('version'),'started_unix_s':self.started,
                'python_sha256':self.files,'elf_sha256':self.elf,'mcu_build':build if fresh else None,
                'mcu_age_s':age,'running_verified':verified,'health':health,
                'capabilities':{'voltage':True,'servo_current':True,'servo_rx_diagnostics':True,'inference_target_hz':50}}
