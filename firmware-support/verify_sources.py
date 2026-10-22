"""Offline source hashes, native fixture binding and source-only package checks."""
import hashlib
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent


def extract_parser(path):
    text = path.read_text('utf-8').replace('\r\n', '\n')
    globals_text = text[text.index('static VoltageSampler displayVoltage;'):text.index('static int motorIndexById(')]
    globals_text = '\n'.join(line for line in globals_text.splitlines()
                             if not line.startswith('#include') and not line.startswith('extern '))
    functions = []
    for name in ('motorIndexById', 'hostWord', 'hostVelSignMag', 'applyMotorFb',
                 'resetServoParser', 'parseServoRx'):
        match = re.search(r'^static [^\n]*\b'+name+r'\(', text, re.M)
        if not match:
            raise ValueError('Parser source function missing: '+name)
        end = text.index('\n}\n', match.start())+3
        functions.append(text[match.start():end])
    return globals_text+'\n\n'+'\n'.join(functions)


def verify():
    lock = json.loads((HERE/'SOURCE_LOCK.json').read_text('utf-8'))['files_sha256']
    for relative, expected in lock.items():
        path = (HERE/relative).resolve()
        path.relative_to(HERE)
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Source hash mismatch: '+relative)
    for mode, source in [('original', HERE/'baseline/sketch/scs_bus.h'),
                         ('candidate', HERE/'sketch/scs_bus.h')]:
        fixture = (HERE/'tests/parser'/(mode+'_actual.inc')).read_text('utf-8')
        if extract_parser(source) != fixture:
            raise ValueError('Actual parser fixture mismatch: '+mode)
    sdk = HERE/'firmware_dependencies/libraries/FTServo/src'
    text = (sdk/'HLSCL.cpp').read_text('utf-8')
    extracted = ''
    for signature in ('HLSCL::HLSCL()', 'int HLSCL::EnableTorque(u8 ID, u8 Enable)'):
        start = text.index(signature)
        extracted += text[start:text.index('}', text.index('{', start))+1]+'\n'
    if extracted != (HERE/'tests/sdk/HLSCL_actual.inc').read_text('utf-8'):
        raise ValueError('Actual SDK constructor/torque fixture mismatch')
    files = [p for p in HERE.rglob('*') if p.is_file()]
    forbidden = ('.elf', '.bin', '.exe', '.obj', '.pyc', '.onnx', '.dll', '.lib', '.a')
    for path in files:
        raw = path.read_bytes()
        if path.suffix.lower() in forbidden or raw.startswith((b'\x7fELF', b'MZ', b'\xcf\xfa\xed\xfe')):
            raise ValueError('Binary/model/cache in source package: '+path.relative_to(HERE).as_posix())
        if re.search(rb'[CD]:[\\/](?:AI_Workspace|Users)[\\/]', raw, re.I) or (b'28277'+b'6529') in raw:
            raise ValueError('Private machine path/identity in package: '+path.relative_to(HERE).as_posix())
    return {'passed': True, 'locked_input_files': len(lock), 'package_files': len(files),
            'bytes': sum(p.stat().st_size for p in files),
            'actual_parser_fixture_bindings': 2, 'actual_sdk_fixture_bindings': 1,
            'hardware_access': False, 'network_access': False}


if __name__ == '__main__':
    print(json.dumps(verify(), indent=2))
