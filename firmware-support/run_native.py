"""Rebuild the existing native parser/startup fixtures; never contacts hardware.

Run from a compiler-enabled terminal: python -B firmware-support/run_native.py
Supports MSVC cl or a C++17 g++/clang++ supplied with --compiler.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

from verify_sources import verify

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler')
    args = parser.parse_args()
    verified = verify()
    compiler = args.compiler or shutil.which('cl') or shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise SystemExit('Use a compiler-enabled terminal or --compiler (C++17).')
    msvc = Path(compiler).name.lower() in ('cl', 'cl.exe')
    libraries = HERE/'firmware_dependencies/libraries'
    sdk = libraries/'FTServo/src'
    results = {}
    with tempfile.TemporaryDirectory(prefix='xgoduck-native-') as temporary:
        build = Path(temporary)
        cases = [('qmi_sync', [HERE/'tests/qmi_sync.cpp'], [HERE/'tests/qmi_sync'], []),
                 ('joint_limits', [HERE/'tests/joint_limits.cpp'], [], []),
                 ('custom_parser', [HERE/'tests/parser/test_parser.cpp'], [], []),
                 ('route_startup', [HERE/'tests/test_startup.cpp'], [], []),
                 ('route_refresh', [HERE/'tests/test_route_refresh.cpp'], [], [])]
        for mode in ('original', 'candidate'):
            scs = HERE/'baseline/FTServo/src/SCS.cpp' if mode == 'original' else sdk/'SCS.cpp'
            cases.append(('sdk_'+mode, [HERE/'tests/sdk/test_sdk.cpp', scs, sdk/'SCSerial.cpp'],
                          [HERE/'tests/sdk', sdk, HERE/'sketch'],
                          ['ARDUINO=100']+(['SDK_CANDIDATE=1'] if mode == 'candidate' else [])))
        for name, sources, includes, defines in cases:
            binary = build/(name+'.exe')
            command = ([compiler, '/nologo', '/std:c++17', '/EHsc', '/O2', '/W3', '/utf-8']
                       if msvc else [compiler, '-std=c++17', '-O2'])
            command += [('/I' if msvc else '-I')+str(path) for path in includes]
            command += [('/D' if msvc else '-D')+value for value in defines]
            command += [str(path) for path in sources]
            command += ['/Fe:'+str(binary)] if msvc else ['-o', str(binary)]
            compiled = subprocess.run(command, cwd=build, capture_output=True, text=True,
                                      encoding='utf-8', errors='replace')
            if compiled.returncode:
                raise RuntimeError('Native compile failed: '+name+'\n'+compiled.stdout+compiled.stderr)
            checked = subprocess.run([str(binary)], cwd=build, capture_output=True, text=True,
                                     encoding='utf-8', errors='replace')
            if checked.returncode:
                raise RuntimeError('Native fixture failed: '+name+'\n'+checked.stdout+checked.stderr)
            try:
                result = json.loads(checked.stdout)
            except ValueError:
                result = {'output': checked.stdout.strip()}
            results[name] = {'passed': True, 'result': result}
    value = {'passed': True, 'source_verification': verified, 'native': results,
             'scope': 'desktop fake I2C/UART/clock/route only', 'arm_recompiled': False,
             'hardware_access': False, 'network_access': False}
    validation = HERE/'validation'
    validation.mkdir(exist_ok=True)
    (validation/'native-result.json').write_text(json.dumps(value, indent=2)+'\n', 'utf-8')
    print(json.dumps(value, indent=2))


if __name__ == '__main__':
    main()
