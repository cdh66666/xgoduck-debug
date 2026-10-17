import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import shutil


PC_FILES = (
    'server.py', 'provisioning.py', 'release_policy.py', 'wireless_calibration_proxy.py',
    'wireless_rediscovery.py', 'commissioning_support.py', 'live_relay.py', 'imu_stream.py',
    'index.html', 'numbering.html', 'model-view.html', 'model-view.js',
    'tools/discover_unoq.py', 'tools/discover_wireless_unoq.py', 'tools/deploy_readonly_wifi.py',
)
ASSETS = (
    'commissioning.css', 'simple-debug.css', 'visual-debug.css', 'wireless-calibration.css',
    'duck-dark.css', 'imu-math.js', 'imu-live.js', 'workflow.js', 'simple-debug.js',
    'live-runtime.js', 'visual-debug.js', 'zero-guide-parent.js', 'wireless-calibration.js',
    'communication.js', 'native-pose-parent.js', 'native-pose.js', 'pose-playback.js',
    'relative-heading.js', 'numbering.js', 'imu-model.html', 'imu-model.css', 'imu-model.js',
    'connection-model.html', 'connection-model.css', 'connection-model.js',
    'zero-guide.html', 'zero-guide.css', 'zero-guide.js', 'official-assembly.html',
)
SEQUENCE_ASSETS = ('sequence-calibration.js', 'motor-current.js')
UI_FILES = frozenset(('index.html', 'numbering.html', 'model-view.html', 'model-view.js')) | frozenset(
    'assets/' + name for name in ASSETS + SEQUENCE_ASSETS)
ASSEMBLY_PDF = 'https://github.com/LuwuDynamics/xgoduck_hardware/blob/90ec55634d700c88fd7aa9281c282be30f56e7e0/Assembly_Guide.pdf'


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def imported(source, statement):
    tree = ast.parse(source)
    offset = 0
    for node in tree.body:
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
            offset = node.end_lineno
        elif isinstance(node, ast.ImportFrom) and node.module == '__future__':
            offset = node.end_lineno
        else:
            break
    lines = source.splitlines(keepends=True)
    lines.insert(offset, statement + '\n')
    return ''.join(lines)


def portable(relative, raw, serial):
    changes = []
    if not relative.endswith(('.py', '.html', '.js', '.json')):
        return raw, changes
    source = raw.decode('utf-8-sig')
    if relative.endswith('.py') and (repr(serial) in source or json.dumps(serial) in source):
        source = source.replace(repr(serial), 'DEVICE_SERIAL').replace(json.dumps(serial), 'DEVICE_SERIAL')
        statement = 'from device_config import DEVICE_SERIAL'
        if relative.startswith('tools/'):
            statement = "import sys\nfrom pathlib import Path\nsys.path.insert(0, str(Path(__file__).resolve().parent.parent))\n" + statement
        source = imported(source, statement)
        changes.append('device serial read from local config or XGODUCK_SERIAL')
    if relative == 'server.py':
        source = re.sub(r"^sys\.path\.insert\(0, str\(Path\(__file__\)\.resolve\(\)\.parents\[1\] / 'tools/python-deps'\)\)\s*\n", '', source, flags=re.M)
        changes.append('use installed pyserial instead of workspace dependency path')
    if relative == 'board/python/calibration_host.py':
        source = imported(source, 'from device_config import HOME_TURN_TARGETS')
        source = re.sub(r'^HOME_TURN_EQUIVALENT_TARGETS = .*$', 'HOME_TURN_EQUIVALENT_TARGETS = HOME_TURN_TARGETS', source, flags=re.M)
        changes.append('omit real calibrated encoder values; explicit audited config required')
    if relative == 'assets/wireless-calibration.js':
        source, replaced = re.subn(r'^\s*record\.position===\(\{[^}\r\n]+\}\)\[id\]&&target===\(\{[^}\r\n]+\}\)\[id\]&&\s*\r?\n', '', source, flags=re.M)
        if replaced != 1:
            raise ValueError('Expected exactly one device-specific UI target pair check')
        changes.append('use backend-audited configured target pairs; preserve reference identity and movement bounds')
    if relative == 'provisioning.py':
        source = imported(source, 'import shutil\nimport sys')
        source = re.sub(r'^PWSH = .*$', "PWSH = Path(shutil.which('pwsh') or 'pwsh')", source, flags=re.M)
        source = re.sub(r'^PYTHON = .*$', 'PYTHON = Path(sys.executable)', source, flags=re.M)
        source = source.replace('    def _command(self, mode):\n',
            "    def _command(self, mode):\n        if mode == 'discover':\n            return [str(PYTHON), str(self.project_root / 'tools' / 'discover_wireless_unoq.py')]\n")
        changes.append('discover with active Python; resolve PowerShell from PATH')
    if relative == 'tools/discover_unoq.py':
        source = imported(source, 'import os')
        source = re.sub(r'^MDNS_EXE = .*$', "MDNS_EXE = Path(os.environ['XGODUCK_MDNS']) if os.environ.get('XGODUCK_MDNS') else next(iter(sorted((Path(os.environ.get('LOCALAPPDATA', str(Path.home() / 'AppData' / 'Local'))) / 'Arduino15/packages/builtin/tools/mdns-discovery').glob('*/mdns-discovery.exe'), reverse=True)), Path('mdns-discovery.exe'))", source, flags=re.M)
        changes.append('resolve Arduino mDNS from current Windows profile or XGODUCK_MDNS')
    if relative == 'assets/zero-guide.html':
        source = re.sub(r'<img class="mouth-photo"[^>]*>', '', source)
        source = source.replace('按照片轻闭合，别再压紧。', '轻闭合嘴巴，别再压紧。').replace('你的实物照片 · 已确认轻闭嘴；', '')
        changes.append('omit private reference photo')
    if relative == 'assets/simple-debug.js':
        source = source.replace("fetch('/assets/imu-static-last.json', {cache: 'no-store'})", "Promise.resolve({ok:false})")
        changes.append('omit historical IMU measurement')
    if relative == 'assets/imu-model.js':
        source = source.replace("fetch('./imu-display-bias.json',{cache:'no-store'})", "Promise.resolve({ok:false})")
        changes.append('omit device-specific historical IMU bias')
    if relative in ('model-assets/model-data.json', 'model-assets/model-data-mouth.json'):
        value = json.loads(source)
        def public_paths(item):
            if isinstance(item, dict):
                return {key: public_paths(value) for key, value in item.items()}
            if isinstance(item, list):
                return [public_paths(value) for value in item]
            if isinstance(item, str) and re.match(r'^[A-Za-z]:[\\/]', item):
                return 'source/' + item.replace('\\', '/').split('/xgoduck-replica/', 1)[-1]
            return item
        source = json.dumps(public_paths(value), ensure_ascii=False, indent=2) + '\n'
        changes.append('replace private workspace provenance paths')
    if relative == 'index.html':
        source = source.replace('/downloads/XGO-Duck-Official-Assembly-Guide.pdf', ASSEMBLY_PDF)
        changes.append('link upstream assembly PDF instead of bundling it')
    if relative == 'assets/official-assembly.html':
        source = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>官方组装说明</title><link rel="stylesheet" href="/assets/duck-dark.css"><body class="duck-dark"><main style="max-width:600px;margin:50px auto;padding:20px;font:16px/1.7 sans-serif"><h1>官方组装说明</h1><p>直接查看上游 PDF，仓库无需重复保存 25 页图片。</p><a href="' + ASSEMBLY_PDF + '" target="_blank" rel="noopener">打开 XGO Duck 官方组装 PDF</a></main></body></html>\n'
        changes.append('replace bundled assembly images with upstream PDF link')
    return source.encode('utf-8'), changes


def main():
    parser = argparse.ArgumentParser(description='Sync the small publishable source set without device data.')
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--approved-update', type=Path, action='append', default=[])
    args = parser.parse_args()
    source = args.source.resolve(strict=True)
    target = Path(__file__).resolve().parent.parent
    if source == target or target.is_relative_to(source):
        raise ValueError('Source must be the separate production servo-debug directory')
    release = source / 'qa/repair-20261016/reviewed-recovery-manifest.json'
    reviewed = json.loads(release.read_text(encoding='utf-8-sig'))
    board = source / 'reliability-release-candidate'
    published_path = target / 'SOURCE_MANIFEST.json'
    published = json.loads(published_path.read_text(encoding='utf-8')) if published_path.is_file() else {}
    expected_sources = dict(reviewed['firmware_after'])
    expected_sources.update({relative: item['sha256'] for relative, item in reviewed['board_files'].items()})
    approved_source_paths = {}
    approved_updates = []
    expected_pc_sources = {}
    expected_ui_sources = {}
    firmware_sha256 = reviewed['elf']['sha256']
    sequence_revision = None
    for update_path in args.approved_update:
        update_path = update_path.resolve(strict=True)
        update_raw = update_path.read_bytes()
        update = json.loads(update_raw)
        if update.get('passed') is not True or not isinstance(update.get('changes'), dict):
            raise ValueError('Update manifest is not approved')
        if 'base_source_manifest_sha256' in update and update['base_source_manifest_sha256'] != digest(published_path.read_bytes()):
            raise ValueError('Published source manifest differs from approved update baseline')
        changed_sources = {}
        for relative, item in update['changes'].items():
            if relative.startswith('data/'):
                continue
            if not isinstance(item, dict):
                raise ValueError('Invalid approved source record: ' + relative)
            if ('base_source_manifest_sha256' in update and relative in expected_sources
                    and 'before_sha256' in item and item['before_sha256'] != expected_sources[relative]):
                raise ValueError('Approved source baseline disagrees: ' + relative)
            if relative not in expected_sources:
                if not re.fullmatch(r'python/[A-Za-z_][A-Za-z_0-9]*\.py', relative):
                    raise ValueError('Update changes an untracked source: ' + relative)
                if 'before_sha256' not in item:
                    raise ValueError('Added source needs an explicit baseline: ' + relative)
                previous = published.get('files', {}).get('board/' + relative, {}).get('source_sha256')
                if item['before_sha256'] is None:
                    if previous not in (None, item['sha256']):
                        raise ValueError('New source conflicts with a published source: ' + relative)
                elif previous not in (item['before_sha256'], item['sha256']):
                    raise ValueError('Existing source baseline is not recognized: ' + relative)
            candidate = (update_path.parent / item['source']).resolve(strict=True)
            if not candidate.is_relative_to(update_path.parent) or digest(candidate.read_bytes()) != item['sha256']:
                raise ValueError('Approved source does not match its manifest: ' + relative)
            expected_sources[relative] = item['sha256']
            approved_source_paths[relative] = candidate
            changed_sources[relative] = item['sha256']
        for relative, expected in update.get('unchanged', {}).items():
            if expected_sources.get(relative) != expected:
                raise ValueError('Approved update baseline disagrees: ' + relative)
        for relative, item in update.get('pc_files', {}).items():
            if relative not in PC_FILES:
                raise ValueError('Approved PC source is outside the release file list: ' + relative)
            before = item.get('before_sha256')
            previous = published.get('files', {}).get(relative, {}).get('source_sha256')
            if 'base_source_manifest_sha256' in update and before is not None and before != previous:
                raise ValueError('Approved PC source baseline disagrees: ' + relative)
            candidate = (update_path.parent / item['source']).resolve(strict=True)
            if not candidate.is_relative_to(update_path.parent) or digest(candidate.read_bytes()) != item['sha256']:
                raise ValueError('Approved PC source does not match its manifest: ' + relative)
            expected_pc_sources[relative] = item['sha256']
        for relative, item in update.get('ui_files', {}).items():
            if relative not in UI_FILES:
                raise ValueError('Approved UI source is outside the release file list: ' + relative)
            before = item.get('before_sha256')
            previous = published.get('files', {}).get(relative, {}).get('source_sha256')
            if before != previous:
                raise ValueError('Approved UI source baseline disagrees: ' + relative)
            candidate = (update_path.parent / item['source']).resolve(strict=True)
            if not candidate.is_relative_to(update_path.parent) or digest(candidate.read_bytes()) != item['sha256']:
                raise ValueError('Approved UI source does not match its manifest: ' + relative)
            expected_ui_sources[relative] = item['sha256']
        build = update.get('build')
        if build is not None:
            if build.get('passed') is not True or set(build.get('source_sha256', {})) != set(reviewed['firmware_after']):
                raise ValueError('Firmware build does not cover all baseline inputs')
            if any(expected_sources.get(relative) != expected for relative, expected in build['source_sha256'].items()):
                raise ValueError('Firmware build sources differ from approved source updates')
            executable = Path(build['elf']['path']).resolve(strict=True)
            if not executable.is_relative_to(update_path.parent) or digest(executable.read_bytes()) != build['elf']['sha256']:
                raise ValueError('Approved firmware executable does not match its manifest')
            firmware_sha256 = build['elf']['sha256']
        if 'sequence_contract_revision' in update:
            if update['sequence_contract_revision'] != 1:
                raise ValueError('Unsupported sequence contract revision')
            sequence_revision = 1
        approved_updates.append({'manifest_sha256': digest(update_raw), 'changed_sources': changed_sources,
                                 'ui_sources': {relative: expected_ui_sources[relative]
                                                for relative in update.get('ui_files', {})}})
    for relative, expected in expected_sources.items():
        mirror = board / relative
        if digest(mirror.read_bytes()) != expected:
            raise ValueError('Board source differs from installed HOME5 manifest: ' + relative)
        path = approved_source_paths.get(relative)
        if path is not None and digest(path.read_bytes()) != expected:
            raise ValueError('Approved source snapshot differs from installed board mirror: ' + relative)
    for relative, expected in expected_pc_sources.items():
        if digest((source / relative).read_bytes()) != expected:
            raise ValueError('PC source differs from the approved update: ' + relative)
    discovery = ast.parse((source / 'tools/discover_unoq.py').read_text(encoding='utf-8-sig'))
    serial = next(ast.literal_eval(node.value) for node in discovery.body if isinstance(node, ast.Assign)
                  and any(isinstance(name, ast.Name) and name.id == 'CALIBRATION_SERIAL' for name in node.targets))
    selected = {relative: source / relative for relative in PC_FILES}
    selected.update({'assets/' + name: source / 'assets' / name for name in ASSETS})
    if sequence_revision is not None:
        selected.update({'assets/' + name: source / 'assets' / name for name in SEQUENCE_ASSETS})
    for path in (source / 'imu_transport').glob('*.py'):
        selected[path.relative_to(source).as_posix()] = path
    for path in (source / 'model-assets').glob('*.stl'):
        selected[path.relative_to(source).as_posix()] = path
    for name in ('model-data.json', 'model-data-mouth.json'):
        selected['model-assets/' + name] = source / 'model-assets' / name
    for path in (source / 'model-assets/vendor').rglob('*.js'):
        selected[path.relative_to(source).as_posix()] = path
    selected['board/app.yaml'] = board / 'app.yaml'
    selected['board/assets/index.html'] = board / 'assets/index.html'
    for folder in ('python', 'sketch', 'tools', 'firmware_dependencies'):
        for path in (board / folder).rglob('*'):
            relative = path.relative_to(board)
            if not path.is_file() or any(part in ('__pycache__', 'examples', 'extras', '.github', 'tests') for part in relative.parts):
                continue
            if folder == 'python' and path.suffix not in ('.py', '.txt'):
                continue
            selected['board/' + relative.as_posix()] = path
    for relative, path in approved_source_paths.items():
        selected['board/' + relative] = path
    for relative in UI_FILES.intersection(selected):
        expected = expected_ui_sources.get(relative)
        if expected is None:
            expected = published.get('files', {}).get(relative, {}).get('source_sha256')
        if expected is None or digest(selected[relative].read_bytes()) != expected:
            raise ValueError('UI source differs from its approved baseline: ' + relative)
    records = {}
    for relative, path in sorted(selected.items()):
        raw = path.read_bytes()
        exported, changes = portable(relative, raw, serial)
        destination = target / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(exported)
        records[relative] = {'source_sha256': digest(raw), 'sha256': digest(exported)}
        if changes:
            records[relative]['publish_changes'] = changes
    shutil.copyfile(target / 'device_config.py', target / 'board/python/device_config.py')
    dependencies = {relative.removeprefix('board/'): {'sha256': item['sha256']}
                    for relative, item in records.items() if relative.startswith('board/firmware_dependencies/')}
    (target / 'board/component-origin.json').write_text(json.dumps(dependencies, indent=2) + '\n', encoding='utf-8')
    manifest = {'base': 'installed HOME ABI5', 'home_contract_revision': 5,
                'installed_firmware_sha256': firmware_sha256,
                'sequence_contract_revision': sequence_revision,
                'verified_firmware_source_count': len(reviewed['firmware_after']),
                'verified_board_source_count': len(expected_sources), 'approved_updates': approved_updates,
                'device_data_included': False, 'physical_acceptance': 'pending', 'files': records}
    (target / 'SOURCE_MANIFEST.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'copied_files': len(records), 'bytes': sum((target / name).stat().st_size for name in records)}))


if __name__ == '__main__':
    main()
