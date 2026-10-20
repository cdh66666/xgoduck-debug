"""Persistent local model catalog. Importing/listing never commands the robot."""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile
import threading
import uuid

from model_auditor import audit_model, MAX_MODEL_BYTES

MAX_PREVIEW_BYTES = 64 * 1024 * 1024
_ID = re.compile(r'^[a-z0-9][a-z0-9-]{0,79}$')


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _preview_extension(filename, content):
    if not isinstance(content, bytes) or not 0 < len(content) <= MAX_PREVIEW_BYTES:
        raise ValueError('训练成果文件必须小于等于 64 MB')
    extension = Path(filename).suffix.lower()
    signatures = {
        '.gif': content.startswith((b'GIF87a', b'GIF89a')),
        '.webp': content[:4] == b'RIFF' and content[8:12] == b'WEBP',
        '.png': content.startswith(b'\x89PNG\r\n\x1a\n'),
        '.jpg': content.startswith(b'\xff\xd8\xff'),
        '.jpeg': content.startswith(b'\xff\xd8\xff'),
        '.mp4': len(content) >= 16 and content[4:8] == b'ftyp',
        '.webm': content.startswith(b'\x1aE\xdf\xa3'),
    }
    if not signatures.get(extension):
        raise ValueError('训练成果文件格式与内容不匹配；支持 GIF、WebP、PNG、JPEG、MP4、WebM')
    return extension


class ModelLibrary:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.directory = self.root / 'model-library'
        self.directory.mkdir(parents=True, exist_ok=True)
        if self.directory.resolve() != self.directory:
            raise ValueError('模型库目录不能重定向到其他路径')
        self._lock = threading.RLock()
        self._epoch = 0
        self._deploying = False
        self._bridge = None

    def _folder(self, model_id):
        if not isinstance(model_id, str) or not _ID.fullmatch(model_id):
            raise ValueError('模型 ID 无效')
        folder = (self.directory / model_id).resolve()
        if folder.parent != self.directory.resolve():
            raise ValueError('模型路径超出模型库')
        return folder

    def _manifest(self, model_id):
        folder = self._folder(model_id)
        manifest = _read(folder / 'manifest.json')
        model_path = (folder / 'model.onnx').resolve()
        if manifest.get('id') != model_id or model_path.parent != folder or not model_path.is_file():
            raise ValueError('模型文件不完整')
        return manifest

    def _public(self, manifest):
        preview = manifest.get('preview')
        return {key: manifest.get(key) for key in ('id', 'name', 'description', 'kind', 'sha256',
                'created_at', 'training', 'compatibility')} | {
                'preview_url': f"/assets/model-training.html?model={manifest['id']}" if manifest.get('builtin') else
                               (f"/api/models/{manifest['id']}/preview" if preview else None),
                'preview_kind': 'iframe' if manifest.get('builtin') else ('video' if preview and preview.endswith(('.mp4', '.webm')) else 'image' if preview else None)}

    def _runtime_bridge(self):
        with self._lock:
            if self._bridge is None:
                path = self.root / 'model_bridge.py'
                spec = importlib.util.spec_from_file_location('xgo_model_runtime_pc_bridge', path)
                if not spec or not spec.loader:
                    raise ValueError('模型运行服务尚未准备好')
                module = importlib.util.module_from_spec(spec)
                sys.modules[spec.name] = module
                spec.loader.exec_module(module)
                self._bridge = module
            return self._bridge

    def list_models(self):
        with self._lock:
            models = []
            for path in sorted(self.directory.glob('*/manifest.json')):
                try:
                    models.append(self._public(self._manifest(path.parent.name)))
                except (ValueError, OSError, KeyError):
                    continue
        try:
            runtime = self._runtime_bridge().runtime_status(self.root)
        except Exception as error:
            runtime = {'state': 'error', 'message': f'模型运行服务不可用：{error}'}
        return {'models': models, 'runtime': runtime}

    def get_model(self, model_id):
        with self._lock:
            return self._public(self._manifest(model_id))

    def media_path(self, model_id):
        with self._lock:
            manifest = self._manifest(model_id)
            preview = manifest.get('preview')
            if not preview:
                return None
            path = (self._folder(model_id) / preview).resolve()
            if path.parent != self._folder(model_id) or not path.is_file():
                raise ValueError('模型预览路径无效')
            return path

    def import_model(self, fields, files):
        model = files.get('model') or {}
        content = model.get('data')
        if not isinstance(content, bytes) or not 0 < len(content) <= MAX_MODEL_BYTES:
            raise ValueError('ONNX 文件必须小于等于 32 MB')
        if Path(str(model.get('filename', ''))).suffix.lower() != '.onnx':
            raise ValueError('请选择 ONNX 模型文件')
        name = str(fields.get('name', '')).strip()
        description = str(fields.get('description', '')).strip()
        if not name or len(name) > 100 or len(description) > 1000:
            raise ValueError('模型名称需为 1 至 100 字，说明最多 1000 字')
        preview = files.get('preview')
        extension = _preview_extension(str(preview.get('filename', '')), preview.get('data')) if preview else None
        with tempfile.TemporaryDirectory(prefix='.import-', dir=self.directory) as temporary:
            stage = Path(temporary)
            model_path = stage / 'model.onnx'
            model_path.write_bytes(content)
            audit = audit_model(model_path)
            model_id = 'model-' + audit['sha256'][:16] + '-' + uuid.uuid4().hex[:8]
            manifest = {'id': model_id, 'name': name, 'description': description, 'kind': 'onnx',
                        'sha256': audit['sha256'], 'created_at': datetime.now(timezone.utc).isoformat(),
                        'training': {}, 'compatibility': {'ready': True, 'reason': '14 关节策略格式及 CPU 推理审计通过；实机表现待验证。'},
                        'audit': audit, 'builtin': False, 'preview': ('preview' + extension) if extension else None}
            if preview:
                (stage / manifest['preview']).write_bytes(preview['data'])
            _atomic_json(stage / 'manifest.json', manifest)
            with self._lock:
                destination = self._folder(model_id)
                if destination.exists():
                    raise ValueError('模型 ID 冲突，请重试')
                stage.rename(destination)
        return {'ok': True, 'model': self._public(manifest)}

    def deploy(self, model_id):
        with self._lock:
            manifest = self._manifest(model_id)
            if not manifest.get('compatibility', {}).get('ready'):
                raise ValueError(manifest.get('compatibility', {}).get('reason', '模型不兼容'))
            path = self._folder(model_id) / 'model.onnx'
            if _sha(path) != manifest['sha256']:
                raise ValueError('模型文件校验失败，请重新导入')
            if self._deploying:
                raise ValueError('已有模型正在部署，请等待或停止')
            self._deploying = True
            epoch = self._epoch
            metadata = dict(manifest['audit']) | {key: manifest[key] for key in ('id', 'name', 'sha256')}
            metadata['audit'] = manifest['audit']
        try:
            bridge = self._runtime_bridge()
            result = bridge.deploy_model(self.root, path, metadata)
            with self._lock:
                cancelled = epoch != self._epoch
            if cancelled:
                stopped = bridge.stop_model(self.root)
                return {'ok': False, 'message': '部署已被停止操作取消', 'runtime': stopped.get('runtime', stopped)}
            return result
        finally:
            with self._lock:
                self._deploying = False

    def stop(self):
        with self._lock:
            self._epoch += 1
        return self._runtime_bridge().stop_model(self.root)

    def _builtins(self):
        base = self.root.parent
        runs = base / 'training' / 'runs'
        weighed = runs / 'r5-weighed-825p5g-20261017'
        return [
            ('official-getup', '官方起身', base / 'references/xgoduck_runtime_arduino/python/xgoduck_getup.onnx',
             'c414f2bd84b0a79b89f0ba6997135eb45e4ecd9df9271ea989c237864027b2a3',
             weighed / 'getup-final-evaluation/teacher-prone-seed20261101', 'getup', None),
        ]

    def _verified_replay(self, definition):
        model_id, name, source, expected, prefix, task, iterations = definition
        if _sha(source) != expected:
            raise ValueError(f'{name}权重校验失败')
        metrics_path, trajectory_path = Path(str(prefix) + '.json'), Path(str(prefix) + '-trajectory.json')
        metrics, trajectory = _read(metrics_path), _read(trajectory_path)
        verification_path = self.root.parent / 'training/runs/r5-weighed-825p5g-20261017/policy-hash-verification.json'
        verification = _read(verification_path)
        hashes = {entry['sha256'] for entry in verification['artifacts']}
        source_sha = metrics.get('source_sha256') or metrics.get('checkpoint_sha256')
        if verification.get('result') != 'PASS' or expected not in hashes or source_sha not in hashes:
            raise ValueError(f'{name}训练证据的权重来源不匹配')
        recorded = metrics.get('source') or metrics.get('checkpoint')
        normalized = str(recorded or '').replace('/mnt/d/', 'D:/')
        source_checkpoint = Path(normalized)
        if not source_checkpoint.is_file() or _sha(source_checkpoint) != source_sha:
            raise ValueError(f'{name}训练来源文件校验失败')
        if task == 'standing' and source_checkpoint.parent != source.parent:
            raise ValueError('站立训练 checkpoint 与 ONNX 不属于同一导出目录')
        if model_id == 'four-direction-getup' and source_checkpoint.parent != source.parent:
            raise ValueError('起身训练 checkpoint 与 ONNX 不属于同一导出目录')
        if model_id == 'official-getup' and source_sha != expected:
            raise ValueError('官方模型训练回放来源不匹配')
        if trajectory.get('source_sha256') != source_sha or trajectory.get('mass_kg') != 0.8255:
            raise ValueError('训练轨迹的权重来源或质量条件不匹配')
        if not isinstance(trajectory.get('frames'), list) or not trajectory['frames']:
            raise ValueError('训练轨迹为空')
        return {'trajectory_path': str(trajectory_path), 'metrics_path': str(metrics_path),
                'source_sha256': source_sha, 'model_sha256': expected, 'label': name,
                'evidence_kind': 'simulation', 'metrics': metrics,
                'trajectory_sha256': _sha(trajectory_path), 'metrics_sha256': _sha(metrics_path),
                'verification_sha256': _sha(verification_path)}

    def replay_info(self, model_id):
        with self._lock:
            manifest = self._manifest(model_id)
            if not manifest.get('builtin'):
                return {}
        definition = next((item for item in self._builtins() if item[0] == model_id), None)
        if definition is None:
            raise ValueError('没有对应的固定训练回放')
        result = self._verified_replay(definition)
        for key in ('trajectory_sha256', 'metrics_sha256', 'verification_sha256'):
            if manifest.get('evidence', {}).get(key) != result[key]:
                raise ValueError('已保存的训练证据发生变化，请重新核对来源')
        result.pop('metrics')
        return result

    def seed_builtin_models(self):
        results = []
        for definition in self._builtins():
            model_id, name, source, expected, prefix, task, iterations = definition
            evidence = self._verified_replay(definition)
            binding = {key: evidence[key] for key in ('trajectory_sha256', 'metrics_sha256', 'verification_sha256', 'source_sha256')}
            with self._lock:
                destination = self._folder(model_id)
                if (destination / 'manifest.json').is_file():
                    existing = self._manifest(model_id)
                    if existing['sha256'] != expected or _sha(destination / 'model.onnx') != expected:
                        raise ValueError(f'{name}已保存文件的校验失败')
                    if iterations is not None:
                        existing['training']['iterations'] = iterations
                    existing['training']['environment'] = '0.8255 kg 复验仿真'
                    existing['evidence'] = binding
                    _atomic_json(destination / 'manifest.json', existing)
                    results.append(self._public(existing))
                    continue
            audit = audit_model(source)
            metrics = evidence['metrics']
            successes = metrics.get('stable') if task == 'standing' else metrics.get('recovered_held_2s')
            training = {'task': '仿真站立' if task == 'standing' else '仿真起身',
                        'environment': '0.8255 kg 复验仿真',
                        'version': expected[:12], 'seed': metrics.get('seed'),
                        'episodes': metrics.get('episodes'), 'simulation_successes': successes}
            if iterations is not None:
                training['iterations'] = iterations
            manifest = {'id': model_id, 'name': name, 'description': f"已训练权重，可在下方实时交互仿真，也可部署到小鸭子本地运行。",
                        'kind': 'onnx', 'sha256': expected, 'created_at': datetime.now(timezone.utc).isoformat(),
                        'training': training, 'compatibility': {'ready': True, 'reason': '策略格式及 CPU 推理审计通过；实机运行须通过部署检查。'},
                        'audit': audit, 'builtin': True, 'preview': None, 'evidence': binding}
            with tempfile.TemporaryDirectory(prefix='.seed-', dir=self.directory) as temporary:
                stage = Path(temporary)
                shutil.copyfile(source, stage / 'model.onnx')
                if _sha(stage / 'model.onnx') != expected:
                    raise ValueError('复制期间模型文件发生变化')
                _atomic_json(stage / 'manifest.json', manifest)
                with self._lock:
                    if not destination.exists():
                        stage.rename(destination)
            results.append(self._public(manifest))
        return {'ok': True, 'models': results}
