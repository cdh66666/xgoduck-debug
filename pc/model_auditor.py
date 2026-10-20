"""Bounded, CPU-only ONNX audit. The web server itself needs no ONNX packages."""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from functools import lru_cache

MAX_MODEL_BYTES = 32 * 1024 * 1024
JOINT_NAMES = ('left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee',
               'left_ankle', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
               'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle')
OBSERVATION_NAMES = ('base_ang_vel', 'projected_gravity', 'joint_pos', 'joint_vel',
                     'actions', 'command', 'head_command', 'body_command')


def _sequence(text):
    if str(text).strip().startswith('['):
        value = json.loads(text)
        if not isinstance(value, list):
            raise ValueError('模型关节元数据格式错误')
        return value
    return [item.strip() for item in str(text).split(',')]


def _audit_worker(path):
    import numpy as np
    import onnx
    import onnxruntime as ort
    path = Path(path)
    if not 0 < path.stat().st_size <= MAX_MODEL_BYTES:
        raise ValueError('ONNX 文件必须小于等于 32 MB')
    model = onnx.load_model(str(path), load_external_data=False)

    def inspect(message):
        if isinstance(message, onnx.TensorProto):
            if message.external_data or message.data_location == onnx.TensorProto.EXTERNAL:
                raise ValueError('模型必须包含全部权重，不支持外部数据文件')
        for field, value in message.ListFields():
            if field.type == field.TYPE_MESSAGE:
                if field.is_repeated:
                    for child in value:
                        inspect(child)
                else:
                    inspect(value)
    inspect(model)
    onnx.checker.check_model(model)
    metadata = {item.key: item.value for item in model.metadata_props}
    names = _sequence(metadata.get('joint_names', ''))
    if names != list(JOINT_NAMES):
        raise ValueError('模型 joint_names 必须匹配小鸭子的 14 个关节顺序')
    observations = _sequence(metadata.get('observation_names', ''))
    if observations != list(OBSERVATION_NAMES):
        raise ValueError('模型 observation_names 必须匹配小鸭子的观测顺序')
    defaults = [float(item) for item in _sequence(metadata.get('default_joint_pos', ''))]
    if len(defaults) != 14 or not all(math.isfinite(item) for item in defaults):
        raise ValueError('模型 default_joint_pos 必须是 14 个有限数值')
    scale = float(metadata.get('action_scale', 'nan'))
    if not math.isfinite(scale) or scale != 1.0:
        raise ValueError('模型 action_scale 必须为 1')
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
    session = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if len(inputs) != 1 or inputs[0].type != 'tensor(float)' or inputs[0].shape != [1, 61]:
        raise ValueError('模型输入必须是 float32 [1,61]')
    if len(outputs) != 1 or outputs[0].type != 'tensor(float)' or outputs[0].shape != [1, 14]:
        raise ValueError('模型输出必须是 float32 [1,14]')
    result = session.run(None, {inputs[0].name: np.zeros((1, 61), dtype=np.float32)})[0]
    if result.shape != (1, 14) or not np.isfinite(result).all():
        raise ValueError('模型零输入推理返回了无效动作')
    return {'ok': True, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'bytes': path.stat().st_size, 'joint_names': names, 'default_joint_pos': defaults,
            'action_scale': scale, 'observation_names': observations,
            'input_name': inputs[0].name, 'output_name': outputs[0].name,
            'input_shape': [1, 61], 'output_shape': [1, 14], 'provider': 'CPUExecutionProvider',
            'zero_input_finite': True, 'onnx_version': onnx.__version__, 'ort_version': ort.__version__}


def _subprocess_options():
    return {'creationflags': subprocess.CREATE_NO_WINDOW} if os.name == 'nt' else {}


@lru_cache(maxsize=1)
def audit_python():
    candidates = [Path(sys.executable)]
    for candidate in candidates:
        if not candidate.is_file():
            continue
        try:
            result = subprocess.run([str(candidate), '-I', '-c', 'import onnx,onnxruntime,numpy; print("ready")'],
                                    capture_output=True, timeout=8, **_subprocess_options())
            if result.returncode == 0 and result.stdout.strip() == b'ready':
                return str(candidate)
        except (OSError, subprocess.TimeoutExpired):
            pass
    raise ValueError('本机缺少可用的 ONNX 审计环境，未保存模型')


def audit_model(path, timeout=20):
    path = Path(path).resolve(strict=True)
    if not 0 < path.stat().st_size <= MAX_MODEL_BYTES:
        raise ValueError('ONNX 文件必须小于等于 32 MB')
    try:
        result = subprocess.run([audit_python(), '-I', str(Path(__file__).resolve()), '--worker', str(path)],
                                capture_output=True, timeout=min(float(timeout), 20),
                                **_subprocess_options())
    except subprocess.TimeoutExpired as error:
        raise ValueError('ONNX 审计超过 20 秒，未保存模型') from error
    try:
        report = json.loads(result.stdout.decode('utf-8'))
    except (ValueError, UnicodeError) as error:
        raise ValueError('ONNX 审计进程未能完成，未保存模型') from error
    if result.returncode or not report.get('ok'):
        raise ValueError(str(report.get('error', '模型审计失败'))[:1000])
    if report.get('sha256') != hashlib.sha256(path.read_bytes()).hexdigest():
        raise ValueError('审计期间模型文件发生变化')
    return report


if __name__ == '__main__':
    try:
        if len(sys.argv) != 3 or sys.argv[1] != '--worker':
            raise ValueError('仅用于模型审计子进程')
        print(json.dumps(_audit_worker(sys.argv[2]), ensure_ascii=True))
    except Exception as exc:
        print(json.dumps({'ok': False, 'error': str(exc)[:1000]}, ensure_ascii=True))
        sys.exit(1)
