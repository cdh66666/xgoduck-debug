import json
import os
from pathlib import Path
import re


def local_config():
    path = Path(__file__).resolve().parent / 'device.local.json'
    if not path.is_file():
        return {}
    if path.is_symlink() or path.stat().st_size > 4096:
        raise ValueError('设备配置文件无效')
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError('设备配置必须是 JSON 对象')
    return value


def configured_serial():
    serial = os.environ.get('XGODUCK_SERIAL', '').strip() or local_config().get('serial', '')
    if not isinstance(serial, str) or (serial and not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', serial)):
        raise ValueError('设备序列号格式不正确')
    return serial


def configured_turn_targets():
    targets = local_config().get('audited_turn_targets', {})
    if not isinstance(targets, dict) or not set(targets).issubset({'10', '22', '30'}):
        raise ValueError('跨圈参考配置无效')
    for pair in targets.values():
        if (not isinstance(pair, list) or len(pair) != 2
                or any(type(value) is not int or not -32767 <= value <= 32767 for value in pair)
                or abs(pair[0] - pair[1]) != 4096):
            raise ValueError('跨圈参考必须是已实测确认的相邻两圈编码值')
    return {key: tuple(pair) for key, pair in targets.items()}


DEVICE_SERIAL = configured_serial()
HOME_TURN_TARGETS = configured_turn_targets()
