"""Read-only diagnostics from a user-selected Uno Q on the local network."""
import ipaddress, json, re, socket, time
from urllib.parse import urlsplit
from urllib.request import Request, build_opener, HTTPRedirectHandler

class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None

PROBE_CATEGORIES = frozenset(('not_received','collecting','tx_incomplete','no_rx',
    'echo_only','rx_unparsed','device_error','individual_read_only','sync_read_only',
    'read_ok','ping_only','replying'))
PROBE_COUNTS = ('tx_count','rx_count','drained_count','valid_frames','crc_errors',
    'device_error_frames','known_frames','unknown_frames','echoes',
    'unexpected_length_frames','valid_read_frames','known_id_mask')
PROBE_TOTALS = ('tx_count','rx_count','valid_frames','crc_errors',
    'device_error_frames','echoes','unknown_frames')

def _bounded_integer(value, maximum):
    return value if type(value) is int and 0 <= value <= maximum else None

def _bounded_text(value, maximum):
    return value[:maximum] if isinstance(value, str) else None

def sanitize_servo_probe(value):
    """Allowlist startup diagnostics; never infer current motor readiness."""
    if not isinstance(value, dict) or value.get('supported') is not True or value.get('scope') != 'startup_id10':
        return None
    result = {'supported': True, 'scope': 'startup_id10', 'expected_reports': 9,
              'startup_only': True, 'repeats_require_mcu_restart': True}
    reports, seen = [], set()
    source = value.get('reports')
    for report in source[:9] if isinstance(source, list) else []:
        if not isinstance(report, dict):
            continue
        opcode, attempt = report.get('opcode'), report.get('attempt')
        if type(opcode) is not int or opcode not in (1, 2, 130) or type(attempt) is not int or not 1 <= attempt <= 3 or report.get('target_id') != 10 or (opcode, attempt) in seen:
            continue
        seen.add((opcode, attempt))
        out = {'opcode': opcode, 'target_id': 10, 'attempt': attempt}
        for name in PROBE_COUNTS:
            number = _bounded_integer(report.get(name), 32767 if name == 'known_id_mask' else 65535)
            if number is not None:
                out[name] = number
        for name in ('address','data_length','tx_requested','last_frame_id','last_status','last_reply_length','last_unknown_id'):
            number = _bounded_integer(report.get(name), 255)
            if number is not None:
                out[name] = number
        for name in ('flags', 'version'):
            number = _bounded_integer(report.get(name), 15 if name == 'flags' else 1)
            if number is not None:
                out[name] = number
        for name in ('first_rx_us_after_tx_observed','last_rx_us_after_tx_observed','window_us','elapsed_us'):
            number = _bounded_integer(report.get(name), 4294967295)
            if number is not None:
                out[name] = number
        age = report.get('received_age_ms')
        if type(age) in (int, float) and 0 <= age <= 31536000000:
            out['received_age_ms'] = age
        prefix = report.get('raw_prefix_hex')
        if isinstance(prefix, str) and len(prefix) <= 95:
            import re
            if re.fullmatch(r'(?:[0-9A-Fa-f]{2}(?: [0-9A-Fa-f]{2}){0,31})?', prefix):
                out['raw_prefix_hex'] = prefix.upper()
        out['position_valid'] = (report.get('position_valid') is True and opcode != 1
            and out.get('valid_read_frames', 0) > 0 and bool(out.get('known_id_mask', 0) & 1)
            and not (out.get('flags', 0) & 10)
            and _bounded_integer(report.get('raw_position'), 65535) is not None)
        out['raw_position'] = report.get('raw_position') if out['position_valid'] else None
        out['raw_speed'] = _bounded_integer(report.get('raw_speed'), 65535) if out['position_valid'] else None
        ids = report.get('known_reply_ids')
        if isinstance(ids, list):
            configured = tuple(range(10,15)) + tuple(range(20,25)) + tuple(range(30,35))
            out['known_reply_ids'] = [sid for sid in configured if sid in ids[:15]]
        reports.append(out)
    result['reports'] = reports
    result['received_reports'] = len(reports)
    result['complete'] = len(reports) == 9 and value.get('complete') is True
    summary = value.get('summary')
    if isinstance(summary, dict) and summary.get('category') in PROBE_CATEGORIES:
        result['summary'] = {'category': summary['category'], 'title': _bounded_text(summary.get('title'), 120), 'detail': _bounded_text(summary.get('detail'), 700)}
    totals = value.get('totals')
    result['totals'] = {name: number for name in PROBE_TOTALS if isinstance(totals, dict) and (number := _bounded_integer(totals.get(name), 589815)) is not None}
    for name in ('bad_frames','duplicates'):
        number = _bounded_integer(value.get(name), 4294967295)
        if number is not None:
            result[name] = number
    evidence = value.get('firmware_evidence', value.get('probe_firmware_evidence'))
    if evidence in ('received_reports','not_received'):
        result['firmware_evidence'] = evidence
    return result

def board_status(value):
    value = str(value).strip()
    if '://' not in value:
        value = 'http://' + value
    u = urlsplit(value)
    if u.scheme != 'http' or not u.hostname or u.username or u.password:
        raise ValueError('请输入 Uno Q 的本地 http 地址，例如 http://192.168.1.50:9527')
    if u.port not in (None, 9527) or u.path not in ('', '/', '/api/status') or u.query or u.fragment:
        raise ValueError('只允许读取 Uno Q 的 9527 端口 /api/status')
    addresses = socket.getaddrinfo(u.hostname, 9527, type=socket.SOCK_STREAM)
    if not addresses:
        raise ValueError('无法解析主控地址')
    for a in addresses:
        ip = ipaddress.ip_address(a[4][0])
        if not (ip.is_private or ip.is_loopback or ip.is_link_local):
            raise ValueError('仅支持本地网络主控，不能读取公网地址')
    # Use validated numeric address. Do not follow redirects or environment proxies.
    ip = addresses[0][4][0]
    host = '['+ip+']' if ':' in ip else ip
    from urllib.request import ProxyHandler
    url = 'http://'+host+':9527/api/status'
    req = Request(url, headers={'Accept':'application/json'})
    with build_opener(ProxyHandler({}), NoRedirect()).open(req, timeout=3) as response:
        raw = response.read(128*1024+1)
    if len(raw)>128*1024:
        raise ValueError('状态响应过大')
    d = json.loads(raw)
    if not isinstance(d,dict) or 'received' not in d or 'mode' not in d:
        raise ValueError('响应不是 XGO Duck 主控状态')
    return {'status': sanitize_status(d), 'read_only':True, 'checked_at':time.time()}


def sanitize_status(d):
    keys = ('mode','commissioning_motion_locked','received','mcu_seq','feedback_hz','feedback_age_ms',
            'imu_ok','enabled','servo_ids','missing_joint_ids','acc','gyro','gravity','angles',
            'raw_pos','raw_mask','full_robot_ready','last_error','bridge_baud','mcu_watchdog',
            'policy_locked','calibration_supported','servo_power','battery','raw_age_ms',
            'host_read_only','no_rpc_writes','mcu_contract_revision','snapshot_version','sticky_supported',
            'mcu_sample_us','mcu_imu_us','mcu_restarts','mcu_state','torque_may_be_on','all_torque_off_verified',
            'session_active','torque_enable_ever_attempted')
    status = {k:d.get(k) for k in keys}
    status['app_code_version'] = d.get('app_code_version') if d.get('app_code_version') in (
        '2026-10-12-probe1', '2026-10-13-wireless-calibration-v1', '2026-10-13-wireless-calibration-v2', '2026-10-13-wireless-calibration-v3', '2026-10-13-wireless-calibration-v4', '2026-10-13-wireless-calibration-v5', '2026-10-13-wireless-calibration-v6', '2026-10-15-wireless-calibration-v7') else None
    instance = d.get('app_instance_id')
    status['app_instance_id'] = instance if isinstance(instance, str) and re.fullmatch(r'[0-9a-f]{32}', instance) else None
    calibration = d.get('calibration')
    calibration = calibration if isinstance(calibration, dict) else {}
    status['calibration_supported'] = (status['app_code_version'] in (
        '2026-10-13-wireless-calibration-v2', '2026-10-13-wireless-calibration-v3', '2026-10-13-wireless-calibration-v4', '2026-10-13-wireless-calibration-v5', '2026-10-13-wireless-calibration-v6', '2026-10-15-wireless-calibration-v7')
        and calibration.get('supported') is True
        and calibration.get('schema') == 'wireless-calibration-v2'
        and calibration.get('version') == status['app_code_version']
        and calibration.get('snapshot_version') == 3
        and calibration.get('mcu_contract_revision') == 3)
    status['calibration_fresh'] = calibration.get('fresh') is True
    status['calibration_age_ms'] = calibration.get('age_ms')
    status['servo_probe'] = sanitize_servo_probe(d.get('servo_probe'))
    # Both read-only transports carry the same strictly validated JP1 observation.
    # An invalid optional pose must not interrupt otherwise valid IMU telemetry.
    pose = d.get('native_pose_compact')
    if pose is not None:
        from imu_transport.pose_codec import decode_pose
        try:
            decode_pose(pose)
        except ValueError:
            pass
        else:
            status['native_pose_compact'] = pose
    return status
