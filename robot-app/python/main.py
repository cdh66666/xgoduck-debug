from release_profile import constrain_command, validate_release_action
import threading
import sys
import hmac
import asyncio
import json
import time
from pathlib import Path
# Use this app's already verified private runtime without changing global packages.
vendor = Path(__file__).resolve().parents[1] / 'model_vendor'
if vendor.is_dir() and not vendor.is_symlink():
    sys.path.insert(0, str(vendor))
from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse, StreamingResponse, FileResponse
from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from controller import Controller
from motion_recording import MotionRecorder

# Bound GIL handoff latency between the 100 Hz receiver, policy and WebUI.
sys.setswitchinterval(0.001)
controller = Controller(Bridge)
motion_recorder = MotionRecorder(controller.status)
ui = WebUI(port=9527)
# Preserve wireless API authentication while the official page is proxied over SSH.
# Docker forwards the host port to this container's interface, not its loopback.
wireless_key = (Path(__file__).resolve().parents[1] / 'data' / 'calibration_key').read_text().strip()
if not wireless_key:
    raise RuntimeError('wireless API key is missing')

# The owner put the duck back in its tray for stationary debugging.
# This local file is removed only after explicit renewed motion authorization.
motion_inhibit_file = Path(__file__).resolve().parents[1] / 'data' / 'motion-inhibit'


def require_motion_permission():
    if motion_inhibit_file.exists():
        raise HTTPException(423, '托盘内静止排查：动作已锁定，仅允许卸力与读取')


def status():
    value = controller.status()
    value['motion_inhibited'] = motion_inhibit_file.exists()
    return value

@ui.app.middleware('http')
async def wireless_access(request, call_next):
    if request.method == 'POST' and request.url.path.startswith('/api/'):
        if not hmac.compare_digest(request.headers.get('X-Cal-Key', ''), wireless_key):
            return JSONResponse({'detail':'Use the authenticated wireless control page'}, status_code=403)
    return await call_next(request)
Bridge.provide('duck_state', controller.receive)
Bridge.provide('duck_display',controller.receive_display)
Bridge.provide('duck_boot_pd', controller.receive_boot_pd)
Bridge.provide('duck_raw', controller.receive_raw)
Bridge.provide('duck_servo_reply', controller.receive_servo_reply)
Bridge.provide('duck_factory_reply', controller.receive_factory_reply)

def mode(body: dict):
    if body.get('mode') != 'off':
        require_motion_permission()
    try:
        result = controller.set_mode(body['mode'])
        motion_recorder.mark_event('mode', body)
        return result
    except (KeyError, ValueError) as exc: raise HTTPException(400, str(exc))

def command(body: dict):
    require_motion_permission()
    try: return controller.command(**constrain_command(body))
    except (TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def filtering(body: dict):
    try:
        leg, head = controller.set_action_alphas(**body)
        return {'leg':leg,'head':head}
    except (TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def pick(_body: dict = None):
    require_motion_permission()
    try: validate_release_action('pick'); return controller.start_pick()
    except ValueError as exc: raise HTTPException(400, str(exc))

def roulade(_body: dict = None):
    require_motion_permission()
    try: validate_release_action('roulade'); return controller.start_roulade()
    except ValueError as exc: raise HTTPException(400, str(exc))

def cal_enter(_body: dict = None):
    require_motion_permission()
    try: return controller.factory_action('enter')
    except ValueError as exc: raise HTTPException(400, str(exc))

def cal_exit(_body: dict = None):
    try: return controller.factory_action('exit')
    except ValueError as exc: raise HTTPException(400, str(exc))

def cal_targets(body: dict):
    require_motion_permission()
    try: return controller.cal_targets_set(body['targets'])
    except (KeyError, TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def cal_finish(body: dict):
    require_motion_permission()
    try:
        from rl_core import MOTOR_IDS
        index=int(body['index'])
        if not 0<=index<len(MOTOR_IDS): raise ValueError('invalid axis index')
        return controller.factory_action('calibrate',int(MOTOR_IDS[index]))
    except (KeyError, TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def servo_enter(_body: dict = None):
    require_motion_permission()
    try: return controller.set_mode('servo_debug')
    except ValueError as exc: raise HTTPException(400, str(exc))

def servo_exit(_body: dict = None):
    try: return controller.set_mode('shadow')
    except ValueError as exc: raise HTTPException(400, str(exc))

def servo_action(body: dict):
    require_motion_permission()
    try:
        return controller.servo_op(
            body['action'], body['target_id'],
            new_id=body.get('new_id'), kp=body.get('kp', 5), kd=body.get('kd', 20),
            raw_pos=body.get('raw_pos', 2047))
    except (KeyError, TypeError, ValueError) as exc:
        raise HTTPException(400, str(exc))


def factory_action(body: dict):
    if body.get('action') not in ('off', 'release', 'read', 'capabilities'):
        require_motion_permission()
    try:
        result = controller.factory_action(body['action'],body.get('target_id',10))
        motion_recorder.mark_event('factory', body)
        return result
    except (KeyError,TypeError,ValueError,OSError) as exc: raise HTTPException(400,str(exc))

ui.expose_api('POST','/api/factory',factory_action)
ui.expose_api('GET','/api/capabilities',controller.factory_status)
ui.expose_api('GET','/api/factory/state',controller.factory_status)


@ui.app.get('/api/status-stream')
async def status_stream(request: Request):
    """Send official snapshots continuously; no commands or control heartbeat."""
    async def snapshots():
        generation = 0
        while not await request.is_disconnected():
            previous = generation
            generation, state = await asyncio.to_thread(motion_recorder.wait_latest, generation)
            if state is None or generation == previous:
                continue
            state['motion_inhibited']=motion_inhibit_file.exists()
            state['telemetry_sent_unix_s']=time.time()
            yield (json.dumps(state,separators=(',',':'),ensure_ascii=False)+'\n').encode('utf-8')
    return StreamingResponse(snapshots(),media_type='application/x-ndjson',
                             headers={'Cache-Control':'no-store'})


@ui.app.get('/api/motion/sessions')
def motion_sessions():
    return {'items': motion_recorder.sessions(), 'recorder': motion_recorder.snapshot()}


@ui.app.get('/api/motion/sessions/{sid}')
def motion_session(sid: str):
    try:
        return FileResponse(motion_recorder.session_path(sid), media_type='application/gzip',
                            headers={'Cache-Control': 'no-store'})
    except (ValueError, FileNotFoundError) as exc:
        raise HTTPException(404, str(exc))


ui.expose_api('GET', '/api/status', status)
ui.expose_api('POST', '/api/mode', mode)
ui.expose_api('POST', '/api/command', command)
ui.expose_api('POST', '/api/filter', filtering)
ui.expose_api('POST', '/api/pick', pick)
ui.expose_api('POST', '/api/roulade', roulade)
ui.expose_api('POST', '/api/cal/enter', cal_enter)
ui.expose_api('POST', '/api/cal/exit', cal_exit)
ui.expose_api('POST', '/api/cal/targets', cal_targets)
ui.expose_api('POST', '/api/cal/finish', cal_finish)
ui.expose_api('POST', '/api/servo/enter', servo_enter)
ui.expose_api('POST', '/api/servo/exit', servo_exit)
ui.expose_api('POST', '/api/servo', servo_action)
worker = threading.Thread(target=controller.run, name='policy50', daemon=True)
worker.start()
try:
    App.run()
finally:
    controller.stop()
    worker.join(timeout=2)
    motion_recorder.close()
