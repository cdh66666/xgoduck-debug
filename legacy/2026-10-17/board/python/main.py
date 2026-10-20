"""Wireless calibration only; never starts policy or automatically pushes zeros."""
from pathlib import Path
import json
import threading
import time
from calibration_host import CalibrationError, VERSION, WirelessCalibrationHost, load_private_key
from telemetry import ReadOnlyTelemetry, live_response
from startup_serial_recovery import StartupSerialRecovery, StartupRecoveryBusy
from checked_bridge import CheckedBridge
from unix_command_rpc import UnixCommandRPC
from imu_transport.integration import start_host_if_enabled
from pose_observation import provider as pose_observation, attach_pose

APP_CODE_VERSION = VERSION
RELIABILITY_REVISION = '2026-10-15-home-slow-hold-v7'


def live_snapshot(host):
    started=time.monotonic()
    result=attach_pose(host,host.live_status())
    result['source_monotonic']=started
    result['checked_at']=time.time()
    return result


def load_imu_config(data_dir):
    path = Path(data_dir) / 'imu_transport.json'
    if not path.exists():
        return None
    if path.is_symlink() or path.stat().st_size > 4096:
        raise ValueError('Invalid IMU transport configuration')
    return json.loads(path.read_text('utf-8'))


def main():
    from arduino.app_utils import App, Bridge
    from arduino.app_bricks.web_ui import WebUI
    from fastapi import HTTPException, Request
    globals()['Request'] = Request
    data_dir = Path(__file__).resolve().parent.parent / 'data'
    telemetry = ReadOnlyTelemetry()
    key = load_private_key(data_dir)
    host = WirelessCalibrationHost(telemetry, CheckedBridge(UnixCommandRPC()), key, data_dir)
    startup_recovery = StartupSerialRecovery(host, telemetry, data_dir)
    imu_endpoint = None
    imu_error = None
    try:
        imu_endpoint = start_host_if_enabled(load_imu_config(data_dir), key, lambda: pose_observation(host,telemetry))
    except (OSError, ValueError):
        imu_error = 'IMU read-only transport configuration unavailable'
    ui = WebUI(port=9527)
    stop = threading.Event()
    Bridge.provide('duck_state', startup_recovery.wrap_feedback(telemetry.receive))
    Bridge.provide('duck_boot_pd', startup_recovery.wrap_feedback(telemetry.receive_boot_pd))
    Bridge.provide('duck_raw', startup_recovery.wrap_feedback(telemetry.receive_raw))
    Bridge.provide('duck_servo_reply', startup_recovery.wrap_feedback(telemetry.receive_servo_reply))
    Bridge.provide('duck_probe', startup_recovery.wrap_feedback(telemetry.receive_probe))
    Bridge.provide('duck_calibration_state', startup_recovery.wrap_feedback(host.receive))
    Bridge.provide('duck_calibration_result', startup_recovery.wrap_feedback(host.receive_result))
    Bridge.provide('duck_home_state', startup_recovery.wrap_feedback(host.receive_home))
    Bridge.provide('duck_sequence_state', startup_recovery.wrap_feedback(host.sequence_controller.receive))
    Bridge.provide('duck_servo_parameters', startup_recovery.wrap_feedback(host.sequence_controller.receive_parameters))
    Bridge.provide('duck_servo_current', startup_recovery.wrap_feedback(host.servo_current.receive))

    def action(body: dict, request: Request):
        try:
            startup_recovery.before_action()
            return host.action(body, request.headers.get('X-Cal-Key', ''))
        except StartupRecoveryBusy as error:
            raise HTTPException(503, {'code': 'startup_serial_recovery_busy', 'message': str(error), 'ids': []}) from None
        except CalibrationError as error:
            raise HTTPException(error.status, {'code': error.code, 'message': str(error), 'ids': error.ids}) from None

    async def live(request: Request):
        return live_response(lambda: live_snapshot(host), request, stop)

    def status():
        result = host.status()
        result['reliability_revision'] = RELIABILITY_REVISION
        result['startup_serial_recovery'] = startup_recovery.snapshot()
        result['imu_transport'] = {'enabled': imu_endpoint is not None, 'error': imu_error,
                                   'control_capability': False}
        return result
    ui.expose_api('GET', '/api/status', status)
    ui.expose_api('GET', '/api/calibration', host.calibration_status)
    ui.expose_api('GET', '/api/live', live)
    ui.expose_api('POST', '/api/calibration/action', action)
    def watchdog():
        while not stop.wait(.025):
            host.tick()
    worker = threading.Thread(target=watchdog, name='calibration-deadman', daemon=True)
    worker.start()
    recovery_worker = threading.Thread(target=startup_recovery.run, args=(stop,), name="startup-serial-recovery", daemon=True)
    recovery_worker.start()
    try:
        App.run()
    finally:
        stop.set(); startup_recovery.cancel()
        if imu_endpoint is not None: imu_endpoint.close()
        host.stop(); worker.join(timeout=1)
        recovery_worker.join(timeout=3.2)


if __name__ == '__main__':
    main()
