"""Receive-only Uno Q commissioning host. No inference or actuator commands."""
from collections import deque
import asyncio
import json
import math
import threading
import time
import uuid
from wire import ALL_MASK, decode_raw, decode_servo_reply, decode_state, elapsed_us
from servo_probe import decode_report, summarize

MOTOR_IDS = (10,11,12,13,14,20,21,22,23,24,30,31,32,33,34)
FRESH_LIMIT_S = 0.15
MOTION_LOCK_FLAG = 0x20
APP_CODE_VERSION = "2026-10-12-probe1"
LIVE_PERIOD_S = .01


async def live_events(status_provider, request, stop_event=None):
    """100 Hz ceiling, latest received DQS only; silence on duplicate/stale data.

    The provider reads in-memory caches. This stream has no Bridge, credential,
    disk I/O, actuator authorization or synthesized telemetry frame.
    """
    previous = None
    next_frame = time.monotonic()
    while (stop_event is None or not stop_event.is_set()) and not await request.is_disconnected():
        status = status_provider()
        identity = (status.get('app_instance_id'), status.get('mcu_restarts'),
                    status.get('mcu_seq'), status.get('mcu_sample_us'))
        if status.get('feedback_fresh') is True and identity[2] is not None and identity[3] is not None and identity != previous:
            payload = json.dumps(status, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
            event_id = ':'.join(str(value) for value in identity)
            yield ('id: '+event_id+'\nevent: status\ndata: '+payload+'\n\n').encode('utf-8')
            previous = identity
        # Include serialization and transport time in the 10 ms budget.
        # If a client falls behind, skip missed slots instead of bursting.
        next_frame += LIVE_PERIOD_S
        now = time.monotonic()
        if next_frame <= now:
            next_frame = now + LIVE_PERIOD_S
        await asyncio.sleep(next_frame - now)


def live_response(status_provider, request, stop_event=None):
    from fastapi.responses import StreamingResponse
    return StreamingResponse(live_events(status_provider, request, stop_event),
                             media_type='text/event-stream',
                             headers={'Cache-Control': 'no-store', 'X-Accel-Buffering': 'no'})

class ReadOnlyTelemetry:
    """Cache MCU reports; deliberately has no Bridge reference."""
    def __init__(self, clock=time.monotonic):
        self.clock, self.started = clock, clock()
        self.app_instance_id = uuid.uuid4().hex
        self.lock = threading.RLock()
        self.feedback = self.rx_time = self.raw = self.raw_time = self.servo_reply = None
        self.rx_times, self.errors = deque(maxlen=1000), deque(maxlen=20)
        self.received = self.bad_frames = self.gaps = self.restarts = 0
        self.mcu_enabled_seen = False
        self.boot_pd = {}
        self.probe_reports = {}
        self.probe_bad_frames = self.probe_duplicates = 0

    def _bad(self, channel, error):
        with self.lock:
            self.bad_frames += 1
            self.errors.append({"channel":channel, "error":str(error),
                                "host_uptime_s":round(self.clock()-self.started,3)})

    def receive(self, raw):
        try:
            fb = decode_state(raw)
        except (ValueError, TypeError) as error:
            self._bad("duck_state", error)
            return
        now = self.clock()
        with self.lock:
            if self.feedback:
                step = elapsed_us(fb.seq, self.feedback.seq)
                if step == 0:
                    return  # Duplicates must not make old reports look fresh.
                if step > 0x80000000:
                    self.restarts += 1
                    self.rx_times.clear()
                    self.raw = self.raw_time = self.servo_reply = None
                    self.boot_pd = {}
                    self.probe_reports.clear()
                else:
                    self.gaps += step-1
            self.feedback, self.rx_time = fb, now
            self.received += 1
            self.rx_times.append(now)
            self.mcu_enabled_seen = self.mcu_enabled_seen or fb.enabled

    def receive_boot_pd(self, raw):
        if not isinstance(raw,(bytes,bytearray)) or len(raw)!=32:
            self._bad("duck_boot_pd","expected 32-byte PD report")
            return
        mask = int.from_bytes(raw[:2],"little")
        if mask & ~ALL_MASK:
            self._bad("duck_boot_pd","invalid servo mask")
            return
        with self.lock:
            self.boot_pd = {str(sid):{"p":raw[2+i],"d":raw[17+i]}
                            for i,sid in enumerate(MOTOR_IDS) if mask&(1<<i)}

    def receive_raw(self, raw):
        try:
            packet = decode_raw(raw)
        except (ValueError, TypeError) as error:
            self._bad("duck_raw",error)
            return
        with self.lock:
            self.raw, self.raw_time = packet, self.clock()

    def receive_servo_reply(self, raw):
        try:
            packet = decode_servo_reply(raw)
        except (ValueError, TypeError) as error:
            self._bad("duck_servo_reply",error)
            return
        with self.lock:
            self.servo_reply = packet

    def receive_probe(self, raw):
        try:
            packet = decode_report(raw)
        except (ValueError, TypeError) as error:
            with self.lock:
                self.probe_bad_frames += 1
            self._bad("duck_probe", error)
            return
        with self.lock:
            key = (packet["opcode"], packet["attempt"])
            if key in self.probe_reports:
                self.probe_duplicates += 1
                return
            self.probe_reports[key] = (packet, self.clock())

    def _probe_status(self, now):
        # Caller holds self.lock. These are historical startup reports; never
        # change DQS1 feedback, raw mask, servo readiness or sent counters.
        reports = [dict(packet, received_age_ms=round(max(0, now-stamp)*1000, 2))
                   for _, (packet, stamp) in sorted(self.probe_reports.items())]
        totals = {name: sum(r[name] for r in reports) for name in (
            "tx_count", "rx_count", "valid_frames", "crc_errors",
            "device_error_frames", "echoes", "unknown_frames")}
        return dict(supported=True, scope="startup_id10", expected_reports=9,
                    received_reports=len(reports), complete=len(reports)==9,
                    reports=reports, summary=summarize(reports), totals=totals,
                    bad_frames=self.probe_bad_frames, duplicates=self.probe_duplicates,
                    startup_only=True, repeats_require_mcu_restart=True,
                    firmware_evidence="received_reports" if reports else "not_received",
                    evidence_note="历史启动诊断，不能替代实时位置、全机通信、接线或机械验收。")

    def live_status(self):
        """Small real DQS snapshot; no probe history, disk access or RPC writes."""
        with self.lock:
            now = self.clock()
            fb = self.feedback
            age = now-self.rx_time if self.rx_time is not None else None
            fresh = age is not None and age < FRESH_LIMIT_S
            recent = [t for t in self.rx_times if now-t <= 2.0]
            hz = ((len(recent)-1)/(recent[-1]-recent[0])
                  if len(recent)>1 and recent[-1]>recent[0] else 0.0)
            mask = fb.mask if fb else 0
            locked = bool(fb.flags&MOTION_LOCK_FLAG) if fb else None
            verified_locked = bool(fresh and locked and fb and not fb.enabled
                                   and not fb.calibrating and not fb.servo_debugging)
            result = dict(mode='read_only', app_code_version=APP_CODE_VERSION,
                          app_instance_id=self.app_instance_id, host_read_only=True,
                          no_rpc_writes=True, control_ready=False, received=self.received,
                          feedback_hz=round(hz, 2),
                          feedback_age_ms=round(age*1000, 2) if age is not None else None,
                          feedback_fresh=fresh, mcu_restarts=self.restarts,
                          mcu_seq=None, mcu_sample_us=None, mcu_imu_us=None,
                          imu_ok=None, acc=None, gyro=None, enabled=None,
                          commissioning_motion_locked=verified_locked, mcu_motion_locked=locked,
                          servo_ids=[sid for i, sid in enumerate(MOTOR_IDS) if mask&(1<<i)],
                          missing_joint_ids=[sid for i, sid in enumerate(MOTOR_IDS) if not mask&(1<<i)],
                          full_robot_ready=bool(verified_locked and fb and fb.imu_ok and mask==ALL_MASK),
                          bridge_baud=2000000, sent=0)
            if fb:
                result.update(mcu_seq=fb.seq, mcu_sample_us=fb.sample_us, mcu_imu_us=fb.imu_us, mcu_work_us=fb.work_us,
                              imu_ok=fb.imu_ok, enabled=fb.enabled, mcu_flags=fb.flags,
                              mcu_watchdog=bool(fb.flags&4), acc=list(fb.acc), gyro=list(fb.gyro))
            return result

    def status(self):
        now = self.clock()
        with self.lock:
            fb = self.feedback
            age = now-self.rx_time if self.rx_time is not None else None
            fresh = age is not None and age < FRESH_LIMIT_S
            locked = bool(fb.flags&MOTION_LOCK_FLAG) if fb else None
            verified_locked = (bool(fresh and locked and not fb.enabled and not fb.calibrating
                                    and not fb.servo_debugging) if fb else None)
            recent = [t for t in self.rx_times if now-t<=2.0]
            hz = ((len(recent)-1)/(recent[-1]-recent[0])
                  if len(recent)>1 and recent[-1]>recent[0] else 0.0)
            mask = fb.mask if fb else 0
            error = ""
            if fb and fb.enabled:
                error = "MCU 报告 enabled=true：当前固件状态不符合只读调试要求，请保持舵机电源断开。"
            elif fb and (fb.calibrating or fb.servo_debugging):
                error = "MCU 报告校准/舵机调试模式：当前固件状态不符合只读调试要求。"
            elif not fb:
                error = "尚未收到 MCU 反馈；网页可访问不代表 MCU 正在运行。"
            elif not fresh:
                error = "MCU 反馈已过期，显示的是最后一次报告。"
            elif not locked:
                error = "MCU 未报告只读固件锁定标记(bit5)，不能确认当前运行的是只读调试固件。"
            elif self.errors:
                error = self.errors[-1]["channel"]+": "+self.errors[-1]["error"]
            result = dict(
                mode="read_only", host_read_only=True, no_rpc_writes=True, port=9527, vm="read-only-ui",
                app_code_version=APP_CODE_VERSION, app_instance_id=self.app_instance_id,
                servo_probe=self._probe_status(now),
                commissioning_motion_locked=verified_locked, mcu_motion_locked=locked,
                motion_lock_evidence="Fresh MCU bit5 plus enabled=false; firmware source reviewed separately. Not physical motion acceptance.",
                received=self.received, sent=0, inferred=0,
                feedback_hz=round(hz,2), feedback_age_ms=round(age*1000,2) if age is not None else None,
                feedback_fresh=fresh, fresh_limit_ms=FRESH_LIMIT_S*1000,
                uptime_s=round(now-self.started,3), host_unix_time_s=time.time(),
                clock_note="反馈年龄使用 Linux monotonic 收到时间；MCU sample_us/imu_us 是启动计时，不能与电脑或 Linux 墙上时间相减。",
                bad_frames=self.bad_frames, sequence_gaps=self.gaps, mcu_restarts=self.restarts,
                mcu_enabled_seen=self.mcu_enabled_seen,
                servo_ids=[sid for i,sid in enumerate(MOTOR_IDS) if mask&(1<<i)],
                missing_joint_ids=[sid for i,sid in enumerate(MOTOR_IDS) if not mask&(1<<i)],
                motors=list(MOTOR_IDS),
                full_robot_ready=bool(verified_locked and fb.imu_ok and mask==ALL_MASK),
                readiness_note="full_robot_ready 仅表示反馈完整且新鲜，不是机械零位、方向、供电或运动验收。",
                control_ready=False, last_error=error, errors=list(self.errors), boot_pd=dict(self.boot_pd),
                raw_pos=list(self.raw["pos"]) if self.raw else None,
                zero_pos=list(self.raw["zero"]) if self.raw else None,
                raw_mask=self.raw["mask"] if self.raw else 0,
                raw_age_ms=round((now-self.raw_time)*1000,2) if self.raw_time is not None else None,
                servo_reply=dict(self.servo_reply) if self.servo_reply else None, bridge_baud=2000000,
                imu_ok=None, enabled=None, mcu_watchdog=None, mcu_seq=None,
                acc=None, gyro=None, gravity=None, angles=None, velocities=None, targets=None,
                gravity_source="normalized_accelerometer_body_axes",
                gravity_note="gravity 只是加速度归一化方向，运动时不等于真实重力方向；未做陀螺融合或安装方向标定。",
                units={"acc":"m/s²","gyro":"deg/s","angles":"deg","velocities":"deg/s","targets":"deg","gravity":"unit vector"})
            if fb:
                norm = math.sqrt(sum(v*v for v in fb.acc))
                gravity = [v/norm for v in fb.acc] if fb.imu_ok and norm>1e-6 else None
                result.update(
                    imu_ok=fb.imu_ok, enabled=fb.enabled, mcu_watchdog=bool(fb.flags&4),
                    mcu_calibrating=fb.calibrating, mcu_servo_debug=fb.servo_debugging,
                    mcu_seq=fb.seq, mcu_flags=fb.flags, mcu_sample_us=fb.sample_us, mcu_imu_us=fb.imu_us,
                    mcu_period_us=fb.period_us, mcu_work_us=fb.work_us, mcu_overruns=fb.overruns,
                    mcu_report_drops=fb.report_drops, mcu_invalid_commands=fb.invalid_commands,
                    mcu_command_seq=fb.command_seq, mcu_command_age_us=fb.command_age_us,
                    angles=list(fb.angles), velocities=list(fb.velocities), targets=list(fb.targets),
                    acc=list(fb.acc), gyro=list(fb.gyro), gravity=gravity)
            return result

def main():
    from arduino.app_utils import App, Bridge
    from arduino.app_bricks.web_ui import WebUI
    telemetry = ReadOnlyTelemetry()
    ui = WebUI(port=9527)
    Bridge.provide("duck_state",telemetry.receive)
    Bridge.provide("duck_boot_pd",telemetry.receive_boot_pd)
    Bridge.provide("duck_raw",telemetry.receive_raw)
    Bridge.provide("duck_servo_reply",telemetry.receive_servo_reply)
    Bridge.provide("duck_probe",telemetry.receive_probe)
    ui.expose_api("GET","/api/status",telemetry.status)
    App.run()

if __name__ == "__main__":
    main()
