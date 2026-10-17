#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <math.h>
#include <string.h>
#include "duck_config.h"
#include "protocol.h"
#include "scs_bus.h"
#include "qmi8658.h"
#include "uart_fifo.h"
#include "servo_probe.h"
#include "calibration_engine.h"
#include "route_refresh.h"

Motor motors[NUM_MOTORS];
struct k_mutex motorsMu;
// App sketches are Zephyr loadable extensions: initialize kernel objects at runtime.
// Avoid iterable kernel-object sections in the loadable sketch.
static struct k_mutex commandMu, stateMu, probeMu, calibrationMu;
static struct k_sem reportReady;
K_THREAD_STACK_DEFINE(controlStack, 4096);
K_THREAD_STACK_DEFINE(reportStack, 4096);
static struct k_thread controlThread, reportThread;
K_THREAD_STACK_DEFINE(routeStack, 4096);
static struct k_thread routeThread;
static bool routeHandlersBound = false;
static void routeTask(void*, void*, void*) {
  struct Port {
    bool handlersBound() const { return routeHandlersBound; }
    bool registerOnly(const char* name, uint32_t deadline) { return Bridge.registerOnly(name, deadline); }
  } port;
  duck_routes::Refresh refresh;
  for (;;) { refresh.tick(k_uptime_get_32(), port); k_msleep(25); }
}
static StatePacket snapshot = {};
static CommandPacket latest = {};
static bool pending = false;
static uint32_t invalidCommands = 0, reportDrops = 0;
static duck_calibration::Engine calibrationEngine;
static duck_calibration::HomeEngine homeEngine;
static duck_calibration::HomeCommand homePending = {};
static bool homePendingReady=false;
static duck_calibration::HomeSnapshot homeSnapshot = {};
static duck_calibration::Command calibrationQueue[4] = {}, calibrationStop = {};
static uint8_t calibrationQueueRead=0, calibrationQueueCount=0;
static uint32_t invalidCalibrationCommands=0;
static bool calibrationStopPending=false;
static duck_calibration::Snapshot calibrationSnapshot = {};
static duck_calibration::ActionResult calibrationResult = {};
static bool calibrationResultReady=false;

// One fixed ID10 diagnostic sequence per MCU startup; no probe/gain/ID/legacy
// calibration RPC. The separate DWC1 interface permits only guarded single-ID
// calibration. Only controlTask calls UART engines; reportTask never touches UART.
struct DuckProbePort {
  int available() { return Serial1.available(); }
  int read() { return Serial1.read(); }
  size_t write(const uint8_t* data, size_t size) { return Serial1.write(data, size); }
  void flush() { Serial1.flush(); }
  uint32_t nowUs() { return micros(); }
};
static DuckProbePort startupProbePort;
static duck_probe::Engine startupProbe;
static duck_probe::Report completedProbe;
static_assert(SERVO_BAUD == duck_probe::kBaud, "Fixed probe baud must match Serial1");
constexpr uint8_t PROBE_REPORT_CAPACITY = 9;
constexpr uint32_t PROBE_REPLAY_INTERVAL_US = 1000000;
constexpr uint32_t PROBE_REPLAY_DURATION_US = 120000000;
static uint8_t probeReports[PROBE_REPORT_CAPACITY][duck_probe::kWireSize] = {};
static uint8_t probeReportsWritten = 0, probeReportsRead = 0;
static_assert(sizeof(probeReports) == 864, "Nine fixed DQP1 slots");

static void storeProbeReport(const duck_probe::Report& report) {
  k_mutex_lock(&probeMu, K_FOREVER);
  // Engine produces at most nine reports; startup-only storage never wraps.
  if (probeReportsWritten < PROBE_REPORT_CAPACITY) {
    duck_probe::encodeReport(report, probeReports[probeReportsWritten]);
    ++probeReportsWritten;
  }
  k_mutex_unlock(&probeMu);
}

static bool copyProbeReport(MsgPack::bin_t<uint8_t>& bytes) {
  k_mutex_lock(&probeMu, K_FOREVER);
  const bool ready = probeReportsRead < probeReportsWritten;
  if (ready) memcpy(bytes.data(), probeReports[probeReportsRead], duck_probe::kWireSize);
  k_mutex_unlock(&probeMu);
  return ready;
}

static void markProbeReportSent() {
  k_mutex_lock(&probeMu, K_FOREVER);
  if (probeReportsRead < probeReportsWritten) ++probeReportsRead;
  k_mutex_unlock(&probeMu);
}

static bool copyProbeReplay(uint8_t& next, MsgPack::bin_t<uint8_t>& bytes) {
  k_mutex_lock(&probeMu, K_FOREVER);
  const bool ready = probeReportsWritten != 0;
  if (ready) {
    if (next >= probeReportsWritten) next = 0;
    memcpy(bytes.data(), probeReports[next], duck_probe::kWireSize);
    ++next;
  }
  k_mutex_unlock(&probeMu);
  return ready;
}

static void receiveCommand(MsgPack::bin_t<uint8_t> bytes) {
  CommandPacket cmd = {};
  bool valid = bytes.size() == sizeof(cmd);
  if (valid) {
    memcpy(&cmd, bytes.data(), sizeof(cmd));
    valid = memcmp(cmd.magic, "DQC1", 4) == 0 && cmd.mode == DISARM &&
            !cmd.reserved && !cmd.reserved2 &&
            cmd.ttlUs > 0 && cmd.ttlUs <= COMMAND_TTL_US;
    for (unsigned i = 0; i < NUM_MOTORS; ++i)
      valid = valid && isfinite(cmd.target[i]);
  }
  k_mutex_lock(&commandMu, K_FOREVER);
  if (valid) { latest = cmd; pending = true; }
  else ++invalidCommands;
  k_mutex_unlock(&commandMu);
}

static void receiveCalibration(MsgPack::bin_t<uint8_t> bytes) {
  duck_calibration::Command command={};duck_calibration::HomeCommand home={};
  const bool isHome=bytes.size()==sizeof(home);
  bool valid=isHome||bytes.size()==sizeof(command);
  if(valid&&isHome){memcpy(&home,bytes.data(),sizeof(home));valid=duck_calibration::HomeEngine::wireValid(home);}
  else if(valid){memcpy(&command,bytes.data(),sizeof(command));valid=duck_calibration::Engine::wireValid(command)||duck_calibration::HomeEngine::controlValid(command);}
  k_mutex_lock(&calibrationMu,K_FOREVER);
  if(!valid){++invalidCalibrationCommands;}
  else if(isHome){if(!homePendingReady&&!calibrationStopPending&&!calibrationQueueCount){homePending=home;homePendingReady=true;}else ++invalidCalibrationCommands;}
  else if(command.op==duck_calibration::CANCEL||command.op==duck_calibration::OFF_ALL){calibrationStop=command;calibrationStopPending=true;calibrationQueueCount=0;homePendingReady=false;}
  else if(calibrationQueueCount<4){calibrationQueue[(calibrationQueueRead+calibrationQueueCount)%4]=command;++calibrationQueueCount;}
  else ++invalidCalibrationCommands;
  k_mutex_unlock(&calibrationMu);
}
static bool takeCalibrationCommand(duck_calibration::Command& command){
  k_mutex_lock(&calibrationMu,K_FOREVER);const bool ready=calibrationStopPending||calibrationQueueCount;
  if(calibrationStopPending){command=calibrationStop;calibrationStopPending=false;}
  else if(calibrationQueueCount){command=calibrationQueue[calibrationQueueRead];calibrationQueueRead=(calibrationQueueRead+1)%4;--calibrationQueueCount;}
  k_mutex_unlock(&calibrationMu);return ready;
}
static bool takeHomeCommand(duck_calibration::HomeCommand& command){
  k_mutex_lock(&calibrationMu,K_FOREVER);const bool ready=homePendingReady;
  if(ready){command=homePending;homePendingReady=false;}k_mutex_unlock(&calibrationMu);return ready;
}

static void fillRawPacket(RawPacket &r, uint16_t freshMask) {
  memcpy(r.magic, "DQR1", 4);
  r.mask = freshMask;
  k_mutex_lock(&motorsMu, K_FOREVER);
  for (unsigned i = 0; i < NUM_MOTORS; ++i) {
    r.pos[i] = motors[i].fbPos;
    r.zero[i] = motors[i].zeroPos;
  }
  k_mutex_unlock(&motorsMu);
}

static void copyCalibrationSnapshot(bool homeOwns, uint32_t now, uint32_t seq) {
  if(homeOwns)homeEngine.calibrationSnapshot(calibrationSnapshot,now,seq);else calibrationEngine.snapshot(calibrationSnapshot,now,seq);
  // Independent engines share a request stream; Host restarts must not replay
  // against an older idle-engine watermark. Compare uint32 sequence with wrap.
  const uint32_t oldReq=calibrationEngine.lastRequest(),homeReq=homeEngine.lastRequest();
  calibrationSnapshot.lastReceivedReqSeq=!oldReq?homeReq:!homeReq?oldReq:
    ((uint32_t)(homeReq-oldReq)<0x80000000u?homeReq:oldReq);
  // The two isolated engines share one boot epoch; sticky evidence is monotonic.
  if(homeEngine.everAttempted()||calibrationEngine.everAttempted())calibrationSnapshot.flags|=duck_calibration::TORQUE_ENABLE_EVER_ATTEMPTED;
}

static void controlTask(void*, void*, void*) {
  Serial1.begin(SERVO_BAUD);
  enableUartFifo(reinterpret_cast<USART_TypeDef*>(DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PATH(zephyr_user), serials, 0))));
  for (unsigned i=0; i<3; ++i) { setTorqueEnable(false); k_msleep(5); }
  // Commissioning: do not change gains at startup.
  // Read temporary KP/KD once at boot (address 50, 2 bytes). Missing IDs stay unset.
  readAllRegisters(ADDR_TEMP_KP, 2); Serial1.flush();
  const uint32_t pdStart=micros();
  while ((uint32_t)(micros()-pdStart)<20000) { parseServoRx(); k_sleep(K_USEC(100)); }
  initImu();
  uint32_t bootNonce=0;for(unsigned attempt=0;attempt<4&&!bootNonce;++attempt)bootNonce=sys_rand32_get();
  calibrationEngine.begin(bootNonce);homeEngine.begin(bootNonce);
  setTorqueEnable(false); // Flushes before probe ownership begins.
  resetServoParser();
  startupProbe.begin(20000);
  uint32_t deadline = micros(), previous = deadline, seq = 0, lastCommandUs = 0;
  uint32_t acceptedSeq = 0, overruns = 0, lastImuUs = 0;
  bool disarmPending=false;
  bool haveCommand = false;
  for (;;) {
    const uint32_t start = micros(), period = start - previous;
    previous = start;
    CommandPacket cmd = {};
    k_mutex_lock(&commandMu, K_FOREVER);
    const bool got = pending;
    if (got) { cmd = latest; pending = false; }
    uint32_t bad = invalidCommands;
    k_mutex_unlock(&commandMu);
    k_mutex_lock(&calibrationMu,K_FOREVER);bad+=invalidCalibrationCommands;k_mutex_unlock(&calibrationMu);
    if (got) {
      // DISARM is the only accepted mode. Targets are never applied, even in RAM.
      acceptedSeq = cmd.seq; haveCommand = true; lastCommandUs = start;
      disarmPending=true;
    }

    servoRxMask = 0;
    // Snapshot ownership before step/takeReport: the final probe tick must not
    // begin normal polling, even when takeReport changes READY to FINISHED.
    const bool probeOwnsThisTick = startupProbe.ownsBus();
    calibrationEngine.setReady(!probeOwnsThisTick);
    duck_calibration::Command calibrationCommand={};
    if(takeCalibrationCommand(calibrationCommand)){
      if(homeEngine.ownsBus()||calibrationCommand.op==duck_calibration::HOME_RUN)homeEngine.control(calibrationCommand,startupProbePort);
      else calibrationEngine.command(calibrationCommand,startupProbePort);
    }
    duck_calibration::HomeCommand homeCommand={};
    if(!probeOwnsThisTick&&takeHomeCommand(homeCommand)&&calibrationEngine.homeReady())homeEngine.prepare(homeCommand,startupProbePort);
    const bool homeOwnsThisTick=homeEngine.ownsBus();
    if (probeOwnsThisTick) {
      const duck_probe::StepResult result = startupProbe.step(startupProbePort);
      if (result.freshFeedbackMask & 1u) {
        applyMotorFb(duck_probe::kTargetId, static_cast<int16_t>(result.rawPosition),
                     hostVelSignMag(result.rawSpeed & 255u, result.rawSpeed >> 8));
      }
      if (result.reportReady && startupProbe.takeReport(completedProbe)) {
        storeProbeReport(completedProbe);
        if (startupProbe.done()) resetServoParser();
      }
      if (updateImu()) lastImuUs = micros();
    } else {
      // Every normal UART operation stays in this branch. No torque-off,
      // drain, normal parser or sync-read may interrupt a probe receive window.
      if(updateImu())lastImuUs=micros();
      if(disarmPending){if(homeOwnsThisTick)homeEngine.stop(startupProbePort);else calibrationEngine.disarm(startupProbePort);disarmPending=false;}
      if(homeOwnsThisTick)homeEngine.startTick(startupProbePort);else calibrationEngine.startTick(startupProbePort);
      const uint32_t readStart = micros();
      while ((uint32_t)(micros()-readStart) < SERVO_REPLY_US) {
        if(homeOwnsThisTick)homeEngine.step(startupProbePort);else calibrationEngine.step(startupProbePort); k_sleep(K_USEC(100));
      }
      if(homeOwnsThisTick)homeEngine.step(startupProbePort);else calibrationEngine.step(startupProbePort);
      servoRxMask=homeOwnsThisTick?homeEngine.tickPositionMask():calibrationEngine.tickPositionMask();
      k_mutex_lock(&motorsMu,K_FOREVER);
      for(unsigned i=0;i<NUM_MOTORS;++i)if(servoRxMask&(1u<<i)){
        motors[i].fbPos=homeOwnsThisTick?homeEngine.rawWord(i):calibrationEngine.rawWord(i);
        motors[i].fbAngleDeg=((homeOwnsThisTick?homeEngine.position(i):calibrationEngine.decodedPosition(i))-motors[i].zeroPos)*M_A/M_N*motors[i].sign;
        motors[i].fbVelDeg=(homeOwnsThisTick?homeEngine.speed(i):calibrationEngine.decodedSpeed(i))*SERVO_SPEED_UNIT*(360.0f/4096.0f)*motors[i].sign;
      }
      k_mutex_unlock(&motorsMu);
    }
    StatePacket s = {};
    memcpy(s.magic, "DQS1", 4); s.version = 1;
    // Policy locked; explicit limited calibration is reported independently.
    s.flags = (imu.ok ? 1 : 0) | ((calibrationEngine.motionPossible()||homeEngine.possibleMotion()) ? 2 : 0) | 0x20;
    s.servoMask = probeOwnsThisTick ? servoRxMask : (homeOwnsThisTick?homeEngine.freshPositionMask(micros()):calibrationEngine.freshPositionMask(micros())); s.seq = ++seq; s.sampleUs = start; s.imuUs = lastImuUs;
    s.commandSeq = acceptedSeq; s.commandAgeUs = haveCommand ? start-lastCommandUs : UINT32_MAX;
    s.periodUs = period; s.overruns = overruns; s.invalidCommands = bad;
    memcpy(s.acc, imu.accRaw, sizeof(s.acc)); memcpy(s.gyro, imu.gyroRaw, sizeof(s.gyro));
    k_mutex_lock(&motorsMu, K_FOREVER);
    for (unsigned i = 0; i < NUM_MOTORS; ++i) {
      s.angle[i] = motors[i].fbAngleDeg; s.velocity[i] = motors[i].fbVelDeg;
      s.target[i] = motors[i].desAngleDeg;
    }
    k_mutex_unlock(&motorsMu);
    s.workUs = micros()-start;
    k_mutex_lock(&stateMu, K_FOREVER);
    if (k_sem_count_get(&reportReady)) ++reportDrops;
    s.reportDrops = reportDrops; snapshot = s;
    copyCalibrationSnapshot(homeOwnsThisTick,micros(),seq);
    homeEngine.snapshot(homeSnapshot,micros(),seq);
    if(calibrationEngine.takeResult(calibrationResult))calibrationResultReady=true;
    k_mutex_unlock(&stateMu);
    k_sem_give(&reportReady);
    deadline += TICK_US;
    const uint32_t now = micros();
    if ((int32_t)(deadline-now) <= 0) {
      const uint32_t missed = (now-deadline)/TICK_US+1;
      overruns += missed; deadline += missed*TICK_US;
    }
    const int32_t remaining = (int32_t)(deadline-micros());
    if (remaining > 0) k_sleep(K_USEC(remaining));
  }
}

static void reportTask(void*, void*, void*) {
  MsgPack::bin_t<uint8_t> bytes, probeBytes, calibrationBytes, calibrationResultBytes, homeBytes;
  unsigned reports = 0;uint8_t lastHomePhase=255;uint32_t lastHomeAck=0;
  const uint32_t replayStartUs = micros();
  uint32_t lastReplayUs = replayStartUs;
  uint8_t replayNext = 0;
  bool replayExpired = false;
  bytes.resize(sizeof(StatePacket));
  probeBytes.resize(duck_probe::kWireSize);
  calibrationBytes.resize(sizeof(duck_calibration::Snapshot));
  calibrationResultBytes.resize(sizeof(duck_calibration::ActionResult));homeBytes.resize(sizeof(duck_calibration::HomeSnapshot));
  for (;;) {
    k_sem_take(&reportReady, K_FOREVER);
    k_mutex_lock(&stateMu, K_FOREVER);
    memcpy(bytes.data(), &snapshot, sizeof(snapshot));
    const uint16_t freshMask = snapshot.servoMask; // Same locked DQS snapshot; no report-thread Engine access.
    memcpy(calibrationBytes.data(),&calibrationSnapshot,sizeof(calibrationSnapshot));
    memcpy(homeBytes.data(),&homeSnapshot,sizeof(homeSnapshot));
    const bool homeChanged=lastHomePhase!=homeSnapshot.phase||lastHomeAck!=homeSnapshot.operationAckSeq;
    lastHomePhase=homeSnapshot.phase;lastHomeAck=homeSnapshot.operationAckSeq;
    const bool hasCalibrationResult=calibrationResultReady;
    if(hasCalibrationResult){memcpy(calibrationResultBytes.data(),&calibrationResult,sizeof(calibrationResult));calibrationResultReady=false;}
    k_mutex_unlock(&stateMu);
    Bridge.notify("duck_state", bytes);
    if((reports%5)==0||homeChanged)Bridge.notify("duck_home_state",homeBytes);
    if((reports%5)==0||hasCalibrationResult)Bridge.notify("duck_calibration_state",calibrationBytes);
    if(hasCalibrationResult)Bridge.notify("duck_calibration_result",calibrationResultBytes);
    if ((reports % 20) == 0) {
      RawPacket raw = {};
      fillRawPacket(raw, freshMask);
      MsgPack::bin_t<uint8_t> rawBytes;
      rawBytes.resize(sizeof(raw));
      memcpy(rawBytes.data(), &raw, sizeof(raw));
      Bridge.notify("duck_raw", rawBytes);
    }
    // Preserve the first-send queue. Linux's receiving callback may start later
    // than Bridge itself: replay stored bytes at most once/second for 120s after
    // this report task starts, without rerunning any probe or touching UART.
    const uint32_t replayNowUs = micros();
    if (!replayExpired && (uint32_t)(replayNowUs-replayStartUs) >= PROBE_REPLAY_DURATION_US)
      replayExpired = true; // Latch; a later full micros() wrap must not restart it.
    if (copyProbeReport(probeBytes)) {
      Bridge.notify("duck_probe", probeBytes);
      markProbeReportSent();
    } else if (!replayExpired &&
               (uint32_t)(replayNowUs-lastReplayUs) >= PROBE_REPLAY_INTERVAL_US &&
               copyProbeReplay(replayNext, probeBytes)) {
      // Recheck after waiting for the mutex; expiration never sends a replay.
      if ((uint32_t)(micros()-replayStartUs) < PROBE_REPLAY_DURATION_US) {
        Bridge.notify("duck_probe", probeBytes);
        lastReplayUs = micros();
      } else {
        replayExpired = true;
      }
    }
    if ((++reports % 100) == 0) {
      MsgPack::bin_t<uint8_t> pd;
      pd.resize(32);
      pd[0]=servoPdMask & 255; pd[1]=servoPdMask >> 8;
      memcpy(pd.data()+2, servoBootP, 15); memcpy(pd.data()+17, servoBootD, 15);
      Bridge.notify("duck_boot_pd", pd);
    }
  }
}

void setup() {
  k_mutex_init(&motorsMu);
  k_mutex_init(&commandMu);
  k_mutex_init(&stateMu);
  k_mutex_init(&probeMu);
  k_mutex_init(&calibrationMu);
  k_sem_init(&reportReady, 0, 1);
  for (unsigned i = 0; i < NUM_MOTORS; ++i) {
    motors[i] = {}; motors[i].id = MOTOR_IDS[i]; motors[i].zeroPos = ZERO_POS[i];
    motors[i].fbPos = ZERO_POS[i];
    motors[i].sign = MOTOR_SIGN[i]; motors[i].desAngleDeg = DEFAULT_DEG[i];
    motors[i].desPos = CAL_CENTER_POS;
  }
  // Start the watchdog/hardware loop even if Linux Bridge startup stalls.
  k_thread_create(&controlThread, controlStack, K_THREAD_STACK_SIZEOF(controlStack),
                  controlTask, nullptr, nullptr, nullptr, 4, 0, K_NO_WAIT);
  k_thread_name_set(&controlThread, "duck_control100");
  ARDUINO_ROUTER_SERIAL.begin(BRIDGE_BAUD);
  enableUartFifo(reinterpret_cast<USART_TypeDef*>(DT_REG_ADDR(ARDUINO_ROUTER_PHANDLE)));
  // Control/watchdog task already runs. Only startup maintenance is retried;
  // each request has a fresh odd ID and a 200 ms deadline, never a user command.
  while (!Bridge.beginBounded(BRIDGE_BAUD, 200)) k_msleep(800);
  while (!Bridge.bindLocalOnce("duck_command", receiveCommand)) k_msleep(25);
  while (!Bridge.bindLocalOnce("duck_calibration", receiveCalibration)) k_msleep(25);
  // The background refresher performs initial/renewed Router registration.
  // Lost ACK/code 5 never blocks reports and is not counted as ownership proof.
  routeHandlersBound = true; // Local callbacks bound once, before worker creation.
  k_thread_create(&routeThread, routeStack, K_THREAD_STACK_SIZEOF(routeStack),
                  routeTask, nullptr, nullptr, nullptr, 7, 0, K_NO_WAIT);
  k_thread_name_set(&routeThread, "duck_route_refresh");
  k_thread_create(&reportThread, reportStack, K_THREAD_STACK_SIZEOF(reportStack),
                  reportTask, nullptr, nullptr, nullptr, 6, 0, K_NO_WAIT);
  k_thread_name_set(&reportThread, "duck_report100");
  // GCC 14 otherwise emits a tail B.W to a flash syscall from RAM, outside
  // Thumb JUMP24 range; the Uno Q extension loader rejects it with ENOEXEC.
  asm volatile("nop" ::: "memory");
}
void loop() { delay(100); }
