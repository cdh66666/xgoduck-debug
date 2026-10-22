#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <string.h>
#include "duck_config.h"
#include "protocol.h"
#include "scs_bus.h"
#include "qmi8658.h"
#include "uart_fifo.h"
#include <SCServo.h>
#include "factory_calibration.h"
#include "route_refresh.h"

Motor motors[NUM_MOTORS];
struct k_mutex motorsMu;
// App sketches are Zephyr loadable extensions: initialize kernel objects at runtime.
// Avoid iterable kernel-object sections in the loadable sketch.
static struct k_mutex commandMu, stateMu, hostMu;
static struct k_sem reportReady;
K_THREAD_STACK_DEFINE(controlStack, 4096);
K_THREAD_STACK_DEFINE(reportStack, 4096);
static struct k_thread controlThread, reportThread;
static StatePacket snapshot = {};
static DisplayPacket displaySnapshot = {};
static CurrentPacket currentSnapshot = {};
static DisplayAttitude displayAttitude;
static uint32_t displayBoot[2]={};
static uint32_t displayComputeUs=0,displayMaxComputeUs=0;
static uint32_t voltageRequestUs=0,voltageMaxRequestUs=0;
static CommandPacket latest = {};
static bool pending = false;
static uint32_t invalidCommands = 0, reportDrops = 0;

static HostBusMode hostMode = HOST_NORMAL;
static CalPacket pendingCal = {};
static bool calPending = false;
static ServoPacket pendingServo = {};
static bool servoPending = false;
static ServoReplyPacket servoReply = {};
static bool servoReplyReady = false;
static bool rawNotifyEveryTick = false;
static HLSCL factoryServo;
static uint16_t factoryHeldMask=0;
static FactoryPacket pendingFactory={};
static bool factoryPending=false, factoryReplyReady=false;
static FactoryReply factoryReply={};

struct FactoryBus {
  bool ack(int value) {return value==1 && !factoryServo.getState() && !factoryServo.getLastError();}
  int read(uint8_t id,uint8_t address,uint8_t length) {
    const int value=length==1?factoryServo.readByte(id,address):factoryServo.readWord(id,address);
    return factoryServo.getState() || factoryServo.getLastError() ? -1 : value;
  }
  bool torque(uint8_t id,uint8_t value) {return ack(factoryServo.EnableTorque(id,value));}
  bool calibrationOfs(uint8_t id) {return ack(factoryServo.CalibrationOfs(id));}
  bool lock(uint8_t id) {return ack(factoryServo.LockEprom(id));}
  bool goal(uint8_t id,uint16_t value) {return ack(factoryServo.writeWord(id,42,value));}
  void settle() {k_msleep(50);}
};

static void receiveFactory(MsgPack::bin_t<uint8_t> bytes) {
  FactoryPacket pkt={};
  if(bytes.size()!=sizeof(pkt)) return;
  memcpy(&pkt,bytes.data(),sizeof(pkt));
  if(memcmp(pkt.magic,"DQF1",4) || pkt.op>FACTORY_OFF || pkt.targetId>253 || pkt.reserved) return;
  k_mutex_lock(&hostMu,K_FOREVER);
  if(!factoryPending) {pendingFactory=pkt; factoryPending=true;}
  k_mutex_unlock(&hostMu);
}

static void applyFactoryOp(const FactoryPacket &pkt) {
  FactoryReply r={};
  memcpy(r.magic,"DQF2",4); r.op=pkt.op; r.targetId=pkt.targetId;
  r.request=pkt.request; r.capabilities=FACTORY_CAPS;
  FactoryBus bus;
  const int index=motorIndexById(pkt.targetId);
  drainServoRx();
  if(pkt.op==FACTORY_CAPABILITIES) {}
  else if(pkt.op==FACTORY_OFF) factoryOff(bus,MOTOR_IDS,NUM_MOTORS,factoryHeldMask,r);
  else if(hostMode!=HOST_SERVO_DEBUG) r.error=FACTORY_MODE;
  else if(index<0) r.error=FACTORY_ID;
  else {
    k_mutex_lock(&motorsMu,K_FOREVER); r.zeroPos=motors[index].zeroPos; k_mutex_unlock(&motorsMu);
    factoryAxis(bus,pkt.op,pkt.targetId,index,factoryHeldMask,r);
    if(r.reserved & 1) {
      k_mutex_lock(&motorsMu,K_FOREVER);
      motors[index].zeroPos=r.zeroPos; motors[index].desPos=r.zeroPos;
      k_mutex_unlock(&motorsMu);
    }
  }
  drainServoRx();
  r.heldMask=factoryHeldMask;
  k_mutex_lock(&hostMu,K_FOREVER);
  factoryReply=r; factoryReplyReady=true;
  k_mutex_unlock(&hostMu);
}

static void receiveCommand(MsgPack::bin_t<uint8_t> bytes) {
  CommandPacket cmd;
  bool valid = bytes.size() == sizeof(cmd);
  if (valid) {
    memcpy(&cmd, bytes.data(), sizeof(cmd));
    valid = memcmp(cmd.magic, "DQC1", 4) == 0 && cmd.mode <= HOLD &&
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

static void receiveCal(MsgPack::bin_t<uint8_t> bytes) {
  CalPacket pkt;
  bool valid = bytes.size() == sizeof(pkt);
  if (valid) {
    memcpy(&pkt, bytes.data(), sizeof(pkt));
    valid = memcmp(pkt.magic, "DQL1", 4) == 0 && pkt.op <= CAL_SET_ZEROS;
    if (pkt.op == CAL_FINISH_ONE) valid = valid && pkt.index < NUM_MOTORS;
    if (pkt.op == CAL_SET_RAW || pkt.op == CAL_SET_ZEROS) {
      for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        if (pkt.mask & (1u << i))
          valid = valid && pkt.raw[i] <= POS_MAX;
      }
    }
  }
  k_mutex_lock(&hostMu, K_FOREVER);
  if (valid) { pendingCal = pkt; calPending = true; }
  else {
    k_mutex_lock(&commandMu, K_FOREVER); ++invalidCommands; k_mutex_unlock(&commandMu);
  }
  k_mutex_unlock(&hostMu);
}

static void receiveServo(MsgPack::bin_t<uint8_t> bytes) {
  ServoPacket pkt;
  bool valid = bytes.size() == sizeof(pkt);
  if (valid) {
    memcpy(&pkt, bytes.data(), sizeof(pkt));
    valid = memcmp(pkt.magic, "DQV1", 4) == 0 && pkt.op <= SERVO_PING &&
            pkt.targetId <= 253;
    if (pkt.op == SERVO_GOTO) valid = valid && pkt.rawPos <= POS_MAX;
    if (pkt.op == SERVO_SET_ID) valid = valid && pkt.newId <= 253;
  }
  k_mutex_lock(&hostMu, K_FOREVER);
  if (valid) { pendingServo = pkt; servoPending = true; }
  else {
    k_mutex_lock(&commandMu, K_FOREVER); ++invalidCommands; k_mutex_unlock(&commandMu);
  }
  k_mutex_unlock(&hostMu);
}

static void fillRawPacket(RawPacket &r) {
  memcpy(r.magic, "DQR1", 4);
  r.mask = servoRxMask;
  k_mutex_lock(&motorsMu, K_FOREVER);
  for (unsigned i = 0; i < NUM_MOTORS; ++i) {
    r.pos[i] = motors[i].fbPos;
    r.zero[i] = motors[i].zeroPos;
  }
  k_mutex_unlock(&motorsMu);
}

static void applyCalOp(const CalPacket &pkt) {
  switch (pkt.op) {
    case CAL_ENTER:
      hostMode = HOST_CAL;
      rawNotifyEveryTick = true;
      k_mutex_lock(&motorsMu, K_FOREVER);
      for (unsigned i = 0; i < NUM_MOTORS; ++i) motors[i].desPos = CAL_CENTER_POS;
      k_mutex_unlock(&motorsMu);
      setTempKpKd(TEMP_KP_CAL, TEMP_KD_CAL);
      setTorqueEnable(true);
      k_mutex_lock(&motorsMu, K_FOREVER);
      sendMotorRawPos();
      k_mutex_unlock(&motorsMu);
      Serial1.flush();
      break;
    case CAL_EXIT:
      hostMode = HOST_NORMAL;
      rawNotifyEveryTick = false;
      setTorqueEnable(false);
      setRunKpKd();
      break;
    case CAL_SET_RAW:
      if (hostMode != HOST_CAL) break;
      k_mutex_lock(&motorsMu, K_FOREVER);
      for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        if (pkt.mask & (1u << i)) motors[i].desPos = satPos(pkt.raw[i]);
      }
      k_mutex_unlock(&motorsMu);
      break;
    case CAL_FINISH_ONE:
      if (hostMode != HOST_CAL || pkt.index >= NUM_MOTORS) break;
      k_mutex_lock(&motorsMu, K_FOREVER);
      motors[pkt.index].zeroPos = motors[pkt.index].fbPos;
      k_mutex_unlock(&motorsMu);
      break;
    case CAL_SET_ZEROS:
      k_mutex_lock(&motorsMu, K_FOREVER);
      for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        if (pkt.mask & (1u << i)) motors[i].zeroPos = satPos(pkt.raw[i]);
      }
      k_mutex_unlock(&motorsMu);
      break;
  }
}

static void applyServoOp(const ServoPacket &pkt) {
  ServoReplyPacket reply = {};
  memcpy(reply.magic, "DQS2", 4);
  reply.op = pkt.op;
  reply.targetId = pkt.targetId;
  reply.ok = 1;
  switch (pkt.op) {
    case SERVO_ENTER:
      hostMode = HOST_SERVO_DEBUG;
      rawNotifyEveryTick = true;
      setTorqueEnable(false);
      factoryHeldMask=0;
      setRunKpKd();
      break;
    case SERVO_EXIT:
      factoryHeldMask=0;
      hostMode = HOST_NORMAL;
      rawNotifyEveryTick = false;
      setTorqueEnable(false);
      setRunKpKd();
      break;
    case SERVO_UNLOCK:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      servoUnlock(pkt.targetId);
      break;
    case SERVO_SET_ID:
      factoryHeldMask=0;
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      setTorqueEnable(false);
      servoUnlock(pkt.targetId);
      k_msleep(2);
      servoSetId(pkt.targetId, pkt.newId);
      k_msleep(2);
      {
        // ID writes leave EEPROM unlocked. Relock at the new address and verify.
        FactoryBus bus;
        const bool locked=bus.lock(pkt.newId);
        const int readback=bus.read(pkt.newId,55,1);
        reply.ok=locked && readback==1;
        reply.rawPos=readback<0 ? 0xffff : readback;
      }
      break;
    case SERVO_GOTO:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      setRunKpKd();
      setTorqueEnable(true);
      writeSingleGoalRaw(pkt.targetId, pkt.rawPos);
      Serial1.flush();
      break;
    case SERVO_SET_PERM_KP_KD:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      servoUnlock(pkt.targetId);
      k_msleep(2);
      servoSetPermKpKd(pkt.targetId, pkt.kp, pkt.kd);
      break;
    case SERVO_PING: {
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      drainServoRx();
      servoPingId = pkt.targetId;
      servoPingSeen = false;
      scsWriteBuf(pkt.targetId, 0x01, nullptr, 0);
      Serial1.flush();
      const uint32_t start = micros();
      while ((uint32_t)(micros() - start) < SERVO_REPLY_US) {
        parseServoRx();
        k_sleep(K_USEC(100));
      }
      parseServoRx();
      reply.ok = servoPingSeen ? 1 : 0;
      servoPingId = 0xFF;
      break;
    }
    case SERVO_READ:
      if (hostMode != HOST_SERVO_DEBUG) { reply.ok = 0; break; }
      drainServoRx();
      servoReadId = pkt.targetId;
      servoReadSeen = false;
      servoReadPos = 0;
      readSinglePresent(pkt.targetId);
      Serial1.flush();
      {
        const uint32_t start = micros();
        while ((uint32_t)(micros() - start) < SERVO_REPLY_US) {
          parseServoRx();
          k_sleep(K_USEC(100));
        }
        parseServoRx();
      }
      {
        const int idx = motorIndexById(pkt.targetId);
        reply.ok = servoReadSeen ? 1 : 0;
        reply.rawPos = servoReadPos;
        servoReadId = 0xFF;
        if (idx >= 0) {
          k_mutex_lock(&motorsMu, K_FOREVER);
          reply.zeroPos = motors[idx].zeroPos;
          k_mutex_unlock(&motorsMu);
        }
      }
      break;
  }
  k_mutex_lock(&hostMu, K_FOREVER);
  servoReply = reply;
  servoReplyReady = true;
  k_mutex_unlock(&hostMu);
}

static void controlTask(void*, void*, void*) {
  Serial1.begin(SERVO_BAUD);
  factoryServo.pSerial=&Serial1; factoryServo.IOTimeOut=30;
  enableUartFifo(reinterpret_cast<USART_TypeDef*>(DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PATH(zephyr_user), serials, 0))));
  for (unsigned i=0; i<3; ++i) { setTorqueEnable(false); k_msleep(5); }
  setRunKpKd();
  // Read temporary KP/KD once at boot (address 50, 2 bytes). Missing IDs stay unset.
  servoBootPdReading=true;
  readAllRegisters(ADDR_TEMP_KP, 2); Serial1.flush();
  const uint32_t pdStart=micros();
  while ((uint32_t)(micros()-pdStart)<20000) { parseServoRx(); k_sleep(K_USEC(100)); }
  servoBootPdReading=false;
  initImu();
  uint32_t deadline = micros(), previous = deadline, seq = 0, lastCommandUs = 0;
  uint32_t acceptedSeq = 0, ttl = COMMAND_TTL_US, overruns = 0, lastImuUs = 0;
  uint32_t lastHealthyUs = 0;
  bool enabled = false, timedOut = false, haveCommand = false;
  for (;;) {
    const uint32_t start = micros(), period = start - previous;
    previous = start;

    CalPacket calPkt;
    ServoPacket servoPkt;
    bool gotCal = false, gotServo = false;
    k_mutex_lock(&hostMu, K_FOREVER);
    if (calPending) { calPkt = pendingCal; calPending = false; gotCal = true; }
    if (servoPending) { servoPkt = pendingServo; servoPending = false; gotServo = true; }
    k_mutex_unlock(&hostMu);
    if (gotCal) applyCalOp(calPkt);
    if (gotServo) applyServoOp(servoPkt);
    FactoryPacket factoryPkt={}; bool gotFactory=false;
    k_mutex_lock(&hostMu,K_FOREVER);
    if(factoryPending) {factoryPkt=pendingFactory; factoryPending=false; gotFactory=true;}
    k_mutex_unlock(&hostMu);
    if(gotFactory) applyFactoryOp(factoryPkt);

    CommandPacket cmd;
    k_mutex_lock(&commandMu, K_FOREVER);
    const bool got = pending;
    if (got) { cmd = latest; pending = false; }
    const uint32_t bad = invalidCommands;
    k_mutex_unlock(&commandMu);

    const bool busTaken = hostMode != HOST_NORMAL;
    if (got && !busTaken) {
      const bool fresh = cmd.mode == DISARM || (uint32_t)(start - cmd.sourceUs) <= MAX_SOURCE_AGE_US;
      const bool ordered = !haveCommand || cmd.mode == DISARM || (int32_t)(cmd.seq - acceptedSeq) > 0;
      if (fresh && ordered) {
        acceptedSeq = cmd.seq; haveCommand = true; lastCommandUs = start; ttl = cmd.ttlUs;
        if (cmd.mode == DISARM) { enabled = false; timedOut = false; }
        if (cmd.mode == ARM && !enabled && imu.ok &&
            (uint32_t)(start-lastImuUs) < 50000) {
          enabled = true; timedOut = false;
          lastHealthyUs = start;
          setRunKpKd();
        }
        k_mutex_lock(&motorsMu, K_FOREVER);
        for (unsigned i = 0; i < NUM_MOTORS; ++i) motors[i].desAngleDeg = cmd.target[i];
        k_mutex_unlock(&motorsMu);
      } else {
        k_mutex_lock(&commandMu, K_FOREVER); ++invalidCommands; k_mutex_unlock(&commandMu);
      }
    }
    if (busTaken) {
      enabled = false;
      timedOut = false;
    } else if (enabled && ((uint32_t)(start-lastCommandUs) > ttl ||
                           (uint32_t)(start-lastHealthyUs) > 100000)) {
      enabled = false; timedOut = true;
    }
    static bool previousEnabled = false;
    if (hostMode == HOST_CAL) {
      k_mutex_lock(&motorsMu, K_FOREVER); sendMotorRawPos(); k_mutex_unlock(&motorsMu);
      Serial1.flush();
    } else if (enabled) {
      k_mutex_lock(&motorsMu, K_FOREVER); sendMotorDesPos(); k_mutex_unlock(&motorsMu);
      Serial1.flush();
    }
    if (!busTaken) {
      if (enabled != previousEnabled || !enabled) setTorqueEnable(enabled);
      previousEnabled = enabled;
    } else {
      previousEnabled = false;
    }
    drainServoRx();
    servoRxMask = 0;
    readAllState(); Serial1.flush();
    const uint32_t readStart = micros();
    currentSampler.beginWindow(readStart);
    bool voltageIssuedThisWindow=false;
    const bool observedImu=updateImu();
    if(observedImu) lastImuUs=micros();
    const uint32_t displayStart=micros();
    const float radians=DisplayMath::RADIANS_PI/(180.f*32.f),accel=9.8f/16384.f;
    displayAttitude.observe({imu.accRaw[0]*accel,imu.accRaw[1]*accel,imu.accRaw[2]*accel},
      {imu.gyroRaw[0]*radians,imu.gyroRaw[1]*radians,imu.gyroRaw[2]*radians},lastImuUs,observedImu);
    displayComputeUs=micros()-displayStart;
    if(displayComputeUs>displayMaxComputeUs)displayMaxComputeUs=displayComputeUs;
    while ((uint32_t)(micros()-readStart) < SERVO_REPLY_US) {
      parseServoRx();
      const uint32_t used=micros()-readStart;
      const uint32_t remaining=used<SERVO_REPLY_US?SERVO_REPLY_US-used:0;
      if(displayVoltage.request(micros(),servoRxMask,remaining,hostMode==HOST_NORMAL && !currentSampler.attemptedWindow)) {
        voltageIssuedThisWindow=true;
        const uint32_t voltageStart=micros();
        const uint8_t read[2]={62,1};scsWriteBuf(10,0x02,read,2);Serial1.flush();
        voltageRequestUs=micros()-voltageStart;
        if(voltageRequestUs>voltageMaxRequestUs)voltageMaxRequestUs=voltageRequestUs;
      }
      if(currentSampler.request(micros(),servoRxMask,remaining,hostMode==HOST_NORMAL,displayVoltage.pending,voltageIssuedThisWindow)) {
        const uint32_t currentStart=micros();
        readAllRegisters(69,2);Serial1.flush();
        currentSampler.sent(micros()-currentStart);
      }
      k_sleep(K_USEC(100));
    }
    parseServoRx();displayVoltage.endWindow();currentSampler.endWindow();
    if (imu.ok) lastHealthyUs = micros();
    StatePacket s = {};
    memcpy(s.magic, "DQS1", 4); s.version = 1;
    s.flags = (imu.ok ? 1 : 0) | (enabled ? 2 : 0) | (timedOut ? 4 : 0) |
              ((hostMode == HOST_CAL) ? 8 : 0) | ((hostMode == HOST_SERVO_DEBUG) ? 16 : 0);
    s.servoMask = servoRxMask; s.seq = ++seq; s.sampleUs = start; s.imuUs = lastImuUs;
    CurrentPacket current={};currentSampler.fill(current,displayBoot[0],displayBoot[1],s.seq);
    s.commandSeq = acceptedSeq; s.commandAgeUs = haveCommand ? start-lastCommandUs : UINT32_MAX;
    s.periodUs = period; s.overruns = overruns; s.invalidCommands = bad;
    memcpy(s.acc, imu.accRaw, sizeof(s.acc)); memcpy(s.gyro, imu.gyroRaw, sizeof(s.gyro));
    k_mutex_lock(&motorsMu, K_FOREVER);
    for (unsigned i = 0; i < NUM_MOTORS; ++i) {
      s.angle[i] = motors[i].fbAngleDeg; s.velocity[i] = motors[i].fbVelDeg;
      s.target[i] = motors[i].desAngleDeg;
    }
    k_mutex_unlock(&motorsMu);
    DisplayPacket d={};memcpy(d.magic,"DQD1",4);d.version=2;
    d.flags=(displayAttitude.valid&&imu.ok?1:0)|(displayAttitude.haveHeading&&!displayAttitude.heldHeading?2:0)|
      (displayAttitude.heldHeading?4:0)|(displayAttitude.continuous?8:0)|(displayVoltage.valid?16:0)|
      (displayAttitude.bias.phase==BootGyroBias::READY?32:displayAttitude.bias.phase==BootGyroBias::CALIBRATING?64:128);
    d.voltageId=10;d.voltageRaw=displayVoltage.raw;
    d.bootLow=displayBoot[0];d.bootHigh=displayBoot[1];d.seq=s.seq;d.sampleUs=s.sampleUs;
    d.imuUs=displayAttitude.sourceUs;d.segment=displayAttitude.segment;
    const auto &q=displayAttitude.q;const auto &r=displayAttitude.relative;const auto &g=displayAttitude.gravity;
    d.quaternion[0]=q.x;d.quaternion[1]=q.y;d.quaternion[2]=q.z;d.quaternion[3]=q.w;
    d.relative[0]=r.x;d.relative[1]=r.y;d.relative[2]=r.z;d.relative[3]=r.w;
    d.gravity[0]=g.x;d.gravity[1]=g.y;d.gravity[2]=g.z;d.headingDeg=displayAttitude.headingDeg;
    d.voltageSampleUs=displayVoltage.sampleUs;d.voltageAttempts=displayVoltage.attempts;
    d.voltageReplies=displayVoltage.replies;d.voltageSkipped=displayVoltage.skipped;
    d.computeUs=displayComputeUs;d.maxComputeUs=displayMaxComputeUs;
    d.voltageRequestUs=voltageRequestUs;d.maxVoltageRequestUs=voltageMaxRequestUs;
    const auto &bias=displayAttitude.bias.mean;const float degrees=180.f/DisplayMath::RADIANS_PI;
    d.biasGyroDegrees[0]=bias.x*degrees;d.biasGyroDegrees[1]=bias.y*degrees;d.biasGyroDegrees[2]=bias.z*degrees;
    d.biasSamples=displayAttitude.bias.samples;
    s.workUs = micros()-start;
    k_mutex_lock(&stateMu, K_FOREVER);
    if (k_sem_count_get(&reportReady)) ++reportDrops;
    s.reportDrops = reportDrops; snapshot = s; displaySnapshot=d; currentSnapshot=current;
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
  MsgPack::bin_t<uint8_t> bytes;
  unsigned reports = 0;
  bytes.resize(sizeof(StatePacket)+sizeof(CurrentPacket));
  for (;;) {
    k_sem_take(&reportReady, K_FOREVER);
    k_mutex_lock(&stateMu, K_FOREVER);
    memcpy(bytes.data(), &snapshot, sizeof(snapshot));
    memcpy(bytes.data()+sizeof(StatePacket), &currentSnapshot, sizeof(currentSnapshot));
    DisplayPacket display=displaySnapshot;
    k_mutex_unlock(&stateMu);
    Bridge.notify("duck_state", bytes);
    if((reports%5)==0) {
      MsgPack::bin_t<uint8_t> db;db.resize(sizeof(display));memcpy(db.data(),&display,sizeof(display));
      Bridge.notify("duck_display",db);
    }
    const bool wantRaw = rawNotifyEveryTick || ((reports % 20) == 0);
    if (wantRaw) {
      RawPacket raw = {};
      fillRawPacket(raw);
      MsgPack::bin_t<uint8_t> rawBytes;
      rawBytes.resize(sizeof(raw));
      memcpy(rawBytes.data(), &raw, sizeof(raw));
      Bridge.notify("duck_raw", rawBytes);
    }
    bool replyReady = false;
    ServoReplyPacket reply = {};
    k_mutex_lock(&hostMu, K_FOREVER);
    if (servoReplyReady) { reply = servoReply; servoReplyReady = false; replyReady = true; }
    k_mutex_unlock(&hostMu);
    if (replyReady) {
      MsgPack::bin_t<uint8_t> rb;
      rb.resize(sizeof(reply));
      memcpy(rb.data(), &reply, sizeof(reply));
      Bridge.notify("duck_servo_reply", rb);
    }
    FactoryReply fr={}; bool frReady=false;
    k_mutex_lock(&hostMu,K_FOREVER);
    if(factoryReplyReady) {fr=factoryReply; factoryReplyReady=false; frReady=true;}
    k_mutex_unlock(&hostMu);
    if(frReady) {
      MsgPack::bin_t<uint8_t> rb; rb.resize(sizeof(fr)); memcpy(rb.data(),&fr,sizeof(fr));
      Bridge.notify("duck_factory_reply",rb);
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
  k_mutex_init(&hostMu);
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
  // Display boot marker only: existing bounded clocks, no RNG or persistence.
  // Captured after control/watchdog creation; Host/seq epochs handle restarts.
  displayBoot[0]=micros();displayBoot[1]=millis();
  ARDUINO_ROUTER_SERIAL.begin(BRIDGE_BAUD);
  enableUartFifo(reinterpret_cast<USART_TypeDef*>(DT_REG_ADDR(ARDUINO_ROUTER_PHANDLE)));
  // Bridge startup is driven by the bounded, rate-limited loop below.
  // The watchdog/control task remains alive and unloaded while Linux boots.
  // GCC 14 otherwise emits a tail B.W to a flash syscall from RAM, outside
  // Thumb JUMP24 range; the Uno Q extension loader rejects it with ENOEXEC.
  asm volatile("nop" ::: "memory");
}
struct StartupPort {
  bool reported=false;
  bool beginBounded(uint32_t deadlineMs) {return Bridge.beginBounded(BRIDGE_BAUD,deadlineMs);}
  bool bindLocal(unsigned index) {
    switch(index) {
      case 0:return Bridge.bindLocalOnce("duck_command",receiveCommand);
      case 1:return Bridge.bindLocalOnce("duck_cal",receiveCal);
      case 2:return Bridge.bindLocalOnce("duck_servo",receiveServo);
      case 3:return Bridge.bindLocalOnce("duck_factory",receiveFactory);
      default:return false;
    }
  }
  bool startReports() {
    if(reported) return true;
    const k_tid_t tid=k_thread_create(&reportThread,reportStack,K_THREAD_STACK_SIZEOF(reportStack),
                                    reportTask,nullptr,nullptr,nullptr,6,0,K_NO_WAIT);
    if(!tid) return false;
    k_thread_name_set(&reportThread,"duck_report100");reported=true;return true;
  }
};
struct RoutePort {
  bool allowed() {
    if(k_mutex_lock(&stateMu,K_NO_WAIT)!=0) return false;
    const uint32_t seq=snapshot.seq,sampleUs=snapshot.sampleUs;
    const uint8_t flags=snapshot.flags;
    k_mutex_unlock(&stateMu);
    return routeRefreshAllowed(seq,flags,sampleUs,micros());
  }
  bool registerOnly(const char *name,uint32_t deadlineMs) {
    return Bridge.registerOnly(name,deadlineMs);
  }
};
void loop() {
  static StartupPort startupPort;
  static RouteStartup startup;
  if(!startup.reportsStarted()) {
    startup.tick(k_uptime_get_32(),startupPort);
    if(!startup.reportsStarted()) {delay(100);return;}
  }
  static RoutePort port;
  static RouteRefresh refresh;
  refresh.tick(k_uptime_get_32(),port);
  delay(100);
}
