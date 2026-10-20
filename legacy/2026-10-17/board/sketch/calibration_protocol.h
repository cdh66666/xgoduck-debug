#pragma once
#include <stdint.h>

namespace duck_calibration {
constexpr uint8_t kVersion=1, kCount=15;
// Revision3 binds fixed-group capture; revision2 bound sticky enable-attempt bit7.
// Commands and action results stay at version1; snapshot layout stays238 bytes.
constexpr uint8_t SNAPSHOT_VERSION=3;
constexpr uint16_t kAllMask=0x7fff;
constexpr uint8_t kIds[kCount]={10,11,12,13,14,20,21,22,23,24,30,31,32,33,34};
constexpr uint32_t kSourceMaxUs=150000, kMetadataMaxUs=750000, kHeartbeatUs=250000;
constexpr uint32_t kSessionUs=30000000, kMoveUs=800000, kReadWindowUs=6000;
constexpr uint32_t kCaptureHoldUs=2000000;
constexpr int16_t kUnknownPosition=(-32767-1), kStepMax=22, kOffsetMax=114;
enum Op:uint8_t { READ=0,ARM=1,STEP=2,HEARTBEAT=3,CANCEL=4,CAPTURE_ALL=5,OFF_ALL=6,DIAG=7,CAPTURE_GROUP=8 };
constexpr uint8_t PARAM_READ=10;
enum SequenceOp:uint8_t { SEQ_BEGIN=0,SEQ_SELECT=1,SEQ_HOLD=2,SEQ_HEARTBEAT=3,SEQ_STOP=4 };
struct __attribute__((packed)) SequenceCommand {
  char magic[4];uint8_t version,op,targetId,flags;
  uint32_t bootNonce,requestSeq,session,sourceUs,ttlUs,selectionSeq;
  int16_t expectedPosition;uint8_t expectedMode,reserved;
};
struct __attribute__((packed)) SequenceSnapshot {
  char magic[4];uint8_t version,phase,result,error;
  uint32_t bootNonce,stateSeq,sampleUs,operationAckSeq,session,expiresUs,lastRequestSeq,selectionSeq,captureSeq;
  uint16_t heldMask,freshMask,metadataMask,offMask,enabledMask;
  uint8_t selectedId,stage;int16_t capturePosition;uint8_t captureMode,reserved;
  int16_t goals[kCount],positions[kCount];uint8_t modes[kCount],torques[kCount];
  uint32_t captureStartUs,captureUs,heldUs;
};
struct __attribute__((packed)) ParameterSnapshot {
  char magic[4];uint8_t version,result,error,targetId;
  uint32_t bootNonce,requestSeq,sampleUs,parameter21Us,parameter40Us;
  uint8_t parameter21[7],parameter40[12],reserved;
};
struct __attribute__((packed)) CurrentSnapshot {
  char magic[4];uint8_t version,reserved;uint16_t validMask;
  uint32_t bootNonce,stateSeq,sampleUs;uint16_t rawWords[kCount],agesMs[kCount];
};
static_assert(sizeof(SequenceCommand)==36,"DZC1 command ABI");
static_assert(sizeof(SequenceSnapshot)==162,"DZS1 snapshot ABI");
static_assert(sizeof(ParameterSnapshot)==48,"DPC1 parameters ABI");
static_assert(sizeof(CurrentSnapshot)==80,"DIC1 current ABI");
enum Group:uint8_t { LEFT=1,RIGHT=2,HEAD=3 };
static inline uint16_t groupMask(uint8_t group){return group==LEFT?0x001f:group==RIGHT?0x03e0:group==HEAD?0x7c00:0;}
enum State:uint8_t { BOOT=0,IDLE=1,PREFLIGHT=2,ARMED=3,MOVING=4,VERIFY_OFF=5,CAPTURING=6,FAULT=7 };
enum Flag:uint8_t { POLICY_LOCKED=1,SESSION_ACTIVE=2,TORQUE_MAY_BE_ON=4,CAPTURE_BUSY=8,ALL_TORQUE_OFF_VERIFIED=16,BOOT_READY=32,DIAG_READY=64,TORQUE_ENABLE_EVER_ATTEMPTED=128 };
enum Result:uint8_t { PENDING=0,OK=1,ERROR=2 };
enum Error:uint8_t { NONE=0,BAD_COMMAND=1,STALE=2,SESSION=3,BAD_ID=4,BUSY=5,FEEDBACK=6,MODE=7,POSITION=8,VOLTAGE=9,TEMPERATURE=10,RANGE=11,MOVE_TIMEOUT=12,TORQUE_OFF=13,UNSTABLE=14,INCOMPLETE=15,DIAG_ERROR=16,BOOT_ERROR=17,REPLAY=18 };
struct __attribute__((packed)) Command {
  char magic[4]; uint8_t version,op,targetId,flags;
  uint32_t bootNonce,requestSeq,session,sourceUs,ttlUs;
  int16_t deltaTicks; uint16_t reserved;
};
struct __attribute__((packed)) Row {
  uint16_t rawWord; int16_t signedSteps; uint16_t positionAgeMs,metadataAgeMs;
  uint8_t mode,torque,voltageRaw,temperature;
};
struct __attribute__((packed)) Snapshot {
  char magic[4]; uint8_t version,state,flags,activeId;
  uint32_t bootNonce,stateSeq,sampleUs,operationAckSeq,session,expiresUs,lastReceivedReqSeq;
  int16_t anchor,goal; uint16_t positionMask,metadataMask,stableMask,captureMask;
  uint8_t result,op,error,diagId; int16_t diagOffset; uint16_t diagMin,diagMax;
  Row rows[kCount];
};
struct __attribute__((packed)) ActionResult {
  char magic[4]; uint8_t version,op,targetId,result;
  uint32_t bootNonce,requestSeq,session,sampleUs;
  int16_t anchor,start,goal,reachedPosition,offPosition,deltaTicks;
  uint32_t reachedUs,offReadUs; uint8_t error,voltageRaw,temperature,reserved;
};
constexpr uint8_t HOME_RUN=9;
constexpr uint8_t HOME_CONTRACT_VERSION=5; // Adds native asymmetric ID30/31 HOME ranges; no wire-layout change.
constexpr uint32_t kHomeMotionUs=450000000, kHomeJointUs=30000000, kHomeStableUs=300000, kHomeTrackUs=1000000;
constexpr int16_t kHomeArrivalTicks=57, kHomeTrackTicks=57, kHomeHardErrorTicks=57;
constexpr uint32_t kHomeVelocityMilli=40000, kHomeAccelerationMilli=40000; // ticks/s and ticks/s^2 scaled by1000.
// Native user-zero q=-(position-target)*360/4096 deg. No modulo or zero rewrite.
static inline bool homeHead(uint8_t id){return id==30||id==31;}
static inline bool homePositionAllowed(uint8_t id,int16_t position,int16_t target){
  const int32_t delta=(int32_t)position-target;
  const int32_t q=-delta*360;
  if(id==30)return delta>=-682&&delta<=2047; // Return-only recovery, below half a revolution.
  if(id==31)return q>=-105*4096&&q<=105*4096;
  return delta>=-1024&&delta<=1024;
}
static inline uint32_t homeJointBudgetUs(uint8_t id,int16_t position,int16_t target){
  if(!homeHead(id))return kHomeJointUs;
  const int32_t delta=(int32_t)position-target;const uint32_t span=delta<0?-delta:delta;
  // 40ticks/s cruise +1s accel/brake allowance +0.6s stable windows +4.4s scan/tracking reserve.
  const uint32_t budget=span*25000u+6000000u;
  return budget<kHomeJointUs?kHomeJointUs:budget>60000000u?60000000u:budget;
}
enum HomeStage:uint8_t { HS_IDLE=0,HS_READ_MODE=1,HS_READ_TORQUE=2,HS_READ_POSITION=3,HS_PRIME_ECHO=4,HS_ENABLE_READBACK=5,HS_PRIME_SETTLE=6,HS_RAMP=7,HS_FINAL_SETTLE=8,HS_HOLD=9,HS_RELEASE=10 };
enum HomePhase:uint8_t { HOME_OFF=0,HOME_PREPARING=1,HOME_PREPARED=2,HOME_ENABLING=3,HOME_MOVING=4,HOME_HOLDING=5,HOME_RELEASING=6,HOME_DONE=7,HOME_FAULT=8 };
struct __attribute__((packed)) HomeCommand {
  char magic[4];uint8_t version,flags;uint16_t reserved;
  uint32_t bootNonce,requestSeq,session,sourceUs,ttlUs;
  uint8_t nativeSha256[32];int16_t targets[kCount];uint8_t modes[kCount],reservedTail[3];
};
struct __attribute__((packed)) HomeSnapshot {
  char magic[4];uint8_t version,phase,result,error;
  uint32_t bootNonce,stateSeq,sampleUs,operationAckSeq,session,expiresUs;
  uint8_t nativeSha256[32];
  uint16_t targetMask,freshMask,arrivedMask,offMask,primedMask,enabledMask,steps;
  uint8_t activeId,stage;
  int16_t targets[kCount],goals[kCount],positions[kCount];uint8_t modes[kCount],torques[kCount];
};
static_assert(sizeof(HomeCommand)==108,"DWH1 fixed native vector ABI");
static_assert(sizeof(HomeSnapshot)==200,"DHS1 fixed home evidence ABI");
static_assert(sizeof(Command)==32,"DWC1 command ABI");
static_assert(sizeof(Row)==12,"DWR1 row ABI");
static_assert(sizeof(Snapshot)==238,"DWR1 snapshot ABI");
static_assert(sizeof(ActionResult)==48,"DWA1 result ABI");
static inline int indexFor(uint8_t id) { for(uint8_t i=0;i<kCount;++i)if(kIds[i]==id)return i;return -1; }
static inline int16_t signedPosition(uint16_t raw) { int16_t n=(int16_t)(raw&0x7fff);return (raw&0x8000)?(int16_t)-n:n; }
static inline int16_t signedOffset(uint16_t raw) { int16_t n=(int16_t)(raw&0x7ff);return (raw&0x800)?(int16_t)-n:n; }
static inline uint32_t elapsed(uint32_t now,uint32_t before) { return now-before; }
static inline bool expired(uint32_t now,uint32_t deadline) { return (int32_t)(now-deadline)>=0; }
static inline int32_t magnitude(int32_t x) { return x<0?-x:x; }
}
