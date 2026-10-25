#pragma once
#include <stdint.h>

// Only invoked by the official control task while HOST_SERVO_DEBUG owns the bus.
// Bus delegates CalibrationOfs directly to the unmodified Feetech HLSCL SDK.
enum FactoryOp : uint8_t {
  FACTORY_CAPABILITIES = 0, FACTORY_READ = 1, FACTORY_CALIBRATE = 2,
  FACTORY_RELEASE = 3, FACTORY_OFF = 4, FACTORY_CALIBRATE_UNLOADED = 5
};
enum FactoryError : uint8_t {
  FACTORY_OK = 0, FACTORY_MODE = 1, FACTORY_ID = 2, FACTORY_READ_FAILED = 3,
  FACTORY_CAL_FAILED = 4, FACTORY_LOCK_FAILED = 5, FACTORY_GOAL_FAILED = 6,
  FACTORY_ENABLE_FAILED = 7, FACTORY_OFF_FAILED = 8
};
constexpr uint16_t FACTORY_CAPS = 0x7f; // also direct-offset unloaded calibration (no CAL instruction/goal)
struct __attribute__((packed)) FactoryPacket {
  char magic[4];
  uint8_t op, targetId;
  uint16_t reserved;
  uint32_t request;
};
struct __attribute__((packed)) FactoryReply {
  char magic[4];
  uint8_t op, targetId, error, reserved;
  uint32_t request;
  uint16_t position, zeroPos, offsetRaw, goal;
  uint8_t mode, torque, locked, padding;
  uint16_t heldMask, offMask, capabilities, reserved2;
};
static_assert(sizeof(FactoryPacket) == 12, "Factory command ABI");
static_assert(sizeof(FactoryReply) == 32, "Factory reply ABI");

template<class Bus> static bool factorySnapshot(Bus &bus, uint8_t id, FactoryReply &r) {
  const int position=bus.read(id,56,2), offset=bus.read(id,31,2), goal=bus.read(id,42,2);
  const int mode=bus.read(id,33,1), torque=bus.read(id,40,1), locked=bus.read(id,55,1);
  if(position<0 || position>4095 || offset<0 || goal<0 || mode<0 || torque<0 || locked<0) return false;
  r.position=position; r.offsetRaw=offset; r.goal=goal;
  r.mode=mode; r.torque=torque; r.locked=locked;
  return true;
}

template<class Bus> static void factoryOff(Bus &bus, const uint8_t *ids, unsigned count,
                                        uint16_t &held, FactoryReply &r) {
  bus.torque(254,0);
  held=0;
  r.offMask=0;
  for(unsigned i=0;i<count;++i) if(bus.read(ids[i],40,1)==0) r.offMask|=1u<<i;
  if(r.offMask != (1u<<count)-1) r.error=FACTORY_OFF_FAILED;
}

template<class Bus> static void factoryUnloaded(Bus &bus, uint8_t id,
                                              unsigned index, uint16_t &held, FactoryReply &r) {
  const uint16_t bit=1u<<index;
  if(!factorySnapshot(bus,id,r)) {r.error=FACTORY_READ_FAILED; return;}
  if(r.mode!=4) {r.error=FACTORY_MODE; return;}
  held &= ~bit;
  if(!bus.torque(id,0) || bus.read(id,40,1)!=0) {r.error=FACTORY_OFF_FAILED; return;}
  const int oldOffset=(r.offsetRaw & 0x8000) ? -int(r.offsetRaw & 0x7fff) : int(r.offsetRaw);
  const int newOffset=oldOffset+int(r.position)-2048;
  if(newOffset < -2047 || newOffset > 2047) {r.error=FACTORY_CAL_FAILED; return;}
  const uint16_t encoded=newOffset<0 ? uint16_t(0x8000u | -newOffset) : uint16_t(newOffset);
  if(encoded!=r.offsetRaw) {
    if(!bus.unlock(id) || bus.read(id,55,1)!=0 || bus.read(id,40,1)!=0) r.error=FACTORY_LOCK_FAILED;
    else if(!bus.offset(id,encoded)) r.error=FACTORY_CAL_FAILED;
    else {r.reserved |= 2; bus.settle();}
  }
  // No calibration instruction or goal write: those triggered an unexpected
  // torque enable in the real prototype. Only EEPROM offset is changed here.
  if(!bus.lock(id) || bus.read(id,55,1)!=1) r.error=FACTORY_LOCK_FAILED;
  if(!r.error && (!factorySnapshot(bus,id,r) || r.mode!=4 || r.torque!=0 || r.locked!=1 ||
                 r.offsetRaw!=encoded || r.position<2044 || r.position>2052)) r.error=FACTORY_READ_FAILED;
  if(!r.error) {r.zeroPos=r.position; r.reserved |= 1; r.offMask=bit;}
  else {bus.torque(id,0); if(bus.read(id,40,1)==0) r.offMask=bit;}
}

template<class Bus> static void factoryAxis(Bus &bus, uint8_t op, uint8_t id,
                                          unsigned index, uint16_t &held, FactoryReply &r) {
  const uint16_t bit=1u<<index;
  if(op==FACTORY_READ) {
    if(!factorySnapshot(bus,id,r)) r.error=FACTORY_READ_FAILED;
    return;
  }
  if(op==FACTORY_RELEASE) {
    const bool ack=bus.torque(id,0);
    if(!ack || bus.read(id,40,1)!=0) r.error=FACTORY_OFF_FAILED;
    else {held &= ~bit; r.offMask=bit;}
    if(!factorySnapshot(bus,id,r) && !r.error) r.error=FACTORY_READ_FAILED;
    return;
  }
  if(op==FACTORY_CALIBRATE_UNLOADED) {factoryUnloaded(bus,id,index,held,r); return;}
  // Read before any write; require the commissioned official mode, never change it.
  if(!factorySnapshot(bus,id,r)) {r.error=FACTORY_READ_FAILED; return;}
  if(r.mode!=4) {r.error=FACTORY_MODE; return;}
  held &= ~bit;
  if(!bus.torque(id,0) || bus.read(id,40,1)!=0) {r.error=FACTORY_OFF_FAILED; return;}
  // Manufacturer implementation disables this ID, unlocks EEPROM, then INST_CAL.
  const bool calibrated=bus.calibrationOfs(id);
  bus.settle();
  const bool locked=bus.lock(id);
  const int newPosition=bus.read(id,56,2);
  if(!calibrated) r.error=FACTORY_CAL_FAILED;
  else if(!locked || bus.read(id,55,1)!=1) r.error=FACTORY_LOCK_FAILED;
  else if(newPosition<0 || newPosition>4095) r.error=FACTORY_READ_FAILED;
  else {
    // The new encoder value is the target: no old-zero window, centering or ramp.
    r.zeroPos=newPosition;
    r.reserved |= 1; // MCU and host must retain the new zero even if hold later fails.
    if(!bus.goal(id,newPosition) || bus.read(id,42,2)!=newPosition) r.error=FACTORY_GOAL_FAILED;
    else if(!bus.torque(id,1) || bus.read(id,40,1)!=1) r.error=FACTORY_ENABLE_FAILED;
    else if(!factorySnapshot(bus,id,r) || r.mode!=4 || r.torque!=1 || r.locked!=1 || r.goal!=newPosition)
      r.error=FACTORY_READ_FAILED;
    else held |= bit;
  }
  // Failure releases only the selected axis. Previously held axes remain held.
  if(r.error) {bus.torque(id,0); if(bus.read(id,40,1)==0) r.offMask=bit;}
}
