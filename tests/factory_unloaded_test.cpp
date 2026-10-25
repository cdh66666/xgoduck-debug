#include <cassert>
#include <cstdlib>
#include <iostream>
#include "../robot-app/sketch/factory_calibration.h"

struct FakeBus {
  int position, offsetRaw, goalValue=73, modeValue=4, torqueValue=0, lockedValue=1;
  int enables=0, goals=0, calibrations=0, offsets=0;
  bool failOffset=false;
  static int decode(int raw) {return raw&0x8000 ? -(raw&0x7fff) : raw;}
  int read(uint8_t,uint8_t address,uint8_t) {
    switch(address) {
      case 56:return position; case 31:return offsetRaw; case 42:return goalValue;
      case 33:return modeValue; case 40:return torqueValue; case 55:return lockedValue;
      default:return -1;
    }
  }
  bool torque(uint8_t,uint8_t value) {if(value) ++enables;torqueValue=value;return true;}
  bool calibrationOfs(uint8_t) {++calibrations;position=2048;torqueValue=1;return true;}
  bool goal(uint8_t,uint16_t value) {++goals;goalValue=value;return true;}
  bool lock(uint8_t) {lockedValue=1;return true;}
  bool unlock(uint8_t) {lockedValue=0;return true;}
  bool offset(uint8_t,uint16_t value) {
    assert(torqueValue==0 && lockedValue==0);
    if(failOffset)return false;
    ++offsets;position+=decode(offsetRaw)-decode(value);offsetRaw=value;return true;
  }
  void settle() {}
};

void check(int position,int offset,int expectedOffset) {
  FakeBus bus{position,offset};uint16_t held=0;FactoryReply reply={};
  factoryAxis(bus,FACTORY_CALIBRATE_UNLOADED,23,8,held,reply);
  assert(reply.error==FACTORY_OK && reply.position==2048 && reply.zeroPos==2048);
  assert(reply.torque==0 && reply.locked==1 && reply.goal==73);
  assert(reply.offMask==(1u<<8) && held==0 && (reply.reserved&1));
  assert(FakeBus::decode(reply.offsetRaw)==expectedOffset);
  assert(bus.enables==0 && bus.goals==0 && bus.calibrations==0);
}

int main() {
  check(3613,50,1615);               // Reinstalled right knee, measured data.
  check(2057,1375,1384);             // Positive EEPROM offset.
  check(2039,0x8000|144,-153);       // Negative EEPROM offset.
  check(2048,0x8000|633,-633);       // First prototype axis: already centered.
  FakeBus unchanged{2048,99};uint16_t held=0;FactoryReply r={};
  factoryAxis(unchanged,FACTORY_CALIBRATE_UNLOADED,34,14,held,r);
  assert(unchanged.offsets==0 && !(r.reserved&2));
  FakeBus failure{2060,50};failure.failOffset=true;r={};
  factoryAxis(failure,FACTORY_CALIBRATE_UNLOADED,23,8,held,r);
  assert(r.error!=FACTORY_OK && failure.torqueValue==0 && failure.lockedValue==1);
  assert(failure.enables==0 && failure.goals==0 && failure.calibrations==0);
  FakeBus range{4000,2000};r={};
  factoryAxis(range,FACTORY_CALIBRATE_UNLOADED,23,8,held,r);
  assert(r.error==FACTORY_CAL_FAILED && range.offsets==0 && range.enables==0);
  std::cout<<"PASS: direct-offset calibration never enables torque, writes a goal, or invokes CAL; signed offsets, idempotence and failures verified\n";
}
