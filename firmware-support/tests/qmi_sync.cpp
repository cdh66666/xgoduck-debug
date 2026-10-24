#include <cassert>
#include <cstdio>
#include "../sketch/qmi8658.h"
int main(){
  qmiAddr=0x6a;uint8_t raw[12];
  assert(qmiRead(QMI8658C_REG_ACC_X_L,raw,12));
  assert(int16_t(raw[6]|raw[7]<<8)==511); // unlocked baseline reproduces the torn value
  Wire=FakeWire();fakeUs=0;
  assert(initImu());assert(Wire.ahbDisabled);assert(Wire.registers[8]==0x83);
  assert(updateImu());assert(imu.ok==1);assert(imu.gyroRaw[1]==-255);
  assert(Wire.unlockedBursts==0);
  const auto count=Wire.bursts;Wire.available=false;
  assert(!updateImu());assert(imu.ok==0);assert(Wire.bursts==count);
  Wire.available=true;Wire.shortRead=true;
  assert(!updateImu());assert(imu.ok==0);assert(Wire.locked);
  Wire.shortRead=false;assert(updateImu());assert(imu.gyroRaw[1]==-255);
  // A real saturated QMI sample must keep its direction through the body-axis
  // mapping. The old narrow cast returned -32768 on all three negated axes.
  Wire.injectSample=true;
  const int16_t values[6]={123,-32768,-456,-32768,789,-32768};
  for(unsigned i=0;i<6;++i){
    const uint16_t value=uint16_t(values[i]);
    Wire.injectedSample[2*i]=uint8_t(value);Wire.injectedSample[2*i+1]=uint8_t(value>>8);
  }
  assert(updateImu());
  assert(imu.accRaw[0]==-456&&imu.accRaw[1]==123&&imu.accRaw[2]==32767);
  assert(imu.gyroRaw[0]==32767&&imu.gyroRaw[1]==32767&&imu.gyroRaw[2]==789);
  for(int value=-32768;value<=32767;++value)
    assert(qmiNegateRaw(int16_t(value))==(value==-32768?32767:-value));
  Wire=FakeWire();Wire.commandStuck=true;fakeUs=0;
  assert(!initImu());assert(imu.ok==0);assert(qmiAddr==0);assert(fakeUs<100000);
  Wire=FakeWire();Wire.ackStuck=true;fakeUs=0;
  assert(!initImu());assert(imu.ok==0);assert(qmiAddr==0);assert(fakeUs<100000);
  std::puts("QMI sync: locking, recovery, handshake and saturated body-axis mapping PASS");
}
