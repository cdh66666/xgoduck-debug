#include "Arduino.h"
#include "HLSCL.h"
#include "HLSCL_actual.inc"
#define __attribute__(x)
#pragma pack(push,1)
#include "factory_calibration.h"
#pragma pack(pop)
#include <iostream>
#include <stdexcept>
#include <string>
unsigned long fakeUs=0;
static unsigned checks=0;
static void expect(bool good) { ++checks; if(!good) throw std::runtime_error("assertion "+std::to_string(checks)); }
static std::vector<unsigned char> packet(unsigned char id,unsigned char value) {
  return {255,255,id,3,0,value,static_cast<unsigned char>(~(id+3+value))};
}
static void arm(HardwareSerial &serial,int prefix,int delay,unsigned char answerId=255,int stale=0) {
  serial.onPacket=[&,prefix,delay,answerId,stale](const auto &tx) {
    expect(tx[4]==2 && tx[5]==40 && tx[6]==1);
    const auto id=tx[2];
    std::vector<unsigned char> reply;
    if(stale==1) reply=packet(static_cast<unsigned char>(id+1),0);
    if(stale==2) reply={255,255,id,2,0,static_cast<unsigned char>(~(id+2))};
    reply.insert(reply.end(),static_cast<size_t>(prefix),255);
    auto valid=packet(answerId==255?id:answerId,0);
    reply.insert(reply.end(),valid.begin(),valid.end());
    serial.future={reply.begin(),reply.end()}; serial.readyUs=fakeUs+delay;
  };
}
static void armBytes(HardwareSerial &serial,const std::vector<unsigned char>&bytes,unsigned long gap=0) {
  serial.rx.clear();serial.future.clear();serial.fragments.clear();serial.endlessFf=false;
  serial.onPacket=[&,bytes,gap](const auto&) {
    unsigned long when=fakeUs+20;
    for(auto byte:bytes) {serial.fragments.emplace_back(when,byte);when+=gap;}
  };
}
static int run() {
  HardwareSerial serial; HLSCL sdk; sdk.pSerial=&serial; sdk.IOTimeOut=30;
  for(unsigned id=0;id<254;++id) {
    arm(serial,0,20); expect(sdk.readByte(static_cast<u8>(id),40)==0); expect(sdk.getLastError()==0);
  }
  arm(serial,1,20); int triple=sdk.readByte(10,40); auto tripleError=sdk.getLastError();
#ifdef SDK_CANDIDATE
  expect(triple==0 && tripleError==0);
#else
  expect(triple==-1 && tripleError==ERR_SLAVE_ID);
#endif
  serial.rx.clear(); arm(serial,0,20,255,1);
  expect(sdk.readByte(10,40)==-1 && sdk.getLastError()==ERR_SLAVE_ID);
  expect(!serial.rx.empty()); // the requested valid response is still buffered, unexamined
  serial.rx.clear(); arm(serial,0,20,255,2);
  expect(sdk.readByte(10,40)==-1 && sdk.getLastError()==ERR_BUFF_LEN);
  serial.rx.clear(); arm(serial,0,30000); expect(sdk.readByte(10,40)==0);
  arm(serial,0,32000); expect(sdk.readByte(10,40)==-1 && sdk.getLastError()==ERR_NO_REPLY);
  serial.future.clear(); serial.rx.clear();
  expect(serial.flushCalls==0); // actual SCSerial::wFlushSCS() never delegates flush
  // FF is legal in payload/checksum; exercise real Read across all byte values,
  // every commissioned ID and every redundant-FF count through the new budget.
  unsigned legalCases=0;
  for(unsigned id=0;id<254;++id) for(unsigned value=0;value<256;++value) for(unsigned extra=0;extra<=10;++extra) {
    auto bytes=packet(static_cast<u8>(id),static_cast<u8>(value));
    bytes.insert(bytes.begin(),extra,255);armBytes(serial,bytes);
    int actual=sdk.readByte(static_cast<u8>(id),40);
#ifdef SDK_CANDIDATE
    expect(actual==static_cast<int>(value) && sdk.getLastError()==0);
#else
    expect(extra ? actual==-1 && sdk.getLastError()==ERR_SLAVE_ID : actual==static_cast<int>(value) && sdk.getLastError()==0);
#endif
    ++legalCases;
  }
  // Continuous-ready FF input models an arbitrarily long FF run. Candidate must
  // reject while almost all bytes remain, rather than wait for the input to end.
  armBytes(serial,std::vector<unsigned char>(2048,255));
  auto started=fakeUs,reads=serial.readCalls;
  expect(sdk.readByte(10,40)==-1);
#ifdef SDK_CANDIDATE
  expect(sdk.getLastError()==ERR_NO_REPLY && serial.readCalls-reads==35);
#else
  expect(sdk.getLastError()==ERR_SLAVE_ID && serial.readCalls-reads==25);
#endif
  expect(fakeUs-started<100 && serial.rx.size()>2000);
  // Unlike a finite queue, this producer can return FF forever after the request.
  serial.rx.clear();serial.future.clear();serial.fragments.clear();
  serial.onPacket=[&](const auto&){serial.endlessFf=true;};
  started=fakeUs;reads=serial.readCalls;expect(sdk.readByte(10,40)==-1);
#ifdef SDK_CANDIDATE
  expect(sdk.getLastError()==ERR_NO_REPLY && serial.readCalls-reads==16 && serial.endlessBytes==15);
#else
  expect(sdk.getLastError()==ERR_SLAVE_ID && serial.readCalls-reads==6 && serial.endlessBytes==5);
#endif
  expect(fakeUs-started<100);serial.endlessFf=false;
  for(unsigned extra:{11u,12u,32u,1000u}) {
    auto bytes=packet(10,0);bytes.insert(bytes.begin(),extra,255);armBytes(serial,bytes);
    reads=serial.readCalls;expect(sdk.readByte(10,40)==-1);
#ifdef SDK_CANDIDATE
    expect(sdk.getLastError()==ERR_NO_REPLY && serial.readCalls-reads<=35);
#else
    expect(sdk.getLastError()==ERR_SLAVE_ID && serial.readCalls-reads<=25);
#endif
  }
  // Chunked arrival at 1ms byte gaps, including FF data and an FF checksum.
  for(unsigned extra:{0u,1u,10u}) for(unsigned value:{255u,243u}) {
    auto bytes=packet(10,static_cast<u8>(value));bytes.insert(bytes.begin(),extra,255);
    armBytes(serial,bytes,1000);int actual=sdk.readByte(10,40);
#ifdef SDK_CANDIDATE
    expect(actual==static_cast<int>(value) && sdk.getLastError()==0);
#else
    expect(extra?actual==-1 && sdk.getLastError()==ERR_SLAVE_ID:actual==static_cast<int>(value));
#endif
  }
  // Two-byte FF payload is not mistaken for redundant header.
  for(unsigned extra:{0u,1u,10u}) {
    std::vector<unsigned char> bytes={255,255,10,4,0,255,255,static_cast<u8>(~(10+4+255+255))};
    bytes.insert(bytes.begin(),extra,255);armBytes(serial,bytes,1000);
    int actual=sdk.readWord(10,40);
#ifdef SDK_CANDIDATE
    expect(actual==65535 && sdk.getLastError()==0);
#else
    expect(extra?actual==-1 && sdk.getLastError()==ERR_SLAVE_ID:actual==65535);
#endif
  }
  // Every truncated prefix of a legitimate response rejects without hanging.
  for(unsigned length=0;length<7;++length) {
    auto bytes=packet(10,255);bytes.resize(length);armBytes(serial,bytes,1000);
    started=fakeUs;expect(sdk.readByte(10,40)==-1 && sdk.getLastError()==ERR_NO_REPLY);
    expect(fakeUs-started<40000);
  }
  // Header recovery waiting on a missing shifted field also times out.
  armBytes(serial,{255,255,255,10,3},1000);started=fakeUs;
  expect(sdk.readByte(10,40)==-1);
#ifdef SDK_CANDIDATE
  expect(sdk.getLastError()==ERR_NO_REPLY && fakeUs-started<40000);
#else
  expect(sdk.getLastError()==ERR_SLAVE_ID);
#endif
  serial.rx.clear();serial.future.clear();serial.fragments.clear();
  // Exercise the actual factoryOff template with actual SDK reads and broadcast write.
  const uint8_t ids[]={10,11,12,13,14,20,21,22,23,24,30,31,32,33,34};
  struct Bus {
    HLSCL &sdk;
    bool torque(uint8_t id,uint8_t value) {return sdk.EnableTorque(id,value)==1;}
    int read(uint8_t id,uint8_t address,uint8_t) {int value=sdk.readByte(id,address);return sdk.getState()||sdk.getLastError()?-1:value;}
  } bus{sdk};
  serial.onPacket=[&](const auto &tx) {
    if(tx[2]==254) {expect(tx[4]==3 && tx[5]==40 && tx[6]==0); return;}
    auto bytes=packet(tx[2],0); if(tx[2]==10) bytes.insert(bytes.begin(),255);
    serial.future={bytes.begin(),bytes.end()};serial.readyUs=fakeUs+20;
  };
  uint16_t held=0;FactoryReply reply{};factoryOff(bus,ids,15,held,reply);
#ifdef SDK_CANDIDATE
  expect(reply.offMask==32767 && reply.error==0);
#else
  expect(reply.offMask==32766 && reply.error==FACTORY_OFF_FAILED);
#endif
  std::cout << "{\"checks\":"<<checks<<",\"legal_cases\":"<<legalCases<<",\"extra_header_budget\":10,\"triple_read\":"<<triple<<",\"triple_sdk_error\":"<<unsigned(tripleError)<<",\"off_mask\":"<<reply.offMask<<",\"factory_error\":"<<unsigned(reply.error)<<",\"flush_calls\":"<<serial.flushCalls<<"}\n";
  return 0;
}
int main() {try{return run();}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
