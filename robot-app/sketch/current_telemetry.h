#pragma once
#include <stdint.h>
#include <string.h>

// Diagnostic only: no current value is used in motion/PD/watchdog calculations.
struct __attribute__((packed)) CurrentPacket {
  char magic[4];
  uint8_t version, flags;
  uint16_t mask;
  uint32_t bootLow, bootHigh, seq, startUs, endUs, attempts, skipped;
  uint16_t raw[15], reserved;
  uint32_t requestUs;
};
static_assert(sizeof(CurrentPacket)==72,"Current telemetry tail v1 ABI");

struct CurrentSampler {
  static constexpr uint16_t ALL=0x7fff;
  static constexpr uint32_t WINDOW_US=6000, MIN_REMAINING_US=2500;
  bool attemptedWindow=false,pending=false;
  uint16_t mask=0,raw[15]={};
  uint32_t windowStartUs=0,startUs=0,endUs=0,requestUs=0,attempts=0,skipped=0;

  void beginWindow(uint32_t now) {
    attemptedWindow=pending=false;mask=0;memset(raw,0,sizeof(raw));
    windowStartUs=now;startUs=endUs=requestUs=0;
  }
  bool request(uint32_t now,uint16_t positionMask,uint32_t remainingUs,
               bool normalMode,bool voltagePending,bool voltageIssued) {
    if(attemptedWindow||pending||!normalMode||positionMask!=ALL||
       voltagePending||voltageIssued||remainingUs<MIN_REMAINING_US||
       (uint32_t)(now-windowStartUs)>WINDOW_US-MIN_REMAINING_US)return false;
    attemptedWindow=pending=true;startUs=now;++attempts;return true;
  }
  void sent(uint32_t elapsedUs) {requestUs=elapsedUs;}
  bool reply(uint8_t index,uint16_t value,uint8_t error,uint32_t now) {
    if(!pending||error||index>=15||(uint32_t)(now-windowStartUs)>=WINDOW_US||
       (mask&(uint16_t)(1u<<index)))return false;
    raw[index]=value;mask|=(uint16_t)(1u<<index);endUs=now;
    if(mask==ALL)pending=false;
    return true;
  }
  void endWindow() {
    if(!attemptedWindow)++skipped;
    pending=false;
  }
  void fill(CurrentPacket &packet,uint32_t bootLow,uint32_t bootHigh,uint32_t seq) const {
    memset(&packet,0,sizeof(packet));memcpy(packet.magic,"DQI1",4);
    packet.version=1;packet.flags=attemptedWindow?1:0;packet.mask=mask;
    packet.bootLow=bootLow;packet.bootHigh=bootHigh;packet.seq=seq;
    packet.startUs=startUs;packet.endUs=endUs;packet.attempts=attempts;packet.skipped=skipped;
    memcpy(packet.raw,raw,sizeof(raw));packet.requestUs=requestUs;
  }
};
