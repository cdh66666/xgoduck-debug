#pragma once
#include <array>
#include <cstdint>
#include <vector>
inline uint32_t fakeUs=0;
inline uint32_t millis(){return fakeUs/1000;}
inline void delay(uint32_t ms){fakeUs+=ms*1000;}
inline void delayMicroseconds(uint32_t us){fakeUs+=us;}
struct FakeWire {
  std::array<uint8_t,256> registers{};
  std::array<uint8_t,12> injectedSample{};
  std::vector<uint8_t> tx,rx;
  uint8_t pointer=0;unsigned index=0,bursts=0,unlockedBursts=0;uint32_t lockedAt=0;
  bool available=true,locked=false,shortRead=false,commandStuck=false,ackStuck=false,ahbDisabled=false,injectSample=false;
  FakeWire(){registers[0]=5;registers[0x4D]=0x80;}
  void begin(){} void setClock(uint32_t){}
  void beginTransmission(uint8_t){tx.clear();}
  void write(uint8_t x){tx.push_back(x);}
  int endTransmission(bool=true){
    pointer=tx.at(0);
    if(tx.size()>1){
      registers[pointer]=tx[1];
      if(pointer==0x60){registers[0x4D]=0x80;locked=false;}
      if(pointer==0x0A&&tx[1]==0x12&&!commandStuck){registers[0x2D]|=0x80;ahbDisabled=registers[0x0B]==1;}
      if(pointer==0x0A&&tx[1]==0&&!ackStuck)registers[0x2D]&=0x7f;
    }
    return 0;
  }
  int requestFrom(int,int length){
    rx.clear();index=0;
    if(pointer==0x2D){
      if((registers[8]&0x80)&&available&&!locked){locked=true;lockedAt=fakeUs;}
      rx.push_back(registers[0x2D]|(available?1:0)|(locked&&fakeUs-lockedAt>=12?2:0));
    }else if(pointer==0x35&&length==12){
      ++bursts;if(!locked)++unlockedBursts;
      std::array<uint8_t,12> sample{};
      // An update between GX_L and GX_H mixes 0x00ff and 0x0100 into 0x01ff.
      sample[6]=0xff;sample[7]=locked&&ahbDisabled&&fakeUs-lockedAt>=12?0:1;
      if(injectSample)sample=injectedSample;
      rx.assign(sample.begin(),sample.end());
      if(shortRead)rx.pop_back();else locked=false;
    }else for(int i=0;i<length;++i)rx.push_back(registers[pointer+i]);
    return int(rx.size());
  }
  int read(){return rx.at(index++);}
};
inline FakeWire Wire;
