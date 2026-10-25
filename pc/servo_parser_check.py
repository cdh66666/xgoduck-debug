"""Generate a host C++ regression harness using the actual firmware parser body.

Usage: python pc/servo_parser_check.py robot-app/sketch/scs_bus.h output.cpp
Then compile and run output.cpp with a C++17 compiler. No robot connection.
"""
from pathlib import Path
import sys

def harness(source):
    begin=source.index('static void parseServoRx() {')
    level=0;end=None
    for i in range(source.index('{',begin),len(source)):
        level+=(source[i]=='{')-(source[i]=='}')
        if level==0:end=i+1;break
    body=source[begin:end]
    return r'''
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>
#include <iostream>
#include <cassert>
struct Serial {std::deque<uint8_t> q;bool available(){return !q.empty();}int read(){int v=q.front();q.pop_front();return v;}} Serial1;
static uint8_t servoParserFlag=0,servoRxBuffer[32]={},servoRxLen=0,servoRxDataLen=0;
static uint8_t servoPingId=255,servoReadId=255,servoParamsId=255;
static bool servoPingSeen=false,servoReadSeen=false,servoParamsSeen=false,servoBootPdReading=false;
static uint16_t servoReadPos=0,servoPdMask=0;
static uint8_t servoParamKp=0,servoParamKd=0,servoBootP[15]={},servoBootD[15]={};
struct Observer {bool pending=false;template<class... T>void reply(T...){}} displayVoltage,currentSampler;
struct Health {uint32_t bytes=0,crcErrors=0,lengthErrors=0,frames[15]={},faults[15]={};uint8_t lastError[15]={},badFrame[16]={},badLen=0;} servoHealth;
static uint8_t servoTickHead[32]={},servoTickHeadLen=0;
static uint32_t micros(){return 100;}
static int motorIndexById(uint8_t id){return id>=10&&id<=14?id-10:id>=20&&id<=24?id-15:id>=30&&id<=34?id-20:-1;}
static int16_t hostWord(uint8_t a,uint8_t b){return int16_t(a|(b<<8));}
static int32_t hostVelSignMag(uint8_t a,uint8_t b){uint16_t v=a|(b<<8);return (v&32768)?-int32_t(v&32767):v;}
static std::vector<int> accepted;
static void applyMotorFb(uint8_t id,int16_t,int32_t){accepted.push_back(id);}
''' + body + r'''
static std::vector<uint8_t> frame(uint8_t id,uint8_t error=0) {
  std::vector<uint8_t> b={255,255,id,8,error,0,8,0,0,0,0,0};
  uint8_t c=0;for(size_t i=2;i+1<b.size();++i)c+=b[i];b.back()=~c;return b;
}
static void feed(const std::vector<uint8_t> &b) {
  // One byte at a time also checks state retention across fragmented reads.
  for(auto v:b){Serial1.q.push_back(v);parseServoRx();}
}
static void reset(){servoParserFlag=servoRxLen=servoRxDataLen=servoTickHeadLen=0;accepted.clear();}
int main() {
  reset();feed(frame(10));feed(frame(11));assert((accepted==std::vector<int>{10,11}));
  reset();auto b=frame(10);b.insert(b.begin(),255);feed(b);feed(frame(11));
  if(accepted!=std::vector<int>{10,11}) {std::cout<<"FAIL repeated FF header lost ID10 while ID11 survives\n";return 2;}
  reset();b=frame(10);b.back()^=1;feed(b);feed(frame(11));assert((accepted==std::vector<int>{11}));
  reset();feed(frame(10,32));feed(frame(11));assert((accepted==std::vector<int>{11}));
  std::cout<<"PASS actual parser: fragmented replies, repeated FF, checksum rejection, device-fault rejection\n";
}
'''

if __name__=='__main__':
    source,output=map(Path,sys.argv[1:3])
    output.write_text(harness(source.read_text(encoding='utf8')),encoding='utf8')
