#include <array>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <vector>
using Bytes=std::vector<uint8_t>;
struct Snapshot {
  uint16_t mask=0,pdmask=0,pos=0;
  bool ping=false,read=false;
  uint8_t flag=0,len=0,dataLen=0;
  std::array<std::tuple<uint16_t,float,float>,15> motor;
  std::array<std::array<uint8_t,2>,15> pd;
  std::vector<std::array<uint32_t,4>> voltage,current;
  bool operator==(const Snapshot& b) const { return std::tie(mask,pdmask,pos,ping,read,flag,len,dataLen,motor,pd,voltage,current)==std::tie(b.mask,b.pdmask,b.pos,b.ping,b.read,b.flag,b.len,b.dataLen,b.motor,b.pd,b.voltage,b.current); }
};
namespace original {
#include "fixture_base.inc"
#include "original_actual.inc"
#include "fixture_probe.inc"
}
namespace candidate {
#include "fixture_base.inc"
#include "candidate_actual.inc"
#include "fixture_probe.inc"
}
static uint64_t checks=0,equalities=0;
static void require(bool condition,const char* message) { checks++; if(!condition) throw std::runtime_error(message); }
static Bytes frame(int id,int length,uint8_t fill=0x35,uint8_t error=0) {
  Bytes b={255,255,(uint8_t)id,(uint8_t)length,error};
  for(int i=0;i<length-2;i++) b.push_back((uint8_t)(fill+i));
  uint8_t sum=0; for(size_t i=2;i<b.size();i++) sum=(uint8_t)(sum+b[i]);
  b.push_back((uint8_t)~sum); return b;
}
static void append(Bytes& a,const Bytes& b) { a.insert(a.end(),b.begin(),b.end()); }
static void validateAccepted(const Snapshot& s,int length,bool pending,uint8_t error) {
  if(length==2) require(s.ping,"ping not accepted");
  if(length==3) require(s.voltage.size()==1,"voltage dispatch not accepted");
  if(length==4 && pending) require(s.current.size()==1,"current dispatch not accepted");
  if(length==4 && !pending) require(s.pdmask==(error?0:1),"boot PD acceptance changed");
  if(length==8) require(s.mask==(error?0:1) && s.read==!error,"position acceptance changed");
  if(length==9 || length==11) require(s.mask==0 && !s.read && !s.ping && s.voltage.empty() && s.current.empty(),"ignored length changed");
}
static Snapshot equalRun(const Bytes& bytes,const std::vector<size_t>& chunks,int id,int length,bool pending=false) {
  auto a=original::run(bytes,chunks,id,length,pending), b=candidate::run(bytes,chunks,id,length,pending);
  require(a==b,"original/candidate differ for ordinary stream"); equalities++; return b;
}
int main() {
  try {
    // Every legal unicast ID, all accepted framing lengths, both PD/current routes,
    // error flags and every payload seed (including FF) with full and byte chunks.
    for(int id=0;id<=253;id++) for(int length:{2,3,4,8,9,11}) for(int pending=0;pending<=(length==4?1:0);pending++) {
      for(int fill=0;fill<256;fill++) for(uint8_t error:{(uint8_t)0,(uint8_t)4}) {
        Bytes b=frame(id,length,(uint8_t)fill,error);
        for(bool bytewise:{false,true}) {
          std::vector<size_t> chunks=bytewise?std::vector<size_t>(b.size(),1):std::vector<size_t>{b.size()};
          validateAccepted(equalRun(b,chunks,id,length,pending!=0),length,pending!=0,error);
        }
      }
      Bytes b=frame(id,length);
      for(size_t split=0;split<=b.size();split++) equalRun(b,{split,b.size()-split},id,length,pending!=0);
      Bytes corrupt=b; corrupt.back()^=1;
      auto bad=equalRun(corrupt,std::vector<size_t>(corrupt.size(),1),id,length,pending!=0);
      require(bad.mask==0 && bad.pdmask==0 && !bad.ping && !bad.read && bad.voltage.empty() && bad.current.empty(),"bad checksum accepted");
      Bytes junk={0x13,0xFE,0x00,0xFF,0x07,0x23}; append(junk,b);
      validateAccepted(equalRun(junk,std::vector<size_t>(junk.size(),1),id,length,pending!=0),length,pending!=0,0);
      Bytes recovery=corrupt; append(recovery,b);
      validateAccepted(equalRun(recovery,{recovery.size()},id,length,pending!=0),length,pending!=0,0);
    }
    const uint8_t ids[15]={10,11,12,13,14,20,21,22,23,24,30,31,32,33,34};
    Bytes burst; for(uint8_t id:ids) append(burst,frame(id,8));
    auto good=equalRun(burst,{burst.size()},-1,8);
    require(good.mask==32767,"legal 15-axis burst incomplete");
    auto fragmented=equalRun(burst,std::vector<size_t>(burst.size(),1),-1,8);
    require(fragmented.mask==32767,"fragmented 15-axis burst incomplete");
    // Checksum FF immediately followed by the next FF FF header is legal and unchanged.
    Bytes checksumFF=frame(10,8); checksumFF[9]=(uint8_t)(checksumFF[9]+checksumFF.back()+1);
    uint8_t sum=0; for(size_t i=2;i+1<checksumFF.size();i++) sum=(uint8_t)(sum+checksumFF[i]);
    checksumFF.back()=(uint8_t)~sum; require(checksumFF.back()==255,"checksum FF fixture invalid");
    append(checksumFF,frame(11,8)); require(equalRun(checksumFF,std::vector<size_t>(checksumFF.size(),1),-1,8).mask==3,"boundary three FF changed");
    uint16_t tripleOriginal=0,tripleCandidate=0;
    for(int extra=1;extra<=8;extra++) {
      Bytes noisy(extra,255); append(noisy,burst);
      auto byteA=original::run(noisy,std::vector<size_t>(noisy.size(),1),-1,8);
      auto byteB=candidate::run(noisy,std::vector<size_t>(noisy.size(),1),-1,8);
      require(byteB.mask==32767,"candidate bytewise extra-FF recovery incomplete");
      if(extra==1) require(byteA.mask==32766,"original bytewise three-FF weakness not reproduced");
      for(size_t split=0;split<=noisy.size();split++) {
        auto a=original::run(noisy,{split,noisy.size()-split},-1,8);
        auto b=candidate::run(noisy,{split,noisy.size()-split},-1,8);
        require(b.mask==32767,"candidate extra-FF recovery incomplete");
        if(extra==1) { require(a.mask==32766,"original three-FF weakness not reproduced"); tripleOriginal=a.mask; tripleCandidate=b.mask; }
      }
    }
    std::cout<<"{\"passed\":true,\"checks\":"<<checks<<",\"ordinary_stream_equalities\":"<<equalities<<",\"legal_ids\":254,\"lengths\":[2,3,4,8,9,11],\"triple_ff_original_mask\":"<<tripleOriginal<<",\"triple_ff_candidate_mask\":"<<tripleCandidate<<",\"original_id10_missing\":true,\"following_14_original_valid\":true,\"candidate_all_15_valid\":true,\"physical_cause_proven\":false,\"deployed\":false}\n";
  } catch(const std::exception& error) { std::cerr<<error.what()<<"\n"; return 1; }
}
