#include <cmath>
#include <iostream>
#include <stdexcept>
#include "../sketch/joint_limits.h"

int main() {
  try {
    unsigned checks=0;
    auto require=[&](bool ok){++checks;if(!ok)throw std::runtime_error("head cable limit failed");};
    // Actual get-up outputs recorded before the three head servos stopped replying.
    for(float q:{-144.3607178f,94.8172836f,-90.f,0.f,90.f}) {
      const float bounded=DuckLimits::angle(32,q);
      require(bounded>=-90.f && bounded<=90.f);
      require(DuckLimits::angle(31,q)==q);
    }
    for(uint16_t zero:{uint16_t(0),uint16_t(900),uint16_t(2048),uint16_t(3500),uint16_t(4095)}) {
      for(int32_t raw=-1;raw<=4096;++raw) {
        const auto p=DuckLimits::position(32,raw,zero);
        require(p<=4095 && std::abs(int32_t(p)-zero)<=1023);
        if(raw>=0 && raw<=4095) require(DuckLimits::position(31,raw,zero)==raw);
      }
    }
    require(DuckLimits::position(32,2048,2048)==2048);
    std::cout<<"{\"passed\":true,\"checks\":"<<checks<<",\"head_yaw_id\":32,\"range_deg\":[-90,90],\"hardware_access\":false}\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<"\n";return 1;}
}
