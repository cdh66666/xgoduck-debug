#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include "position_decode_candidate.h"
// Current production helper, copied exactly; retained to prove the discrepancy.
static int16_t hostWord(uint8_t lo, uint8_t hi) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}
int main() {
  const uint16_t input[]={0x0000,0x07ff,0x0fff,0x8000,0x8001,0x9000,0xffff};
  const int16_t golden[]={0,2047,4095,0,-1,-4096,-32767};
  int disagreements=0;
  for(int i=0;i<7;++i){
    const uint8_t lo=input[i]&255,hi=input[i]>>8;
    assert(hostPositionSignMagnitude(lo,hi)==golden[i]);
    if(hostWord(lo,hi)!=golden[i])++disagreements;
  }
  // No difference anywhere in the commissioned single-turn positive domain.
  for(int i=0;i<=4095;++i)assert(hostPositionSignMagnitude(i&255,i>>8)==hostWord(i&255,i>>8));
  assert(disagreements==4);
  puts("7 manufacturer position goldens and all 4096 single-turn values passed; legacy mismatch in 4 negative encodings. Candidate only.");
}
