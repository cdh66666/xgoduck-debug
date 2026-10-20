#pragma once
#include <stdint.h>
// Isolated candidate, NOT wired into deployed firmware.
// Matches local Feetech HLSCL::ReadPos: bit15 direction + 15-bit magnitude.
static int16_t hostPositionSignMagnitude(uint8_t lo,uint8_t hi) {
  const uint16_t raw=(uint16_t)lo|((uint16_t)hi<<8);
  const int16_t magnitude=(int16_t)(raw&0x7fff);
  return raw&0x8000 ? -magnitude : magnitude;
}
