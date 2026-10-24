#pragma once
#include <stdint.h>

// The head cable permits only the forward 180 degrees about the saved zero.
// Enforce at the final bus boundary, including raw calibration/debug goals.
namespace DuckLimits {
constexpr uint8_t HEAD_YAW_ID = 32;
constexpr float HEAD_YAW_DEG = 90.0f;
constexpr int32_t HEAD_YAW_STEPS = 1023; // floor(90/360*4095), stays inside 90 deg.

static inline float angle(uint8_t id, float requested) {
  if (id != HEAD_YAW_ID) return requested;
  if (requested < -HEAD_YAW_DEG) return -HEAD_YAW_DEG;
  if (requested > HEAD_YAW_DEG) return HEAD_YAW_DEG;
  return requested;
}

static inline uint16_t position(uint8_t id, int32_t requested, uint16_t zero) {
  int32_t low = 0, high = 4095;
  if (id == HEAD_YAW_ID) {
    if (int32_t(zero)-HEAD_YAW_STEPS > low) low = int32_t(zero)-HEAD_YAW_STEPS;
    if (int32_t(zero)+HEAD_YAW_STEPS < high) high = int32_t(zero)+HEAD_YAW_STEPS;
  }
  if (requested < low) requested = low;
  if (requested > high) requested = high;
  return uint16_t(requested);
}
}
