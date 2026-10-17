#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Integrated into the commissioning sketch source; deployment and real UART
// behavior require separate Uno Q build and hardware verification.
// The control thread must be the sole owner of the port. Suspend its usual
// sync polling while ownsBus() is true. Send torque-off before begin(), then
// do not insert any UART write into a probe's receive window. This engine
// sends only fixed ID10 PING / READ / SYNC_READ packets; no register writes.
// Port adapter contract: available(), read(), write(data,size), flush(), nowUs().
// flush() must wait for the final stop bit; nowUs() is an unsigned 32-bit clock.
namespace duck_probe {

constexpr uint8_t kTargetId = 10;
constexpr uint32_t kBaud = 1000000;
constexpr uint32_t kQuietUs = 2000;
constexpr uint32_t kQuietTimeoutUs = 50000;
constexpr size_t kWireSize = 96;
constexpr size_t kRawPrefixSize = 32;
constexpr size_t kDrainBudget = 256;

enum ReportFlags : uint8_t {
  REPORT_FINISHED = 1u,
  SHORT_TX = 2u,
  RAW_PREFIX_TRUNCATED = 4u,
  QUIET_TIMEOUT = 8u
};

struct Report {
  uint8_t flags, opcode, targetId, attempt, address, dataLength, txRequested;
  uint16_t txCount, rxCount, drainedCount, validFrames, crcErrors;
  uint16_t deviceErrorFrames, knownFrames, unknownFrames, echoes;
  uint16_t unexpectedLengthFrames, validReadFrames, knownIdMask;
  uint32_t firstRxUs, lastRxUs, windowUs, elapsedUs;
  uint8_t lastFrameId, lastStatus, lastLength, rawPrefixLength;
  uint16_t rawPosition, rawSpeed;
  bool positionValid;
  uint8_t lastUnknownId, rawPrefix[kRawPrefixSize];
};

struct StepResult {
  bool ownsBus, reportReady, done;
  // Fresh only for this step. Neither PING nor cached values set this bit.
  uint16_t freshFeedbackMask, rawPosition, rawSpeed;
};

inline int knownIdIndex(uint8_t id) {
  if (id >= 10 && id <= 14) return id - 10;
  if (id >= 20 && id <= 24) return id - 20 + 5;
  if (id >= 30 && id <= 34) return id - 30 + 10;
  return -1;
}

inline void bump(uint16_t& value) { if (value != UINT16_MAX) ++value; }
inline void put16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value); out[1] = static_cast<uint8_t>(value >> 8);
}
inline void put32(uint8_t* out, uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
}

// Explicit little-endian encoding avoids compiler packing / bool ABI issues.
// This is a separate DQP1 report, not a change to the 240-byte DQS1 State ABI.
inline void encodeReport(const Report& r, uint8_t out[kWireSize]) {
  memset(out, 0, kWireSize);
  memcpy(out, "DQP1", 4); out[4] = 1; out[5] = r.flags;
  out[6] = r.opcode; out[7] = r.targetId; out[8] = r.attempt;
  out[9] = r.address; out[10] = r.dataLength; out[11] = r.txRequested;
  const uint16_t counts[12] = {r.txCount, r.rxCount, r.drainedCount, r.validFrames,
    r.crcErrors, r.deviceErrorFrames, r.knownFrames, r.unknownFrames, r.echoes,
    r.unexpectedLengthFrames, r.validReadFrames, r.knownIdMask};
  for (unsigned i = 0; i < 12; ++i) put16(out + 12 + i * 2, counts[i]);
  put32(out + 36, r.firstRxUs); put32(out + 40, r.lastRxUs);
  put32(out + 44, r.windowUs); put32(out + 48, r.elapsedUs);
  out[52] = r.lastFrameId; out[53] = r.lastStatus; out[54] = r.lastLength;
  out[55] = r.rawPrefixLength; put16(out + 56, r.rawPosition);
  put16(out + 58, r.rawSpeed); out[60] = r.positionValid ? 1 : 0;
  out[61] = r.lastUnknownId;
  memcpy(out + 64, r.rawPrefix, kRawPrefixSize);
}

class Engine {
 public:
  Engine() : phase_(IDLE), query_(0), attempt_(1), used_(0), txSize_(0),
    quietStarted_(false), quietStart_(0), quietSince_(0), captureStart_(0),
    drained_(0), windowUs_(20000), aborted_(false), freshMask_(0),
    freshPosition_(0), freshSpeed_(0) { memset(&report_, 0, sizeof(report_)); }

  bool begin(uint32_t windowUs = 20000) {
    if (ownsBus() || (windowUs != 20000 && windowUs != 50000)) return false;
    windowUs_ = windowUs; query_ = 0; attempt_ = 1; aborted_ = false;
    phase_ = QUIET; quietStarted_ = false; drained_ = 0; used_ = 0;
    freshMask_ = 0; return true;
  }
  void cancel() { phase_ = FINISHED; used_ = 0; freshMask_ = 0; aborted_ = true; }
  bool ownsBus() const { return phase_ == QUIET || phase_ == CAPTURE || phase_ == READY; }
  bool reportReady() const { return phase_ == READY; }
  bool done() const { return phase_ == FINISHED; }

  bool takeReport(Report& out) {
    if (phase_ != READY) return false;
    out = report_;
    if (aborted_ || (query_ == 2 && attempt_ == 3)) {
      phase_ = FINISHED;
    } else {
      if (++attempt_ > 3) { attempt_ = 1; ++query_; }
      phase_ = QUIET; quietStarted_ = false; drained_ = 0; used_ = 0;
    }
    return true;
  }

  template <typename Port> StepResult step(Port& port) {
    freshMask_ = 0;
    if (phase_ == QUIET) {
      const uint32_t now = port.nowUs();
      if (!quietStarted_) {
        quietStarted_ = true; quietStart_ = quietSince_ = now;
      }
      unsigned consumed = 0;
      while (port.available() > 0 && consumed++ < kDrainBudget) {
        if (port.read() < 0) break;
        bump(drained_); quietSince_ = port.nowUs();
      }
      if (static_cast<uint32_t>(port.nowUs() - quietStart_) >= kQuietTimeoutUs) {
        resetReport(); report_.flags = REPORT_FINISHED | QUIET_TIMEOUT;
        report_.elapsedUs = port.nowUs() - quietStart_;
        aborted_ = true; phase_ = READY;
      } else if (port.available() == 0 &&
                 static_cast<uint32_t>(port.nowUs() - quietSince_) >= kQuietUs) {
        resetReport(); used_ = 0;
        const size_t written = port.write(tx_, txSize_);
        report_.txCount = static_cast<uint16_t>(written > UINT16_MAX ? UINT16_MAX : written);
        if (written != txSize_) { report_.flags |= SHORT_TX; aborted_ = true; }
        port.flush(); captureStart_ = port.nowUs(); phase_ = CAPTURE;
      }
    }
    if (phase_ == CAPTURE) {
      unsigned consumed = 0;
      while (port.available() > 0 && consumed++ < kDrainBudget) {
        const int value = port.read();
        if (value < 0) break;
        const uint32_t elapsed = port.nowUs() - captureStart_;
        bump(report_.rxCount);
        if (report_.firstRxUs == UINT32_MAX) report_.firstRxUs = elapsed;
        report_.lastRxUs = elapsed;
        if (report_.rawPrefixLength < kRawPrefixSize)
          report_.rawPrefix[report_.rawPrefixLength++] = static_cast<uint8_t>(value);
        else report_.flags |= RAW_PREFIX_TRUNCATED;
        consume(static_cast<uint8_t>(value));
      }
      report_.elapsedUs = port.nowUs() - captureStart_;
      if (report_.elapsedUs >= windowUs_) {
        report_.flags |= REPORT_FINISHED; phase_ = READY;
      }
    }
    return {ownsBus(), reportReady(), done(), freshMask_, freshPosition_, freshSpeed_};
  }

 private:
  enum Phase : uint8_t { IDLE, QUIET, CAPTURE, READY, FINISHED };
  Phase phase_;
  uint8_t query_, attempt_, used_, txSize_, tx_[9], frame_[32];
  bool quietStarted_;
  uint32_t quietStart_, quietSince_, captureStart_;
  uint16_t drained_;
  uint32_t windowUs_;
  bool aborted_;
  Report report_;
  uint16_t freshMask_, freshPosition_, freshSpeed_;

  void resetReport() {
    memset(&report_, 0, sizeof(report_));
    tx_[0] = tx_[1] = 0xFF;
    if (query_ == 0) {
      tx_[2] = kTargetId; tx_[3] = 2; tx_[4] = 0x01; txSize_ = 6;
      report_.address = 0xFF;
    } else if (query_ == 1) {
      tx_[2] = kTargetId; tx_[3] = 4; tx_[4] = 0x02;
      tx_[5] = 56; tx_[6] = 6; txSize_ = 8;
      report_.address = 56; report_.dataLength = 6;
    } else {
      tx_[2] = 0xFE; tx_[3] = 5; tx_[4] = 0x82;
      tx_[5] = 56; tx_[6] = 6; tx_[7] = kTargetId; txSize_ = 9;
      report_.address = 56; report_.dataLength = 6;
    }
    uint8_t sum = 0;
    for (unsigned i = 2; i + 1u < txSize_; ++i) sum = static_cast<uint8_t>(sum + tx_[i]);
    tx_[txSize_ - 1] = static_cast<uint8_t>(~sum);
    report_.opcode = tx_[4]; report_.targetId = kTargetId; report_.attempt = attempt_;
    report_.txRequested = txSize_; report_.drainedCount = drained_;
    report_.firstRxUs = report_.lastRxUs = UINT32_MAX;
    report_.windowUs = windowUs_;
    report_.lastFrameId = report_.lastStatus = report_.lastUnknownId = 0xFF;
  }

  void drop(unsigned count) {
    if (count >= used_) { used_ = 0; return; }
    used_ = static_cast<uint8_t>(used_ - count);
    memmove(frame_, frame_ + count, used_);
  }
  void consume(uint8_t value) {
    if (used_ == sizeof(frame_)) drop(1);
    frame_[used_++] = value;
    while (used_ >= 2) {
      if (frame_[0] != 0xFF || frame_[1] != 0xFF) { drop(1); continue; }
      if (used_ < 3) return;
      if (frame_[2] == 0xFF) { drop(1); continue; }
      if (used_ < 4) return;
      const uint8_t length = frame_[3];
      if (length < 2 || length > 28) { bump(report_.unexpectedLengthFrames); drop(1); continue; }
      const unsigned total = length + 4u;
      if (used_ < total) return;
      uint8_t sum = 0;
      for (unsigned i = 2; i < total - 1; ++i) sum = static_cast<uint8_t>(sum + frame_[i]);
      if (static_cast<uint8_t>(~sum) != frame_[total - 1]) {
        bump(report_.crcErrors); drop(1); continue;
      }
      if (total == txSize_ && memcmp(frame_, tx_, txSize_) == 0) {
        bump(report_.echoes); drop(total); continue;
      }
      bump(report_.validFrames);
      const uint8_t id = frame_[2], status = frame_[4];
      const int index = knownIdIndex(id);
      report_.lastFrameId = id; report_.lastStatus = status; report_.lastLength = length;
      if (index >= 0) {
        bump(report_.knownFrames); report_.knownIdMask |= static_cast<uint16_t>(1u << index);
      } else { bump(report_.unknownFrames); report_.lastUnknownId = id; }
      if (status != 0) bump(report_.deviceErrorFrames);
      const uint8_t wantedLength = query_ == 0 ? 2 : 8;
      if (length != wantedLength) bump(report_.unexpectedLengthFrames);
      if (query_ != 0 && !aborted_ && id == kTargetId && status == 0 && length == 8) {
        bump(report_.validReadFrames); report_.positionValid = true;
        report_.rawPosition = static_cast<uint16_t>(frame_[5] | (uint16_t(frame_[6]) << 8));
        report_.rawSpeed = static_cast<uint16_t>(frame_[7] | (uint16_t(frame_[8]) << 8));
        freshMask_ |= static_cast<uint16_t>(1u << knownIdIndex(kTargetId));
        freshPosition_ = report_.rawPosition; freshSpeed_ = report_.rawSpeed;
      }
      drop(total);
    }
  }
};
}  // namespace duck_probe
