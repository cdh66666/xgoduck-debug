#pragma once
#include <stdint.h>

// Isolated candidate integrated with patched Arduino_RouterBridge 0.4.3.
// Required Port adapter contract:
//   bool handlersBound(): both local callbacks were bound successfully once;
//   bool registerOnly(const char*, uint32_t deadlineMs): router registration only,
//     returns by deadline, retires its timed-out/late RPC responses, never binds a
//     handler, sends a servo command, or holds a control/report lock while waiting.
// libraries/Arduino_RouterBridge/src/bridge.h implements that contract. The
// original Bridge.call().result() does not and must not replace registerOnly().
namespace duck_routes {
class Refresh {
 public:
  static constexpr uint32_t kPeriodMs = 1000;
  static constexpr uint32_t kAttemptDeadlineMs = 200;
  uint32_t attempts() const { return attempts_; }
  uint32_t failures() const { return failures_; }

  // Call only from one dedicated low-priority worker (suggested priority 7).
  // The 100 Hz control/report tasks never call or wait on this function.
  template<class Port> bool tick(uint32_t nowMs, Port& port) {
    // Connectivity must recover even with missing servo power/feedback, FAULT
    // or an active transaction. Registering a route grants no motion authority
    // and is also needed for an explicitly requested STOP to reach the MCU.
    if (!port.handlersBound()) return false;
    if (haveAttempt_ && uint32_t(nowMs - lastAttemptMs_) < kPeriodMs) return false;
    haveAttempt_ = true;
    lastAttemptMs_ = nowMs;
    static const char* const names[] = {"duck_command", "duck_calibration"};
    for (const char* name : names) {
      ++attempts_;
      if (!port.registerOnly(name, kAttemptDeadlineMs)) ++failures_;
    }
    return true;
  }

 private:
  bool haveAttempt_ = false;
  uint32_t lastAttemptMs_ = 0, attempts_ = 0, failures_ = 0;
};
}
