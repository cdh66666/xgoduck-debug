#pragma once
#include <stdint.h>

// Main-loop startup owner. Each tick is one bounded attempt, never an
// unbounded setup wait. Linux may become ready tens of seconds after MCU boot.
// localMask is ONLY local callback binding, not Router ownership/ARM approval.
class RouteStartup {
 public:
  static constexpr uint32_t PERIOD_MS=1000, REQUEST_MS=200;
  template<class Port> bool tick(uint32_t nowMs,Port &port) {
    if(reportsStarted_) return false;
    if(haveAttempt_ && uint32_t(nowMs-lastAttemptMs_)<PERIOD_MS) return false;
    haveAttempt_=true;lastAttemptMs_=nowMs;
    if(!bridgeStarted_) {
      ++beginAttempts_;
      if(!port.beginBounded(REQUEST_MS)) return true;
      bridgeStarted_=true;
    }
    for(unsigned i=0;i<4;++i) {
      if(!(localMask_&(1u<<i))) {
        ++bindAttempts_;
        if(port.bindLocal(i)) localMask_|=uint8_t(1u<<i);
      }
    }
    if(localMask_==15) {
      ++reportAttempts_;
      reportsStarted_=port.startReports();
    }
    return true;
  }
  bool bridgeStarted() const {return bridgeStarted_;}
  bool reportsStarted() const {return reportsStarted_;}
  uint8_t localMask() const {return localMask_;}
  uint32_t beginAttempts() const {return beginAttempts_;}
  uint32_t bindAttempts() const {return bindAttempts_;}
  uint32_t reportAttempts() const {return reportAttempts_;}
 private:
  bool haveAttempt_=false,bridgeStarted_=false,reportsStarted_=false;
  uint8_t localMask_=0;
  uint32_t lastAttemptMs_=0,beginAttempts_=0,bindAttempts_=0,reportAttempts_=0;
};

// Only published state is inspected; no control/EEPROM operation is issued.
inline bool routeRefreshAllowed(uint32_t seq, uint8_t flags,
                                uint32_t sampleUs, uint32_t nowUs) {
  return seq && !(flags & (2u | 8u | 16u)) &&
         uint32_t(nowUs - sampleUs) <= 50000u;
}

class RouteRefresh {
 public:
  static constexpr uint32_t PERIOD_MS = 1000;
  static constexpr uint32_t REQUEST_MS = 200;
  template<class Port> bool tick(uint32_t nowMs, Port &port) {
    if (haveAttempt_ && uint32_t(nowMs-lastAttemptMs_) < PERIOD_MS) return false;
    if (!port.allowed()) return false;
    haveAttempt_=true; lastAttemptMs_=nowMs; lastAcceptedMask_=0;
    static const char *const names[]={"duck_command","duck_cal","duck_servo","duck_factory"};
    for (unsigned i=0;i<4;++i) {
      // Snapshot locks have been released before each Bridge operation.
      if (!port.allowed()) {++interrupted_; return true;}
      ++attempts_;
      if (port.registerOnly(names[i],REQUEST_MS)) {
        ++accepted_; lastAcceptedMask_|=uint8_t(1u<<i);
      } else {
        // false includes timeout and duplicate-name replies. It is not proof
        // that an existing route belongs to this MCU or that it is unavailable.
        ++notConfirmed_;
      }
    }
    return true;
  }
  uint32_t attempts() const {return attempts_;}
  uint32_t accepted() const {return accepted_;}
  uint32_t notConfirmed() const {return notConfirmed_;}
  uint32_t interrupted() const {return interrupted_;}
  uint8_t lastAcceptedMask() const {return lastAcceptedMask_;}
 private:
  bool haveAttempt_=false;
  uint32_t lastAttemptMs_=0, attempts_=0, accepted_=0, notConfirmed_=0, interrupted_=0;
  uint8_t lastAcceptedMask_=0;
};
