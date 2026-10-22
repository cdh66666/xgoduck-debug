// Platform/storage substitutes only. The parser and motor application functions are extracted.
static const uint8_t NUM_MOTORS=15;
static const float M_N=4095.0f, M_A=360.0f, SERVO_SPEED_UNIT=50.0f;
struct Motor { uint8_t id; uint16_t zeroPos,fbPos; float sign,fbAngleDeg,fbVelDeg; };
static Motor motors[NUM_MOTORS];
struct k_mutex {};
static k_mutex motorsMu;
static const int K_FOREVER=0;
static void k_mutex_lock(k_mutex*,int) {}
static void k_mutex_unlock(k_mutex*) {}
static uint32_t micros() { return 123456; }
struct FakeSerial {
  std::deque<uint8_t> bytes;
  int available() const { return (int)bytes.size(); }
  int read() { int value=bytes.front(); bytes.pop_front(); return value; }
} Serial1;
struct VoltageSampler {
  std::vector<std::array<uint32_t,4>> calls;
  void reply(uint8_t id,uint8_t v,uint8_t error,uint32_t now) { calls.push_back({id,v,error,now}); }
};
struct CurrentSampler {
  bool pending=false;
  std::vector<std::array<uint32_t,4>> calls;
  void reply(uint8_t idx,uint16_t v,uint8_t error,uint32_t now) { calls.push_back({idx,v,error,now}); }
};
