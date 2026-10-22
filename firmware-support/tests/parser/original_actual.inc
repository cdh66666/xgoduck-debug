static VoltageSampler displayVoltage;
static CurrentSampler currentSampler;
static bool servoBootPdReading=false;

static uint16_t servoRxMask = 0;
static uint16_t servoPdMask = 0;
static uint8_t servoBootP[NUM_MOTORS] = {}, servoBootD[NUM_MOTORS] = {};
static uint8_t servoParserFlag = 0;
static uint8_t servoRxBuffer[32];
static uint8_t servoRxLen = 0;
static uint8_t servoRxDataLen = 0;
// Only the hardware task accesses these per-request results. 0xFF means idle;
// ID 0 is a valid servo address, just like IDs 1..253.
static uint8_t servoPingId = 0xFF;
static bool servoPingSeen = false;
static uint8_t servoReadId = 0xFF;
static bool servoReadSeen = false;
static uint16_t servoReadPos = 0;


static int motorIndexById(uint8_t id) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (motors[i].id == id) {
      return i;
    }
  }
  return -1;
}

static int16_t hostWord(uint8_t lo, uint8_t hi) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

static int32_t hostVelSignMag(uint8_t lo, uint8_t hi) {
  /* Bit 15 is a direction flag, not a two's-complement sign. */
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  int32_t speed = (int32_t)(raw & 0x7FFFu);
  if (raw & 0x8000u) {
    speed = -speed;
  }
  return speed;
}

static void applyMotorFb(uint8_t id, int16_t pos, int32_t vel) {
  int idx = motorIndexById(id);
  if (idx < 0) {
    return;
  }
  servoRxMask |= (1u << idx);
  k_mutex_lock(&motorsMu, K_FOREVER);
  motors[idx].fbPos = (uint16_t)pos;
  motors[idx].fbAngleDeg =
      (float)(pos - motors[idx].zeroPos) * M_A / M_N * motors[idx].sign;
  motors[idx].fbVelDeg =
      (float)vel * SERVO_SPEED_UNIT * (360.0f / 4096.0f) * motors[idx].sign;
  k_mutex_unlock(&motorsMu);
}

static void resetServoParser() {
  servoParserFlag = 0;
  servoRxLen = 0;
  servoRxDataLen = 0;
}

static void parseServoRx() {
  while (Serial1.available()) {
    uint8_t res = (uint8_t)Serial1.read();
    switch (servoParserFlag) {
      case 0:
        if (res == 0xFF) {
          servoParserFlag = 1;
          servoRxBuffer[0] = 0xFF;
        }
        break;
      case 1:
        if (res == 0xFF) {
          servoParserFlag = 2;
          servoRxBuffer[1] = 0xFF;
        } else {
          servoParserFlag = 0;
        }
        break;
      case 2:
        servoRxBuffer[2] = res;
        servoParserFlag = 3;
        break;
      case 3:
        servoRxBuffer[3] = res;
        if (res == 0x02 || res == 0x03 || res == 0x04 || res == 0x08 || res == 0x09 || res == 0x0B) {
          servoParserFlag = 4;
          servoRxLen = 0;
          servoRxDataLen = res;
        } else {
          servoParserFlag = 0;
        }
        break;
      case 4:
        if (4 + servoRxLen >= sizeof(servoRxBuffer)) {
          servoParserFlag = 0;
          break;
        }
        servoRxBuffer[4 + servoRxLen] = res;
        servoRxLen++;
        if (servoRxLen == servoRxDataLen) {
          uint8_t check = 0;
          servoParserFlag = 0;
          for (uint8_t i = 0; i < (uint8_t)(1 + servoRxDataLen); i++) {
            check = (uint8_t)(check + servoRxBuffer[2 + i]);
          }
          check = (uint8_t)~check;
          if (check != servoRxBuffer[3 + servoRxDataLen]) {
            break;
          }
          uint8_t sid = servoRxBuffer[2];
          uint8_t dataLen = servoRxDataLen;
          if (dataLen == 2 && servoPingId <= 253 && sid == servoPingId) {
            // Even a device error is evidence that this address responded.
            servoPingSeen = true;
            break;
          }
          if(dataLen==3) {
            displayVoltage.reply(sid,servoRxBuffer[5],servoRxBuffer[4],micros());
            break;
          }
          const int idx = motorIndexById(sid);
          if(dataLen==4 && currentSampler.pending) {
            if(idx>=0)currentSampler.reply((uint8_t)idx,(uint16_t)hostWord(servoRxBuffer[5],servoRxBuffer[6]),servoRxBuffer[4],micros());
            break;
          }
          if (dataLen == 4 && servoBootPdReading && servoRxBuffer[4] == 0 && idx >= 0) {
            servoBootP[idx]=servoRxBuffer[5]; servoBootD[idx]=servoRxBuffer[6];
            servoPdMask |= 1u << idx;
            break;
          }
          // Only accept the requested 6-byte status, with no device error.
          if (dataLen != 8 || servoRxBuffer[4] != 0) {
            break;
          }
          if (servoReadId <= 253 && sid == servoReadId) {
            servoReadSeen = true;
            servoReadPos = (uint16_t)hostWord(servoRxBuffer[5], servoRxBuffer[6]);
          }
          applyMotorFb(sid, hostWord(servoRxBuffer[5], servoRxBuffer[6]),
                       hostVelSignMag(servoRxBuffer[7], servoRxBuffer[8]));
        }
        break;
      default:
        servoParserFlag = 0;
        break;
    }
  }
}
