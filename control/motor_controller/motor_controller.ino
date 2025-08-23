#include <AccelStepper.h>
#include <math.h>

#define ENABLE_PIN 8
#define DISABLE_ALL_STEPPERS 1

// Robot params
#define WHEEL_RADIUS_M 0.03175
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.145

// Speed params
#define STEPS_PER_REV 800
#define STEPS_PER_SEC_MAX 3500 / 4.0
const float STEPS_PER_M = (float)STEPS_PER_REV / (2.0f * (float)M_PI * WHEEL_RADIUS_M);

// Odom
#define ODOM_DT (1000.0 / 50) // 50 Hz
uint32_t last_odom_ms = 0;

// --------- Non-blocking TX buffer ----------
static char tx_buf[64];
static uint8_t tx_pos = 0, tx_len = 0;
inline void pumpSerial() {
  if (tx_pos < tx_len) {
    int room = Serial.availableForWrite();
    if (room > 0) {
      int w = min(room, (int)(tx_len - tx_pos));
      Serial.write((const uint8_t*)&tx_buf[tx_pos], w);
      tx_pos += w;
    }
  }
}

inline void queueOdomLine(float x, float y, float z, float a) {
  if (tx_pos == tx_len) {
    char sx[12], sy[12], sz[12], sa[12];
    dtostrf(x, 0, 2, sx);
    dtostrf(y, 0, 2, sy);
    dtostrf(z, 0, 2, sz);
    dtostrf(a, 0, 2, sa);
    tx_len = (uint8_t)snprintf(tx_buf, sizeof(tx_buf), "%s,%s,%s,%s\n", sx, sy, sz, sa);
    tx_pos = 0;
  }
}

// ------- cmd_vel Parser --------
bool read_line_float(float* out) {
  static char rxbuf[64];
  static uint8_t rxlen = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxlen == 0) continue;
      rxbuf[rxlen] = '\0'; rxlen = 0;
      char *p = rxbuf;
      for (int i = 0; i < 3; i++) { out[i] = (float)strtod(p, &p); if (*p == ',') p++; }
      return true;
    }
    if (rxlen + 1 < sizeof(rxbuf)) rxbuf[rxlen++] = c;
  }
  return false;
}

// --------- Stepper wrapper ----------
struct Stepper {
  AccelStepper drv;
  Stepper(uint8_t step, uint8_t dir) : drv(AccelStepper::DRIVER, step, dir) {}
  void setup() { drv.setMaxSpeed(STEPS_PER_SEC_MAX); drv.setMinPulseWidth(3); drv.setSpeed(0); }
  void setLinearVelocity(float v_mps) {
    float sps = v_mps * STEPS_PER_M;
    if (fabs(sps) > STEPS_PER_SEC_MAX) sps = copysign(STEPS_PER_SEC_MAX, sps);
    drv.setSpeed(sps);
  }
  inline void service() { drv.runSpeed(); }
  inline long pos() const { return drv.currentPosition(); }
};

Stepper stepperX(2, 5);
Stepper stepperY(3, 6);
Stepper stepperZ(4, 7);
Stepper stepperA(12, 13);

static inline float stepsToMeters(long steps) {
  return (steps / (float)STEPS_PER_REV) * (2.0f * (float)M_PI * WHEEL_RADIUS_M);
}

void setup() {
  Serial.begin(1000000); // high baud reduces wire time
  pinMode(ENABLE_PIN, OUTPUT);
  if (DISABLE_ALL_STEPPERS) { digitalWrite(ENABLE_PIN, HIGH); return; }
  digitalWrite(ENABLE_PIN, LOW);
  delayMicroseconds(2000);

  stepperX.setup(); stepperY.setup(); stepperZ.setup(); stepperA.setup();
  last_odom_ms = millis();
}

void loop() {
  // Drain TX so prints never block
  pumpSerial();

  // Service steppers
  stepperX.service(); stepperY.service(); stepperZ.service(); stepperA.service();

  // Parse cmd_vel
  float cmd_v[3];
  if (read_line_float(cmd_v)) {
    float vx = cmd_v[0], vy = cmd_v[1], w = cmd_v[2];
    float vX = -(vy + ROBOT_CENTER_TO_WHEEL_RADIUS * w);
    float vY =  (vx - ROBOT_CENTER_TO_WHEEL_RADIUS * w);
    float vZ =  (vy - ROBOT_CENTER_TO_WHEEL_RADIUS * w);
    float vA = -(vx + ROBOT_CENTER_TO_WHEEL_RADIUS * w);
    stepperX.setLinearVelocity(vX);
    stepperY.setLinearVelocity(vY);
    stepperZ.setLinearVelocity(vZ);
    stepperA.setLinearVelocity(vA);
  }

  // Odom snapshot and queue
  uint32_t now = millis();
  if (now - last_odom_ms >= ODOM_DT) {
    last_odom_ms += ODOM_DT;
    queueOdomLine(
      stepsToMeters(stepperX.pos()),
      stepsToMeters(stepperY.pos()),
      stepsToMeters(stepperZ.pos()),
      stepsToMeters(stepperA.pos())
    );
  }

  stepperX.service(); stepperY.service(); stepperZ.service(); stepperA.service();

  // Keep draining TX
  pumpSerial();
}
