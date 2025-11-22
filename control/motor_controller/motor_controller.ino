#include <AccelStepper.h>
#include <math.h>

#define ENABLE_PIN 8
#define DISABLE_ALL_STEPPERS 0

// Robot params
#define WHEEL_RADIUS_M 0.03575
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.127

// Speed params
#define STEPS_PER_REV 800
#define STEPS_PER_SEC_MAX 3750 / 4.0
const float STEPS_PER_M = (float)STEPS_PER_REV / (2.0f * (float)M_PI * WHEEL_RADIUS_M);

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
Stepper stepperY(4, 7);
Stepper stepperZ(12, 13);
Stepper stepperA(3, 6);

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
}

void loop() {
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
}