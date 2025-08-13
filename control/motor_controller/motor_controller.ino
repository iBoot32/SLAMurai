#include <AccelStepper.h>
#include <math.h>

#define ENABLE_PIN 8
#define DISABLE_ALL_STEPPERS 1

// Wheel and movement params
#define STEPS_PER_REV 800      // 1/4 microstep => 800 steps/rev
#define WHEEL_RADIUS_M 0.05    // 100 mm diameter
#define RPM_MAX 65
#define STEPS_PER_SEC_MAX ((RPM_MAX)/60.0f * (STEPS_PER_REV))
const float STEPS_PER_M = (float)STEPS_PER_REV / (2.0f * (float)M_PI * WHEEL_RADIUS_M);

// Serial R/W
#define ODOM_DT 40   // 25 Hz
uint32_t last_odom_ms = 0;

// Non-blocking read serial
static char rxbuf[32];
static uint8_t rxlen = 0;
bool read_line_float(float* out_val) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rxbuf[rxlen] = '\0';
      rxlen = 0;
      *out_val = atof(rxbuf);
      return true;
    }
    if (rxlen + 1 < sizeof(rxbuf)) rxbuf[rxlen++] = c;
  }
  return false;
}

// Stepper class, encapsulates stepper setup, velocity setting, and running
struct Stepper {
  uint8_t stepPin, dirPin;
  AccelStepper drv;

  Stepper(uint8_t step, uint8_t dir)
  : stepPin(step), dirPin(dir), drv(AccelStepper::DRIVER, step, dir) {}

  void setup() {
    drv.setMaxSpeed(STEPS_PER_SEC_MAX);
    drv.setMinPulseWidth(5);
    drv.setSpeed(0);
  }

  void setLinearVelocity(float v_mps) {
    float sps = v_mps * STEPS_PER_M;          // signed steps/s
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
  // meters = revs * circumference = (steps/STEPS_PER_REV) * 2πR
  return (steps / (float)STEPS_PER_REV) * (2.0f * (float)M_PI * WHEEL_RADIUS_M);
}

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);

  if (DISABLE_ALL_STEPPERS) { digitalWrite(ENABLE_PIN, HIGH); return; }
  digitalWrite(ENABLE_PIN, LOW);          // enable drivers
  delayMicroseconds(2000);                // wake/enable settle

  stepperX.setup();
  stepperY.setup();
  stepperZ.setup();
  stepperA.setup();
}

void loop() {
  // Service steppers ASAP every iteration
  stepperX.service();
  stepperY.service();
  stepperZ.service();
  stepperA.service();

  // cmd_vel read over serial to set motor speeds
  float cmd_v;
  if (read_line_float(&cmd_v)) {
    stepperX.setLinearVelocity(cmd_v);
    stepperY.setLinearVelocity(cmd_v);
    stepperZ.setLinearVelocity(cmd_v);
    stepperA.setLinearVelocity(cmd_v);
  }

  // Odom writing over serial
  uint32_t now = millis();
  if (now - last_odom_ms >= ODOM_DT) {
    last_odom_ms += ODOM_DT;
    float meters = stepsToMeters(stepperY.pos());
    Serial.println(meters, 6);
 
  }
}
