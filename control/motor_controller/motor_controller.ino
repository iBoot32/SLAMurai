#include <AccelStepper.h>
#include <math.h>

#define ENABLE_PIN 8
#define DISABLE_ALL_STEPPERS 0

// Robot params
#define WHEEL_RADIUS_M 0.05    // 100 mm diameter
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.145

// Speed params
#define STEPS_PER_REV 800      // 1/4 microstep => 800 steps/rev
#define RPM_MAX 65
#define STEPS_PER_SEC_MAX ((RPM_MAX)/60.0f * (STEPS_PER_REV))
const float STEPS_PER_M = (float)STEPS_PER_REV / (2.0f * (float)M_PI * WHEEL_RADIUS_M);
#define ODOM_DT (1000.0 / 20) // 25 Hz
uint32_t last_odom_ms = 0;

// Parse cmd_vel serial string into [x, y, w]
bool read_line_float(float* out) {
  static char rxbuf[64];
  static uint8_t rxlen = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxlen == 0) continue;
      rxbuf[rxlen] = '\0';
      rxlen = 0;

      char *p = rxbuf;
      for (int i = 0; i < 3; i++) {
        out[i] = (float)strtod(p, &p); // parse and move pointer
        if (*p == ',') p++;            // skip comma
      }
      return true;
    }
    if (rxlen + 1 < sizeof(rxbuf)) rxbuf[rxlen++] = c;
  }
  return false;
}

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

  // Inverse kinematic to obtain wheel linear velocities:  ωi = (sin((i − 1)·θ + γ)·vbx − cos((i − 1)·θ + γ)·vby − R·ωbz)
  // We pre-compute sin/cos terms using θ=pi/2, γ=0, i=[1,4]
  float cmd_v[3];
  if (read_line_float(cmd_v)) {
    float vx = cmd_v[0];
    float vy = cmd_v[1];
    float w  = cmd_v[2];
  
    float vX = -(vy + ROBOT_CENTER_TO_WHEEL_RADIUS * w);
    float vY =  (vx - ROBOT_CENTER_TO_WHEEL_RADIUS * w);
    float vZ =  (vy - ROBOT_CENTER_TO_WHEEL_RADIUS * w);
    float vA = -(vx + ROBOT_CENTER_TO_WHEEL_RADIUS * w);
  
    stepperX.setLinearVelocity(vX);
    stepperY.setLinearVelocity(vY);
    stepperZ.setLinearVelocity(vZ);
    stepperA.setLinearVelocity(vA);
  }

  // Send odom
  uint32_t now = millis();
  if (now - last_odom_ms >= ODOM_DT) {
    last_odom_ms += ODOM_DT;
  
    Serial.print(stepsToMeters(stepperX.pos()), 2); Serial.print(",");
    stepperX.service(); stepperY.service(); stepperZ.service(); stepperA.service();
    
    Serial.print(stepsToMeters(stepperY.pos()), 2); Serial.print(",");
    stepperX.service(); stepperY.service(); stepperZ.service(); stepperA.service();
    
    Serial.print(stepsToMeters(stepperZ.pos()), 2); Serial.print(",");
    stepperX.service(); stepperY.service(); stepperZ.service(); stepperA.service();
    
    Serial.println(stepsToMeters(stepperA.pos()), 2);
  }
}
