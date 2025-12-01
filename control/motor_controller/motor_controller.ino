#include <AccelStepper.h>
#include <math.h>

#define ENABLE_PIN 8
#define DISABLE_ALL_STEPPERS 0

// Robot params
#define WHEEL_RADIUS_M 0.03665
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.1367

// Speed params
#define STEPS_PER_REV 800
#define STEPS_PER_SEC_MAX 3500 / 4.0
const float STEPS_PER_M = (float)STEPS_PER_REV / (2.0f * (float)M_PI * WHEEL_RADIUS_M); // steps/m = steps/rev * rev/meters
static inline float stepsToMeters(long steps) {
  return (steps / (float)STEPS_PER_REV) *
         (2.0f * (float)M_PI * WHEEL_RADIUS_M);
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
      for (int i = 0; i < 3; i++) {
        out[i] = (float)strtod(p, &p);
        if (*p == ',') p++;
      }
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
  void setup() { drv.setMaxSpeed(STEPS_PER_SEC_MAX); drv.setSpeed(0); }
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

float pose_x = 0, pose_y = 0, pose_yaw = 0;
long lastX = 0, lastY = 0, lastZ = 0, lastA = 0;

void setup() {
  Serial.begin(250000);
  pinMode(ENABLE_PIN, OUTPUT);
  if (DISABLE_ALL_STEPPERS) { digitalWrite(ENABLE_PIN, HIGH); return; }
  digitalWrite(ENABLE_PIN, LOW);
  delayMicroseconds(2000);

  stepperX.setup(); stepperY.setup(); stepperZ.setup(); stepperA.setup();
}

#define CMDVEL_SET_HZ 30
#define ODOM_COMPUTE_HZ 50
unsigned long last_cmdvel_set_time = 0;
unsigned long last_odom_compute_time = 0;

void loop() {
  stepperX.service();
  stepperY.service();
  stepperZ.service();
  stepperA.service();

  unsigned long now = millis();

  // ---- CMD_VEL UPDATE ----
  if (now - last_cmdvel_set_time >= 1000 / CMDVEL_SET_HZ) {
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
    last_cmdvel_set_time = now;
  }

  // ---- ODOM UPDATE ----
  if (now - last_odom_compute_time >= 1000 / ODOM_COMPUTE_HZ) {
      long pF = stepperX.pos();
      long pR = stepperY.pos();
      long pB = stepperZ.pos();
      long pL = stepperA.pos();

      long dF = pF - lastX;
      long dR = pR - lastY;
      long dB = pB - lastZ;
      long dL = pL - lastA;

      lastX = pF;  lastY = pR;  lastZ = pB;  lastA = pL;

      float sF = stepsToMeters(dF);
      float sR = stepsToMeters(dR);
      float sB = stepsToMeters(dB);
      float sL = stepsToMeters(dL);

      // Correct 90� omni forward kinematics
      float dx_b  = (sR - sL) * 0.5f;                                  // body X (right)
      float dy_b  = (sF - sB) * 0.5f;                                  // body Y (forward)
      float dyaw  = -(sF + sR + sB + sL) * 
                    (1.0f / (4.0f * ROBOT_CENTER_TO_WHEEL_RADIUS));    // rotation

      // Integrate into world frame
      float cy = cosf(pose_yaw);
      float sy = sinf(pose_yaw);

      pose_x   += cy * dx_b - sy * dy_b;
      pose_y   += sy * dx_b + cy * dy_b;
      pose_yaw += dyaw;

      Serial.print(pose_x, 4); Serial.print(",");
      Serial.print(pose_y, 4); Serial.print(",");
      Serial.println(pose_yaw, 4);

      last_odom_compute_time = now;
  }
}

