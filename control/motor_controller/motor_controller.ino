#include <MobaTools.h>
#include <math.h>
#define ENABLE_PIN 8
#define DISABLE_ALL_STEPPERS 0

// Robot params
#define WHEEL_RADIUS_M 0.036104
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.1268
#define STEPS_PER_REV 800
#define STEPS_PER_SEC_MAX 8000 / 4.0
const float STEPS_PER_M = (float)STEPS_PER_REV / (2.0f * (float)M_PI * WHEEL_RADIUS_M);

// ------- cmd_vel parser --------
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
  MoToStepper drv;
  uint8_t _stepPin, _dirPin;
  Stepper(uint8_t step, uint8_t dir) : drv(STEPS_PER_REV, STEPDIR), _stepPin(step), _dirPin(dir) {}
  inline long pos() const { return drv.readSteps(); }

  void setup() { 
    drv.attach(_stepPin, _dirPin); 
    drv.setRampLen(35); // quick accel for smoothing
    drv.setSpeed(0); // zero initial speed
  }

  void setLinearVelocity(float v_mps) {
    float sps = v_mps * STEPS_PER_M;
    if (fabs(sps) > STEPS_PER_SEC_MAX) sps = copysign(STEPS_PER_SEC_MAX, sps); // Clamping
    
    // Convert to steps/10sec (integer) for MobaTools
    int steps10 = abs(sps) * 10; 
    drv.setSpeedSteps(steps10);

    if (sps == 0) {
      drv.setSpeedSteps(0);   // ramp down to stop
    } else if (sps > 0) {
      drv.rotate(1); // Forward at speed
    } else {
      drv.rotate(-1); // Backward at speed
    }
  }
};

Stepper stepperFront(3, 6);
Stepper stepperLeft(2, 5);
Stepper stepperBack(4, 7);
Stepper stepperRight(12, 13);

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  if (DISABLE_ALL_STEPPERS) { digitalWrite(ENABLE_PIN, HIGH); return; }
  digitalWrite(ENABLE_PIN, LOW);
  delayMicroseconds(2000);
  stepperFront.setup(); stepperLeft.setup(); stepperBack.setup(); stepperRight.setup();
}

// -------- CMD_VEL READING --------
float CMD_VEL_READ_PERIOD_US = 1e6 * (1 / 40.0);
unsigned long last_cmd_vel_send_time = 0;

// -------- ODOM SENDING --------
float ODOM_SEND_PERIOD_US = 1e6 * (1 / 40.0);
float pose_x = 0, pose_y = 0, pose_yaw = 0;
long lastFront = 0, lastLeft = 0, lastBack = 0, lastRight = 0;
unsigned long last_odom_send_time = 0;

void loop() {
  // Parse cmd_vel at specified hz
  unsigned long now = micros();
  if (now - last_cmd_vel_send_time >= CMD_VEL_READ_PERIOD_US) {
    last_cmd_vel_send_time = now;

    // Inverse kinematics (body twist -> motor vel)
    float cmd_v[3];
    if (read_line_float(cmd_v)) {
      float vx = cmd_v[0], vy = cmd_v[1], w = cmd_v[2];
      float vFront = -(vy + ROBOT_CENTER_TO_WHEEL_RADIUS * w);
      float vLeft =   (vx - ROBOT_CENTER_TO_WHEEL_RADIUS * w);
      float vBack =   (vy - ROBOT_CENTER_TO_WHEEL_RADIUS * w);
      float vRight = -(vx + ROBOT_CENTER_TO_WHEEL_RADIUS * w);
   
      stepperFront.setLinearVelocity(vFront);
      stepperLeft.setLinearVelocity(vLeft);
      stepperBack.setLinearVelocity(vBack);
      stepperRight.setLinearVelocity(vRight);
    }
  }

  // ---- ODOM UPDATE ----
  long pFront = stepperFront.pos();
  long pLeft = stepperLeft.pos();
  long pBack = stepperBack.pos();
  long pRight = stepperRight.pos();

  // Motor tick deltas
  long dFront = pFront - lastFront;
  long dLeft = pLeft - lastLeft;
  long dBack = pBack - lastBack;
  long dRight = pRight - lastRight;
  lastFront = pFront; lastLeft = pLeft; lastBack = pBack; lastRight = pRight;

  // Motor tick delta to meter deltas
  float sFront = dFront / STEPS_PER_M;
  float sLeft = dLeft / STEPS_PER_M;
  float sBack = dBack / STEPS_PER_M;
  float sRight = dRight / STEPS_PER_M;

  // Forward kinematics (stepper deltas to world pose)
  float dx_b = ( sLeft - sRight ) * 0.5f;
  float dy_b = (-sFront + sBack ) * 0.5f;
  float dyaw = -(sFront + sLeft + sBack + sRight) * (1.0f / (4.0f * ROBOT_CENTER_TO_WHEEL_RADIUS));

  // Integrate into world frame
  float cy = cos(pose_yaw);
  float sy = sin(pose_yaw);
  pose_x += cy * dx_b - sy * dy_b;
  pose_y += sy * dx_b + cy * dy_b;
  pose_yaw += dyaw;

  // ---- ODOM PUBLISH @ SPECIFIED HZ ----
  now = micros();
  if (now - last_odom_send_time >= ODOM_SEND_PERIOD_US) {
    last_odom_send_time = now;

    float x_mm = roundf(pose_x * 1000.0f) / 1000.0f;
    float y_mm = roundf(pose_y * 1000.0f) / 1000.0f;
    float yaw_r = roundf(pose_yaw * 1000.0f) / 1000.0f;

    // Accurate to 1mm and 0.5 deg
    Serial.print(x_mm, 3);
    Serial.print(',');
    Serial.print(y_mm, 3);
    Serial.print(',');
    Serial.println(yaw_r, 2);
  }
}