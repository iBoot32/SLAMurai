#include <MobaTools.h>
#include <math.h>
#define ENABLE_PIN 8
#define DISABLE_ALL_STEPPERS 0

// Robot params
#define WHEEL_RADIUS_M 0.0297
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.1325
#define STEPS_PER_REV 800
#define STEPS_PER_SEC_MAX 8000 / 4.0
const float STEPS_PER_M = (float)STEPS_PER_REV / (2.0f * (float)M_PI * WHEEL_RADIUS_M);

// Kinematic Acceleration & Deceleration Limits
#define ACCEL_XY 0.40f // m/s^2
#define DECEL_XY 0.55f // m/s^2 
#define ACCEL_W  2.0f // rad/s^2
#define DECEL_W  2.0f // rad/s^2

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
    drv.setRampLen(0);
    drv.setSpeed(0);
  }

  void setLinearVelocity(float v_mps) {
    float sps = v_mps * STEPS_PER_M;
    if (fabs(sps) > STEPS_PER_SEC_MAX) sps = copysign(STEPS_PER_SEC_MAX, sps);
    if (fabs(sps) < 1e-3) {
      drv.rotate(0);
      drv.setSpeedSteps(0);
      return;
    }
    int steps10 = abs(sps) * 10;
    drv.setSpeedSteps(steps10);
    if (sps == 0) {
      drv.setSpeedSteps(0);
    } else if (sps > 0) {
      drv.rotate(1);
    } else {
      drv.rotate(-1);
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

// -------- KINEMATIC STATE --------
float target_vx = 0, target_vy = 0, target_w = 0;
float current_vx = 0, current_vy = 0, current_w = 0;

float CMD_VEL_READ_PERIOD_US = 1e6 * (1 / 40.0);
unsigned long last_cmd_vel_send_time = 0;
unsigned long last_cmd_received_time = 0;
const unsigned long CMD_TIMEOUT_US = 500000;

float ODOM_SEND_PERIOD_US = 1e6 * (1 / 40.0);
float pose_x = 0, pose_y = 0, pose_yaw = 0;
float dx_b_sum = 0, dy_b_sum = 0, dyaw_sum = 0; 
long lastFront = 0, lastLeft = 0, lastBack = 0, lastRight = 0;
unsigned long last_odom_send_time = 0;

float move_towards(float current, float target, float accel_limit, float decel_limit, float dt) {
  if (current == target) return current;
  bool accelerating = (target > 0 && current >= 0 && target > current) || (target < 0 && current <= 0 && target < current);
  float max_step = (accelerating ? accel_limit : decel_limit) * dt;
  if (current < target) {
    current += max_step;
    if (current > target) current = target;
  } else {
    current -= max_step;
    if (current < target) current = target;
  }
  return current;
}

void loop() {
  unsigned long now = micros();
  float cmd_v[3];
  if (read_line_float(cmd_v)) {
    
    last_cmd_received_time = now;
    target_vx = cmd_v[0];
    target_vy = cmd_v[1];
    target_w  = cmd_v[2];
  }

  // ---- KINEMATIC TIMING & SMOOTHING ----
  if (now - last_cmd_vel_send_time >= CMD_VEL_READ_PERIOD_US) {
    float dt_smooth = (now - last_cmd_vel_send_time) / 1000000.0f;
    last_cmd_vel_send_time = now;

    // Timeout Safety
    if (now - last_cmd_received_time > CMD_TIMEOUT_US) {
      target_vx = 0; target_vy = 0; target_w = 0;
    }

    // Apply acceleration limits
    current_vx = move_towards(current_vx, target_vx, ACCEL_XY, DECEL_XY, dt_smooth);
    current_vy = move_towards(current_vy, target_vy, ACCEL_XY, DECEL_XY, dt_smooth);
    current_w  = move_towards(current_w,  target_w,  ACCEL_W,  DECEL_W,  dt_smooth);

    float vFront = -(current_vy + ROBOT_CENTER_TO_WHEEL_RADIUS * current_w);
    float vLeft  =  (current_vx - ROBOT_CENTER_TO_WHEEL_RADIUS * current_w);
    float vBack  =  (current_vy - ROBOT_CENTER_TO_WHEEL_RADIUS * current_w);
    float vRight = -(current_vx + ROBOT_CENTER_TO_WHEEL_RADIUS * current_w);
   
    stepperFront.setLinearVelocity(vFront);
    stepperLeft.setLinearVelocity(vLeft);
    stepperBack.setLinearVelocity(vBack);
    stepperRight.setLinearVelocity(vRight);
  }

  // ---- ODOM UPDATE (Runs as fast as possible) ----
  long pFront = stepperFront.pos();
  long pLeft = stepperLeft.pos();
  long pBack = stepperBack.pos();
  long pRight = stepperRight.pos();

  long dFront = pFront - lastFront;
  long dLeft = pLeft - lastLeft;
  long dBack = pBack - lastBack;
  long dRight = pRight - lastRight;
  lastFront = pFront; lastLeft = pLeft; lastBack = pBack; lastRight = pRight;

  float sFront = (float)dFront / STEPS_PER_M;
  float sLeft = (float)dLeft / STEPS_PER_M;
  float sBack = (float)dBack / STEPS_PER_M;
  float sRight = (float)dRight / STEPS_PER_M;

  float dx_b = ( sLeft - sRight ) * 0.5f;
  float dy_b = (-sFront + sBack ) * 0.5f;
  float dyaw = -(sFront + sLeft + sBack + sRight) * (1.0f / (4.0f * ROBOT_CENTER_TO_WHEEL_RADIUS));

  dx_b_sum += dx_b;
  dy_b_sum += dy_b;
  dyaw_sum += dyaw;

  float cy = cos(pose_yaw);
  float sy = sin(pose_yaw);
  pose_x += cy * dx_b - sy * dy_b;
  pose_y += sy * dx_b + cy * dy_b;
  pose_yaw += dyaw;

  // ---- SERIAL ODOM SEND ----
  now = micros();
  if (now - last_odom_send_time >= ODOM_SEND_PERIOD_US) {
    float dt_odom = (now - last_odom_send_time) / 1000000.0f;
    last_odom_send_time = now;

    float x_mm = roundf(pose_x * 1000.0f) / 1000.0f;
    float y_mm = roundf(pose_y * 1000.0f) / 1000.0f;
    float yaw_r = roundf(pose_yaw * 1000.0f) / 1000.0f;
    float vel_x = dx_b_sum / dt_odom;
    float vel_y = dy_b_sum / dt_odom;
    float vel_w = dyaw_sum / dt_odom;

    dx_b_sum = 0; dy_b_sum = 0; dyaw_sum = 0;

    Serial.print(x_mm, 3); Serial.print(',');
    Serial.print(y_mm, 3); Serial.print(',');
    Serial.print(yaw_r, 3); Serial.print(',');
    Serial.print(vel_x, 3); Serial.print(',');
    Serial.print(vel_y, 3); Serial.print(',');
    Serial.println(vel_w, 3);
  }
}