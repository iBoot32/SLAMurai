// X-drive controller with a hard safety envelope.  Based on ../motor_controller_xdrive/motor_controller_xdrive.ino (left
// untouched).  Differences, all of them one-way tightenings:
//   * wheel-centre radius 0.133 m (v17 CAD; the old file had the v13 value 0.151)
//   * every command is limited to V_MAX_XY / W_MAX / wheel-speed cap by safety_limits.h (compile-time, not changeable
//     over serial); the four wheel speeds are scaled TOGETHER so the direction of travel is kept
//   * malformed serial lines (not three finite numbers) are dropped, not half-parsed
//   * command timeout 0.25 s (was 0.5 s)
//   * tether: farther than TETHER_M from the power-up point -> latched stop (needs a board reset)
//   * optional bumper switch (BUMPER_PIN) -> latched stop
//   * odometry line gets a 7th field: fault/limit flags (see SL_FLAG_*)
// Serial protocol is unchanged: "vx,vy,w\n" in (m/s, m/s, rad/s), "x,y,yaw,vx,vy,w,flags\n" out at 40 Hz.
#include <MobaTools.h>
#include <math.h>
#include "safety_limits.h"

#define ENABLE_PIN 8
#define DISABLE_ALL_STEPPERS 0
#define BUMPER_PIN -1            // set to a pin (INPUT_PULLUP, switch to GND) when a bumper ring is fitted

// Robot params
#define WHEEL_RADIUS_M 0.0297
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.133   // v17 CAD wheel-centre plane radius; trim after a 360 deg spin test
#define STEPS_PER_REV 1600
#define STEPS_PER_SEC_MAX 5000
#define MOTOR_DIR_SIGN -1      // TMC2209 drivers spin every motor the opposite way to the old DRV8825: -1 flips all wheels (and their step counts)
const float STEPS_PER_M = (float)STEPS_PER_REV / (2.0f * (float)M_PI * WHEEL_RADIUS_M);

// ---- safety envelope (robot_side/config/safety_config.yaml is generated from the same numbers) ----
#define V_MAX_XY   0.21f                                   // m/s
#define W_MAX      0.5f                                    // rad/s
#define WHEEL_CAP  (0.98f * (float)STEPS_PER_SEC_MAX / STEPS_PER_M)   // 0.5715 m/s
#define TETHER_M   6.0f                                    // m from the power-up point (the ROS gate has a tighter, per-session one)
const SlParams LIMITS = {V_MAX_XY, W_MAX, WHEEL_CAP, ROBOT_CENTER_TO_WHEEL_RADIUS};

// Kinematic Acceleration & Deceleration Limits
#define ACCEL_XY 0.40f // m/s^2
#define DECEL_XY 0.55f // m/s^2
#define ACCEL_W  2.0f // rad/s^2
#define DECEL_W  2.0f // rad/s^2
#define ESTOP_DECEL_XY 1.0f  // m/s^2 when a latched stop is active (CAD: motors can do 2.2 m/s^2 at half pull-out torque)
#define ESTOP_DECEL_W  3.0f

unsigned char fw_flags = 0;
bool stop_latched = false;

// ------- cmd_vel parser: accepts only "a,b,c" with three finite numbers -------
bool read_line_float(float* out) {
  static char rxbuf[64];
  static uint8_t rxlen = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxlen == 0) continue;
      rxbuf[rxlen] = '\0'; rxlen = 0;
      char *p = rxbuf;
      bool ok = true;
      for (int i = 0; i < 3; i++) {
        char *end = p;
        out[i] = (float)strtod(p, &end);
        if (end == p) { ok = false; break; }
        p = end;
        if (i < 2) { if (*p != ',') { ok = false; break; } p++; }
      }
      if (ok && *p != '\0') ok = false;
      if (!ok) { fw_flags |= SL_FLAG_MALFORMED; continue; }
      return true;
    }
    if (rxlen + 1 < sizeof(rxbuf)) rxbuf[rxlen++] = c;
    else rxlen = 0;                               // overlong line: drop it
  }
  return false;
}

// --------- Stepper wrapper ----------
struct Stepper {
  MoToStepper drv;
  uint8_t _stepPin, _dirPin;
  Stepper(uint8_t step, uint8_t dir) : drv(STEPS_PER_REV, STEPDIR), _stepPin(step), _dirPin(dir) {}
  inline long pos() const { return MOTOR_DIR_SIGN * drv.readSteps(); }

  void setup() {
    drv.attach(_stepPin, _dirPin);
    drv.setRampLen(0);
    drv.setSpeed(0);
  }

  void setLinearVelocity(float v_mps) {
    float sps = MOTOR_DIR_SIGN * v_mps * STEPS_PER_M;
    if (fabs(sps) > STEPS_PER_SEC_MAX) sps = copysign(STEPS_PER_SEC_MAX, sps);   // never reached: sl_limit keeps wheels under the cap
    if (fabs(sps) < 1e-3) {
      drv.rotate(0);
      drv.setSpeedSteps(0);
      return;
    }
    uint16_t steps10 = (uint16_t)(fabs(sps) * 10.0f);    drv.setSpeedSteps(steps10);
    if (sps == 0) {
      drv.setSpeedSteps(0);
    } else if (sps > 0) {
      drv.rotate(1);
    } else {
      drv.rotate(-1);
    }
  }
};

// v17 X-drive: wheels on the diagonals, camera (+x) between wheels 45 and 315 (angles CCW from +x, seen from above).
// Pin mapping kept from the old controller: Front->45 (X), Left->135 (Y), Back->225 (Z), Right->315 (A).
// Per-wheel sign: flip if a wheel turns the wrong way when commanding +w (robot should spin CCW seen from above).
#define SGN_45  1.0f
#define SGN_135 1.0f
#define SGN_225 1.0f
#define SGN_315 1.0f
const float S2 = 0.70710678f;
Stepper stepperFront(3, 6);     // wheel at 45 deg
Stepper stepperLeft(2, 5);      // wheel at 135 deg
Stepper stepperBack(4, 7);      // wheel at 225 deg
Stepper stepperRight(12, 13);   // wheel at 315 deg

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
#if BUMPER_PIN >= 0
  pinMode(BUMPER_PIN, INPUT_PULLUP);
#endif
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
const unsigned long CMD_TIMEOUT_US = 250000;

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
    SlCmd lim = sl_limit({cmd_v[0], cmd_v[1], cmd_v[2]}, LIMITS, &fw_flags);
    if (!stop_latched) { target_vx = lim.vx; target_vy = lim.vy; target_w = lim.w; }
  }

  // ---- latched stops: tether and bumper
  if (!stop_latched && sl_tether_exceeded(pose_x, pose_y, 0.0f, 0.0f, TETHER_M)) { stop_latched = true; fw_flags |= SL_FLAG_TETHER; }
#if BUMPER_PIN >= 0
  if (!stop_latched && digitalRead(BUMPER_PIN) == LOW) { stop_latched = true; fw_flags |= SL_FLAG_BUMPER; }
#endif
  if (stop_latched) { target_vx = 0; target_vy = 0; target_w = 0; }

  // ---- KINEMATIC TIMING & SMOOTHING ----
  if (now - last_cmd_vel_send_time >= CMD_VEL_READ_PERIOD_US) {
    float dt_smooth = (now - last_cmd_vel_send_time) / 1000000.0f;
    last_cmd_vel_send_time = now;

    // Timeout Safety
    if (now - last_cmd_received_time > CMD_TIMEOUT_US) {
      target_vx = 0; target_vy = 0; target_w = 0;
      fw_flags |= SL_FLAG_TIMEOUT;
    } else {
      fw_flags &= ~SL_FLAG_TIMEOUT;
    }

    float dxy = stop_latched ? ESTOP_DECEL_XY : DECEL_XY;
    float dw  = stop_latched ? ESTOP_DECEL_W : DECEL_W;
    current_vx = move_towards(current_vx, target_vx, ACCEL_XY, dxy, dt_smooth);
    current_vy = move_towards(current_vy, target_vy, ACCEL_XY, dxy, dt_smooth);
    current_w  = move_towards(current_w,  target_w,  ACCEL_W,  dw,  dt_smooth);

    // wheel i at angle t drives tangentially: v_i = -sin(t)*vx + cos(t)*vy + R*w
    const float Rw = ROBOT_CENTER_TO_WHEEL_RADIUS * current_w;
    float vFront = SGN_45  * ( S2 * (-current_vx + current_vy) + Rw);
    float vLeft  = SGN_135 * ( S2 * (-current_vx - current_vy) + Rw);
    float vBack  = SGN_225 * ( S2 * ( current_vx - current_vy) + Rw);
    float vRight = SGN_315 * ( S2 * ( current_vx + current_vy) + Rw);

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

  // undo the per-wheel signs, then invert the X-drive kinematics
  float s45 = SGN_45 * sFront, s135 = SGN_135 * sLeft, s225 = SGN_225 * sBack, s315 = SGN_315 * sRight;
  float dx_b = S2 * (-s45 - s135 + s225 + s315) * 0.5f;
  float dy_b = S2 * ( s45 - s135 - s225 + s315) * 0.5f;
  float dyaw = (s45 + s135 + s225 + s315) * (1.0f / (4.0f * ROBOT_CENTER_TO_WHEEL_RADIUS));

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
    Serial.print(vel_w, 3); Serial.print(',');
    Serial.println((int)fw_flags);
    fw_flags &= ~(SL_FLAG_CLAMP | SL_FLAG_WHEELSCALE | SL_FLAG_MALFORMED);   // report each event once; faults stay
  }
}
