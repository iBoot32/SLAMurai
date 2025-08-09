#include <AccelStepper.h>
#include <math.h>

#define ENABLE_PIN 8
#define DISABLE_ALL_STEPPERS 0
#define STEPS_PER_FULL_REV ((long)200)
#define WHEEL_RADIUS_M 0.05  // 50 mm

#define RPM_MAX 150
#define STEPS_PER_SEC_MAX ((RPM_MAX) / 60.0 * (STEPS_PER_FULL_REV))

#define ODOM_DT 5
uint32_t last_time_send_odom = 0;

struct Stepper {
  uint8_t stepPin;
  uint8_t dirPin;
  AccelStepper driver;

  Stepper(uint8_t step, uint8_t dir)
    : stepPin(step), dirPin(dir), driver(AccelStepper::DRIVER, step, dir) {}

  void setup() {
    driver.setMaxSpeed(STEPS_PER_SEC_MAX);
  }

  void setLinearVelocity(double v_mps) {
    double sps = (v_mps / (2.0 * M_PI * WHEEL_RADIUS_M)) *
                 (double)STEPS_PER_FULL_REV;

    if (fabs(sps) > STEPS_PER_SEC_MAX)
      sps = copysign(STEPS_PER_SEC_MAX, sps);

    driver.setSpeed(sps);  // signed speed in steps/sec
  }

  void run() { driver.runSpeed(); }  // constant speed
};

Stepper stepperX = {2, 5};

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  if (DISABLE_ALL_STEPPERS) { 
    digitalWrite(ENABLE_PIN, HIGH); 
    return; 
  }
  digitalWrite(ENABLE_PIN, LOW);

  stepperX.setup();
}

void loop() {
  uint32_t now = millis();

  // Send odom at fixed rate
  if (now - last_time_send_odom >= ODOM_DT) {
    last_time_send_odom += ODOM_DT;
    long curr_step = stepperX.driver.currentPosition();
    double meters = (curr_step / (double)STEPS_PER_FULL_REV) *
                    2 * M_PI * WHEEL_RADIUS_M;
    Serial.println(meters);
  }

  // Read cmd_vel whenever it's ready
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    double value = line.toFloat();
    Serial.println(value, 2);
    stepperX.setLinearVelocity(value);
  }

  stepperX.run();  // now calls runSpeed()
}
