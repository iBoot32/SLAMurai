#pragma once
#include "Arduino.h"
enum { STEPDIR = 1 };
extern double g_max_wheel_sps;        // highest |steps/s| any stepper was ever told to run
class MoToStepper {
 public:
  MoToStepper(int, int) {}
  void attach(int, int) {}
  void setRampLen(int) {}
  void setSpeed(int) {}
  void setSpeedSteps(unsigned s10) { advance(); sps = s10 / 10.0; if (sps > g_max_wheel_sps) g_max_wheel_sps = sps; }
  void rotate(int d) { advance(); dir = d; }
  long readSteps() const { const_cast<MoToStepper*>(this)->advance(); return (long)pos; }
 private:
  void advance() const { double dt = (g_micros - last) * 1e-6; pos += dir * sps * dt; last = g_micros; }
  mutable double pos = 0; mutable unsigned long last = 0; double sps = 0; int dir = 0;
};
