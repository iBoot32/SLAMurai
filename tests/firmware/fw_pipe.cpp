// The REAL motor_controller_xdrive_safe.ino behind a pipe, so the real serial bridge (autonav/nav/src/main.py) can talk to it.
// stdin:  "W<text>"  append <text>+'\n' to the serial receive buffer      "R<us>"  run loop() for <us> microseconds (500 us steps)
// stdout: for every R, each odometry line the firmware printed as "O,<line>", then "E".
#include "stubs/Arduino.h"
unsigned long g_micros = 0; int g_bumper_level = 1; SerialStub Serial; double g_max_wheel_sps = 0;
#include "stubs/MobaTools.h"
#include "../../control/motor_controller_xdrive_safe/motor_controller_xdrive_safe.ino"
#include <iostream>
#include <sstream>

int main() {
  setup();
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    if (line[0] == 'W') {
      Serial.rx += line.substr(1) + "\n";
    } else if (line[0] == 'R') {
      unsigned long end = g_micros + (unsigned long)atol(line.c_str() + 1);
      while (g_micros < end) { g_micros += 500; loop(); }
      std::istringstream is(Serial.tx); std::string l;
      while (std::getline(is, l)) std::cout << "O," << l << "\n";
      std::cout << "E" << std::endl;
      Serial.tx.clear();
    }
  }
}
