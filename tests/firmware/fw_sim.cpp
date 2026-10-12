// Runs the real motor_controller_xdrive_safe.ino against stubs and checks the safety behaviour from the outside, by
// feeding serial lines and reading the odometry lines it prints.
#include "stubs/Arduino.h"
unsigned long g_micros = 0; int g_bumper_level = 1; SerialStub Serial; double g_max_wheel_sps = 0;
#include "stubs/MobaTools.h"
#include "../../control/motor_controller_xdrive_safe/motor_controller_xdrive_safe.ino"
#include <vector>
#include <sstream>

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok:   %s\n", msg); } while (0)

struct Odom { double x, y, yaw, vx, vy, w; int flags; };
static std::vector<Odom> parse_odom() {
  std::vector<Odom> v; std::istringstream is(Serial.tx); std::string line;
  while (std::getline(is, line)) {
    Odom o;
    if (sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%d", &o.x, &o.y, &o.yaw, &o.vx, &o.vy, &o.w, &o.flags) == 7) v.push_back(o);
  }
  return v;
}
static void run(double seconds, const char* cmd_each_40ms = nullptr) {
  unsigned long end = g_micros + (unsigned long)(seconds * 1e6), next = g_micros;
  while (g_micros < end) {
    if (cmd_each_40ms && g_micros >= next) { Serial.rx += cmd_each_40ms; next += 40000; }
    g_micros += 500; loop();
  }
}
static void reset() { Serial.tx.clear(); }
static Odom last() { auto v = parse_odom(); return v.empty() ? Odom{0, 0, 0, 0, 0, 0, 0} : v.back(); }

int main() {
  setup();
  // 1. absurd command is limited
  run(0.1);
  run(4.0, "5.0,5.0,50.0\n");
  Odom o = last();
  CHECK(hypot(o.vx, o.vy) <= V_MAX_XY * 1.02, "5 m/s command is limited to V_MAX_XY");
  CHECK(fabs(o.w) <= W_MAX * 1.02, "50 rad/s command is limited to W_MAX");
  CHECK(g_max_wheel_sps <= STEPS_PER_SEC_MAX + 1e-6, "no wheel was ever told to exceed STEPS_PER_SEC_MAX");
  CHECK(hypot(o.vx, o.vy) > 0.12, "limited command still moves the robot (direction kept)");
  CHECK(fabs(atan2(o.vy, o.vx) - M_PI / 4) < 0.15, "direction of travel preserved by the common wheel scale");
  // 2. silence -> timeout stop within 0.25 s + ramp
  reset(); run(1.2);
  o = last(); CHECK(hypot(o.vx, o.vy) < 0.01 && fabs(o.w) < 0.01, "command timeout stops the robot");
  // 3. malformed lines are dropped
  reset(); run(0.3, "nan,0,0\n"); run(0.3, "abc\n"); run(0.3, "1,2\n"); run(0.3, "1,2,3,4\n"); run(0.3, "0.2,0,0 junk\n");
  o = last(); CHECK(hypot(o.vx, o.vy) < 0.01, "malformed / non-finite lines never move the robot");
  bool saw = false; for (auto& a : parse_odom()) if (a.flags & SL_FLAG_MALFORMED) saw = true;
  CHECK(saw, "malformed lines are reported in the odometry flags");
  // 4. a valid command still works afterwards
  reset(); run(2.0, "0.15,0.0,0.0\n"); o = last(); CHECK(o.vx > 0.12 && fabs(o.vy) < 0.02, "plain forward command works");
  // 5. tether: keep driving forward until the latch
  int tether_seen = 0; double maxdist = 0;
  for (int i = 0; i < 80; i++) {
    reset(); run(1.0, "0.21,0.0,0.0\n");
    o = last(); maxdist = fmax(maxdist, hypot(o.x, o.y)); if (o.flags & SL_FLAG_TETHER) { tether_seen = 1; break; }
  }
  CHECK(tether_seen, "tether latches when the robot leaves the allowed radius");
  CHECK(maxdist < TETHER_M + 0.6, "robot stops within a stopping distance of the tether");
  reset(); run(3.0, "0.21,0.0,0.0\n"); o = last();
  CHECK(hypot(o.vx, o.vy) < 0.01 && (o.flags & SL_FLAG_TETHER), "after the latch further commands are ignored");
  printf(fails ? "FIRMWARE TESTS FAILED\n" : "FIRMWARE TESTS PASSED\n");
  return fails ? 1 : 0;
}
