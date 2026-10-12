// Property tests of safety_limits.h: for ANY float input the limited command satisfies every bound.
#include "../../control/motor_controller_xdrive_safe/safety_limits.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
static double rnd() { return rand() / (double)RAND_MAX; }
int main() {
  SlParams p = {0.21f, 0.5f, 0.2857f, 0.133f};
  const float special[] = {0.0f, -0.0f, 1e-30f, 0.21f, -0.21f, 1.0f, 1e3f, 1e6f, 1e20f, 3.0e38f, -3.0e38f, INFINITY, -INFINITY, NAN};
  int bad = 0; long n = 0;
  auto check = [&](SlCmd in) {
    unsigned char f = 0; SlCmd o = sl_limit(in, p, &f); n++;
    bool fin = sl_finite(o.vx) && sl_finite(o.vy) && sl_finite(o.w);
    bool ok = fin && hypotf(o.vx, o.vy) <= p.v_max * 1.0001f && fabsf(o.w) <= p.w_max * 1.0001f &&
              sl_max_wheel(o, p.R) <= p.wheel_cap * 1.0001f;
    bool finite_in = sl_finite(in.vx) && sl_finite(in.vy) && sl_finite(in.w);
    if (!finite_in) ok = ok && o.vx == 0 && o.vy == 0 && o.w == 0 && (f & SL_FLAG_MALFORMED);
    unsigned char f2 = 0; SlCmd o2 = sl_limit(o, p, &f2);        // idempotent
    ok = ok && fabsf(o2.vx - o.vx) < 1e-6f && fabsf(o2.vy - o.vy) < 1e-6f && fabsf(o2.w - o.w) < 1e-6f;
    if (finite_in && hypotf(in.vx, in.vy) < 0.2f && fabsf(in.w) < 0.4f && sl_max_wheel(in, p.R) < p.wheel_cap)
      ok = ok && o.vx == in.vx && o.vy == in.vy && o.w == in.w;   // inside the limits: untouched
    if (!ok) { bad++; if (bad < 10) printf("BAD in=(%g,%g,%g) out=(%g,%g,%g)\n", in.vx, in.vy, in.w, o.vx, o.vy, o.w); }
  };
  for (float a : special) for (float b : special) for (float c : special) check({a, b, c});
  srand(1);
  for (int i = 0; i < 2000000; i++) {
    double s = pow(10.0, -3 + 5 * rnd());
    check({(float)((rnd() * 2 - 1) * s), (float)((rnd() * 2 - 1) * s), (float)((rnd() * 2 - 1) * s * 3)});
  }
  SlCmd edge = {0.21f * 0.70710678f, -0.21f * 0.70710678f, 0.5f};
  unsigned char f = 0; SlCmd o = sl_limit(edge, p, &f);
  if (sl_max_wheel(o, p.R) > p.wheel_cap * 1.0001f) { bad++; printf("edge case violates cap\n"); }
  printf("%ld cases, %d bad\n", n, bad);
  printf(bad ? "LIMIT TESTS FAILED\n" : "LIMIT TESTS PASSED\n");
  return bad ? 1 : 0;
}
