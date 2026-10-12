// safety_limits.h - the firmware's last line of defence, as plain functions with no Arduino dependencies so the same
// file is compiled and tested on a PC (tests/firmware/).  Everything here is a compile-time limit: it can
// only be changed by editing this file and reflashing, never over the serial link.
#ifndef SAFETY_LIMITS_H
#define SAFETY_LIMITS_H

#include <math.h>

#define SL_FLAG_TETHER     1    // moved farther than TETHER_M from the power-up point: latched stop
#define SL_FLAG_TIMEOUT    2    // no valid command for CMD_TIMEOUT: target forced to zero (self-clears)
#define SL_FLAG_CLAMP      4    // a command was reduced to the limits
#define SL_FLAG_WHEELSCALE 8    // all wheels were scaled together to respect the wheel-speed cap
#define SL_FLAG_MALFORMED  16   // a serial line was rejected (not three finite numbers)
#define SL_FLAG_BUMPER     32   // bumper switch pressed: latched stop

struct SlParams {
  float v_max;        // m/s, translation magnitude
  float w_max;        // rad/s
  float wheel_cap;    // m/s, rim speed of the fastest wheel
  float R;            // m, wheel-centre radius
};

struct SlCmd { float vx, vy, w; };

static inline bool sl_finite(float x) { return (x == x) && (x - x == 0.0f); }

// x-drive: wheel at azimuth t drives v = -sin(t) vx + cos(t) vy + R w; azimuths 45/135/225/315 deg.
static inline float sl_max_wheel(const SlCmd& c, float R) {
  const float S2 = 0.70710678f;
  float a = S2 * (-c.vx + c.vy) + R * c.w;     // 45
  float b = S2 * (-c.vx - c.vy) + R * c.w;     // 135
  float d = S2 * ( c.vx - c.vy) + R * c.w;     // 225
  float e = S2 * ( c.vx + c.vy) + R * c.w;     // 315
  float m = fabsf(a);
  if (fabsf(b) > m) m = fabsf(b);
  if (fabsf(d) > m) m = fabsf(d);
  if (fabsf(e) > m) m = fabsf(e);
  return m;
}

// Returns a command that satisfies |v| <= v_max, |w| <= w_max and every wheel <= wheel_cap for ANY input
// (NaN, inf and absurd magnitudes included).  Direction of travel is preserved: the speed is scaled, never one wheel.
static inline SlCmd sl_limit(SlCmd in, const SlParams& p, unsigned char* flags) {
  SlCmd z = {0.0f, 0.0f, 0.0f};
  if (!sl_finite(in.vx) || !sl_finite(in.vy) || !sl_finite(in.w)) { *flags |= SL_FLAG_MALFORMED; return z; }
  // bring huge values into a range where hypotf cannot overflow before scaling
  const float BIG = 1.0e6f;
  if (fabsf(in.vx) > BIG) in.vx = copysignf(BIG, in.vx);
  if (fabsf(in.vy) > BIG) in.vy = copysignf(BIG, in.vy);
  if (fabsf(in.w)  > BIG) in.w  = copysignf(BIG, in.w);
  SlCmd c = in;
  float s = hypotf(c.vx, c.vy);
  if (s > p.v_max) { float k = p.v_max / s; c.vx *= k; c.vy *= k; *flags |= SL_FLAG_CLAMP; }
  if (fabsf(c.w) > p.w_max) { c.w = copysignf(p.w_max, c.w); *flags |= SL_FLAG_CLAMP; }
  float m = sl_max_wheel(c, p.R);
  if (m > p.wheel_cap) { float k = p.wheel_cap / m; c.vx *= k; c.vy *= k; c.w *= k; *flags |= SL_FLAG_WHEELSCALE; }
  return c;
}

static inline bool sl_tether_exceeded(float x, float y, float x0, float y0, float radius) {
  float dx = x - x0, dy = y - y0;
  return (dx * dx + dy * dy) > radius * radius;
}

#endif
