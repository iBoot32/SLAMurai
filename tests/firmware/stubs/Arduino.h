// Minimal host stand-in for the Arduino core, enough to compile and RUN the real .ino on a PC.
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <string>
#include <stdio.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define LOW 0
#define HIGH 1
#define INPUT_PULLUP 2
#define OUTPUT 1
extern unsigned long g_micros;
inline unsigned long micros() { return g_micros; }
inline void delayMicroseconds(unsigned) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
extern int g_bumper_level;
inline int digitalRead(int) { return g_bumper_level; }
struct SerialStub {
  std::string rx; size_t pos = 0; std::string tx;
  void begin(long) {}
  int available() { return (int)(rx.size() - pos); }
  int read() { return (unsigned char)rx[pos++]; }
  void print(float v, int d = 2) { char b[48]; snprintf(b, sizeof b, "%.*f", d, v); tx += b; }
  void print(double v, int d = 2) { print((float)v, d); }
  void print(char c) { tx += c; }
  void print(int v) { tx += std::to_string(v); }
  void println(int v) { tx += std::to_string(v); tx += '\n'; }
  void println(float v) { print(v); tx += '\n'; }
};
extern SerialStub Serial;
