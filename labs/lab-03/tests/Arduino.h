#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <deque>
#include <string>
#include <sstream>
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define F(x) x
static uint32_t nowMs = 0;
static int modes[20] = {}, levels[20] = {}, pwm[20] = {};
inline uint32_t millis() { return nowMs; }
inline void pinMode(int pin, int mode) { modes[pin] = mode; }
inline void digitalWrite(int pin, int value) { levels[pin] = value; }
inline void analogWrite(int pin, int value) { pwm[pin] = value; }
struct FakeSerial {
  std::deque<char> input;
  std::ostringstream output;
  void begin(int) {}
  int available() { return input.size(); }
  int read() { char c = input.front(); input.pop_front(); return c; }
  template<class T> void print(T value) { output << value; }
  template<class T> void println(T value) { output << value << "\n"; }
  void println() { output << "\n"; }
} Serial;
#define INPUT 0
#define CHANGE 2
static uint32_t extraUs = 0;
inline uint32_t micros() { return nowMs * 1000U + extraUs; }
inline void delayMicroseconds(unsigned int us) { extraUs += us; }
inline int digitalRead(int pin) { return levels[pin]; }
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void noInterrupts() {}
inline void interrupts() {}
