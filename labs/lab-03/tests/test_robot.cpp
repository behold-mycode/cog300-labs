#include <cassert>
#include <iostream>
// Match Arduino concatenation: main sketch first, remaining tabs alphabetically.
#include "../src/MyRobot-V1/MyRobot-V1.ino"
#include "../src/MyRobot-V1/Motor.ino"
#include "../src/MyRobot-V1/Serial.ino"
void tick(uint32_t ms) { nowMs += ms; loop(); }
void send(const std::string& s) {
  for (char c : s) Serial.input.push_back(c);
  while (Serial.available()) loop();
}
void stopped() { assert(pwm[9] == 0 && pwm[10] == 0); }
void start(const std::string& s) { send(s); tick(250); }
int main() {
  setup();
  for (int pin : {9,8,7,10,6,5}) assert(modes[pin] == OUTPUT);
  stopped();
  start("F 180 500\n");
  assert(pwm[9] == 180 && pwm[10] == 180);
  assert(levels[8] == LOW && levels[7] == HIGH);
  assert(levels[6] == HIGH && levels[5] == LOW); // Right motor is inverted.
  tick(499); assert(pwm[9] == 180);
  tick(1); stopped();
  start("B 160 500\r\n");
  assert(levels[8] == HIGH && levels[7] == LOW);
  send("S\n"); stopped();
  start("L 200 500\n"); assert(pwm[9] == 100 && pwm[10] == 200);
  start("R 200 500\n"); assert(pwm[9] == 200 && pwm[10] == 100);
  start("Q 180 500\n"); assert(levels[8] == HIGH && levels[6] == HIGH);
  start("E 180 500\n"); assert(levels[8] == LOW && levels[6] == LOW);
  start("M -150 0 500\n"); assert(pwm[9] == 150 && pwm[10] == 0);
  send("!"); stopped(); send("\n");
  for (const char* bad : {"F 256 500\n", "F -1 500\n", "F 180 0\n", "F 180 5001\n", "F 180x 500\n", "F 180\n", "F 180 500 extra\n", "X\n", "M -256 0 500\n", "F 999999999999999999 500\n"}) {
    start("F 180 500\n"); send(bad); stopped(); tick(250); stopped();
  }
  send("F 180 500\n"); stopped(); send("S\n"); tick(300); stopped();
  start("F 180 500\n"); send("B 180 500\n"); stopped();
  tick(249); stopped(); tick(1); assert(pwm[9] == 180 && levels[8] == HIGH);
  send(std::string(100, 'x') + "\n"); stopped();
  start("F 180 5000\n"); send("F 1"); tick(1000); stopped();
  send("\n");
  nowMs = UINT32_MAX - 300;
  start("F 180 500\n"); tick(500); stopped();
  start("f 120 500\n"); assert(pwm[9] == 120); send("s\n"); stopped();
  // Named presets use the configured default duration.
  start("LEFT_FORWARD\n"); assert(pwm[9] == DEFAULT_TEST_PWM && pwm[10] == 0);
  tick(DEFAULT_TEST_MS); stopped();
  // Full suite: four isolated wheel tests, forward/reverse, curves, spins.
  const int left[]  = {180,-180,0,0,180,-180,90,180,-180,180};
  const int right[] = {0,0,180,-180,180,-180,180,90,180,-180};
  send("T 180 300\n");
  for (int i = 0; i < 10; ++i) {
    tick(250);
    assert(pwm[9] == abs(left[i]) && pwm[10] == abs(right[i]));
    if (left[i]) assert(levels[8] == (left[i] < 0 ? HIGH : LOW));
    if (right[i]) assert(levels[6] == (right[i] < 0 ? LOW : HIGH));
    tick(300); stopped();
    tick(999); stopped();
    tick(1); stopped(); // next step is only queued; settling precedes motion
  }
  tick(10000); stopped();
  // Stop in motion and in the inter-test pause must cancel future steps.
  start("T\n"); send("S\n"); tick(3000); stopped();
  start("T\n"); tick(DEFAULT_TEST_MS); send("S\n"); tick(3000); stopped();
  start("T\n"); send("invalid\n"); tick(3000); stopped();
  start("T\n"); send("!\n"); tick(3000); stopped();
  start("T\n"); send("?\n"); tick(3000); stopped();
  start("T\n"); start("M 0 150 200\n");
  assert(pwm[9] == 0 && pwm[10] == 150); tick(200); tick(3000); stopped();
  for (const char* bad : {"T 0 500\n", "T 256 500\n", "T 180\n", "T 180 5001\n", "1 extra\n"}) {
    send(bad); tick(3000); stopped();
  }
  // Suite gap and movement timers also survive millis() wraparound.
  nowMs = UINT32_MAX - 300;
  start("T 180 100\n"); tick(100); stopped(); tick(1000); tick(250);
  assert(pwm[9] == 180 && levels[8] == HIGH); send("S\n");
  // Per-wheel trim changes actual PWM, preserves reverse and zero, and caps at 255.
  send("P 100 125\n"); start("F 180 500\n");
  assert(pwm[9] == 180 && pwm[10] == 225);
  send("P 80 125\n"); stopped(); tick(1000); stopped();
  start("M -180 -220 500\n");
  assert(pwm[9] == 144 && pwm[10] == 255);
  assert(levels[8] == HIGH && levels[6] == LOW);
  start("M 0 180 500\n"); assert(pwm[9] == 0 && pwm[10] == 225);
  for (const char* bad : {"P 0 100\n", "P 100 201\n", "P 100\n", "P -1 100\n", "P 100 125 extra\n"}) {
    send(bad); stopped();
  }
  send("P 100 100\n"); start("F 180 500\n");
  assert(pwm[9] == 180 && pwm[10] == 180); send("S\n");
  const char* names[] = {"LEFT_FORWARD", "LEFT_REVERSE", "RIGHT_FORWARD", "RIGHT_REVERSE",
    "FORWARD", "BACKWARD", "CURVE_LEFT", "CURVE_RIGHT", "SPIN_LEFT", "SPIN_RIGHT"};
  for (int i = 0; i < 10; ++i) {
    start(std::string(names[i]) + " 180 300\n");
    assert(pwm[9] == abs(left[i]) && pwm[10] == abs(right[i]));
    tick(300); stopped();
  }
  start("right_forward 170 200\n"); assert(pwm[9] == 0 && pwm[10] == 170);
  send("STOP\n"); stopped();
  start("TEST_ALL 180 200\n"); assert(pwm[9] == 180);
  send("HELP\n"); tick(2000); stopped();
  for (const char* bad : {"1\n", "RIGHT_FORWARD 180\n", "SPIN_RIGHT 256 100\n", "FORWARD 180 0\n"}) {
    send(bad); tick(1000); stopped();
  }
  // Keyboard streaming must refresh movement without restarting the settling delay.
  send("HELLO\n"); send("CONFIG 100 100 0 1\n");
  send("DRIVE 180 180\n");
  tick(100); send("DRIVE 180 180\n");
  tick(100); send("DRIVE 180 180\n");
  tick(50); assert(pwm[9] == 180 && pwm[10] == 180);
  for (int i = 0; i < 10; ++i) { tick(100); send("DRIVE 180 180\n"); }
  assert(pwm[9] == 180 && pwm[10] == 180);
  send("DRIVE 90 180\n"); assert(pwm[9] == 90 && pwm[10] == 180);
  send("STOP\n"); stopped(); tick(1000); stopped();
  // Connection loss cancels all motion, including a running suite. Late frames cannot restart it.
  send("HELLO\n"); start("T 180 2000\n");
  send("HEARTBEAT\n"); tick(500); stopped();
  send("DRIVE 180 180\n"); tick(250); stopped();
  tick(3000); stopped();
  // Explicit reconnect and configuration changes stop outputs before changing direction.
  send("HELLO\n"); start("DRIVE 180 180\n");
  send("CONFIG 100 100 0 0\n"); stopped();
  start("DRIVE 0 180\n"); assert(pwm[9] == 0 && pwm[10] == 180 && levels[6] == LOW);
  send("DRIVE 0 -180\n"); stopped();
  tick(200); send("DRIVE 0 -180\n"); tick(50);
  assert(pwm[10] == 180 && levels[6] == HIGH);
  send("CONFIG 0 100 0 1\n"); stopped();
  send("HELLO\n"); send("CONFIG 100 100 0 1\n");
  start("LEFT 180 200\n"); assert(levels[8] == HIGH && levels[6] == HIGH);
  send("STOP\n");
  send("HELLO\n"); send("TEST 42 180 0 300\n"); tick(250);
  assert(pwm[9] == 180 && pwm[10] == 0 && remoteTestId == 42);
  send("HEARTBEAT\n"); tick(300); stopped(); assert(remoteTestId == 0);
  send("TEST 43 0 180 300\n"); send("STOP\n"); tick(250); stopped();
  assert(remoteTestId == 0);
  std::cout << "PASS: startup, movements, wheel isolation, timed stop, stop/cancel, reversal pause, invalid input, overflow, partial input timeout, millis rollover, suite order/timing/cancellation\n";
}
