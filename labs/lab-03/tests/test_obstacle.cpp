#include <cassert>
#include <iostream>
#include "../src/MyRobot-V1/MyRobot-V1.ino"
#include "../src/MyRobot-V1/Motor.ino"
#include "../src/MyRobot-V1/Serial.ino"
#include "../src/MyRobot-V1/Safety.ino"
void command(const char* text) {
  char buffer[64]; strncpy(buffer, text, sizeof(buffer)); processCommand(buffer);
}
void stopped() { assert(pwm[9] == 0 && pwm[10] == 0); }
void clearFront() { for (int i=0; i<3; ++i) acceptDistance(50); }
void start(const char* text) { command(text); nowMs += 250; serviceMotion(); }
void echoPulse(uint32_t width) {
  levels[ECHO_PIN] = HIGH; echoChanged();
  extraUs += width; levels[ECHO_PIN] = LOW; echoChanged();
  serviceSafety();
}
int main() {
  setup();
  assert(levels[12] == HIGH && levels[11] == LOW);
  start("F 180 1000"); stopped(); assert(!pending);
  clearFront(); assert(levels[11] == HIGH && levels[12] == LOW);
  start("F 180 1000"); assert(pwm[9] == 180);
  acceptDistance(25); stopped(); assert(levels[12] == HIGH && levels[11] == LOW);
  // Every entry point, including spins and isolated positive wheels, is guarded.
  for (const char* text : {"F 180 1000", "L 180 1000", "R 180 1000", "Q 180 1000",
       "E 180 1000", "M 0 180 1000", "TEST_ALL 180 1000", "FORWARD 180 1000"}) {
    start(text); stopped(); assert(!pending && !suiteActive && activeTest == -1);
  }
  setDrive(180, 180); stopped(); // Final output gate cannot be bypassed.
  start("B 180 1000"); assert(pwm[9] == 180 && levels[8] == HIGH);
  assert(levels[12] == HIGH && levels[11] == LOW); command("STOP");
  // Hysteresis needs three consecutive clear samples, not one outlier.
  acceptDistance(36); acceptDistance(36); assert(frontBlocked);
  acceptDistance(30); acceptDistance(36); acceptDistance(36); assert(frontBlocked);
  acceptDistance(36); assert(!frontBlocked); stopped(); // No automatic restart.
  acceptDistance(30); assert(!frontBlocked);
  // Stop pending motion and a running autonomous suite; no later steps restart it.
  command("T 180 1000"); acceptDistance(10);
  nowMs += 250; serviceMotion(); stopped(); assert(!suiteActive);
  clearFront(); start("T 180 1000"); assert(pwm[9] == 180);
  acceptDistance(10); clearFront(); nowMs += 3000; serviceMotion(); serviceTestSuite();
  stopped(); assert(!suiteActive);
  clearFront(); start("T 180 100"); nowMs += 100; serviceMotion();
  assert(testWaiting); acceptDistance(10); clearFront();
  nowMs += 1000; serviceTestSuite(); stopped(); assert(!suiteActive && !pending);
  // Repeated live frames cannot defeat the guard. Remote test ID is not armed on rejection.
  command("HELLO"); acceptDistance(10);
  for (int i=0; i<5; ++i) { start("DRIVE 180 180"); stopped(); }
  command("TEST 42 180 180 1000"); assert(remoteTestId == 0); stopped();
  clearFront(); start("TEST 43 180 180 1000"); assert(remoteTestId == 43);
  acceptDistance(10); assert(remoteTestId == 0); stopped();
  // Loss of a valid sensor stops motion; zero/out-of-range are never clear.
  clearFront(); start("DRIVE 180 180"); acceptDistance(-1); stopped();
  assert(!sensorValid && frontBlocked);
  acceptDistance(0); assert(frontBlocked); acceptDistance(401); assert(frontBlocked);
  clearFront(); start("DRIVE 180 180");
  nowMs += 251; pingPending = false; serviceSafety(); stopped(); assert(!sensorValid);
  // Actual interrupt acquisition: 1160us is 20cm, and missing/stuck echo is a fault.
  pingPending = false; nowMs += 70; serviceSafety(); assert(pingPending);
  echoPulse(1160); assert(distanceCm == 20 && sensorValid && frontBlocked);
  for (int i=0; i<3; ++i) { nowMs += 70; serviceSafety(); echoPulse(2900); }
  assert(distanceCm == 50 && !frontBlocked);
  nowMs += 70; serviceSafety(); nowMs += 31; serviceSafety();
  assert(!sensorValid && frontBlocked);
  levels[ECHO_PIN] = HIGH; nowMs += 70; serviceSafety(); assert(!sensorValid);
  levels[ECHO_PIN] = LOW;
  // A pulse spanning micros() wraparound is still measured correctly.
  pingPending = true; echoArmed = true; echoRisen = false;
  extraUs = UINT32_MAX - nowMs * 1000U - 500U;
  echoPulse(1160); assert(distanceCm == 20);
  std::cout << "PASS: sensor acquisition, LEDs, all motion gates, manual and auto cancellation, reverse escape, hysteresis, timeout, stale/stuck echo, timer wraparound\n";
}
