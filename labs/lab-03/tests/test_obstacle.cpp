// Regression: sensor acquisition remains active, but cannot gate or cancel motion.
#include <cassert>
#include <iostream>
#include "../src/MyRobot-V2/MyRobot-V2.ino"
#include "../src/MyRobot-V2/Motor.ino"
#include "../src/MyRobot-V2/Serial.ino"
#include "../src/MyRobot-V2/Safety.ino"
#include "../src/MyRobot-V2/Navigation.ino"
void command(const char* text) { char b[64]; strncpy(b,text,sizeof(b)); processCommand(b); }
void start(const char* text) { command(text); nowMs += 250; serviceMotion(); }
void echoPulse(uint32_t width) {
  levels[ECHO_PIN] = HIGH; echoChanged();
  extraUs += width; levels[ECHO_PIN] = LOW; echoChanged(); serviceSensor();
}
int main() {
  setup();
  assert(!samples[FRONT_SENSOR].valid && levels[GREEN_LED] == LOW);
  for (const char* cmd : {"F 180 1000", "B 180 1000", "L 180 1000", "R 180 1000",
       "Q 180 1000", "E 180 1000", "M 0 180 1000", "TEST_ALL 180 1000"}) {
    acceptDistance(-1); start(cmd); assert(running);
    int left = pwm[ENA], right = pwm[ENB];
    assert(left || right);
    acceptDistance(2); assert(samples[FRONT_SENSOR].valid && levels[GREEN_LED] == HIGH);
    acceptDistance(-1); assert(!samples[FRONT_SENSOR].valid && levels[GREEN_LED] == LOW);
    assert(running && pwm[ENA] == left && pwm[ENB] == right);
    command("STOP"); assert(!running && pwm[ENA] == 0 && pwm[ENB] == 0);
  }
  // Pending movement and suite gaps also survive invalid readings.
  command("T 180 100"); acceptDistance(-1); assert(pending && suiteActive);
  nowMs += 250; serviceMotion(); nowMs += 100; serviceMotion();
  assert(testWaiting); acceptDistance(-1); nowMs += 1000; serviceTestSuite();
  assert(pending && suiteActive); command("STOP");
  start("F 180 1000"); acceptDistance(50); nowMs += STALE_MS + 1; serviceSensor();
  assert(!samples[FRONT_SENSOR].valid && running); // Stale sample does not stop movement.
  command("STOP");
  pingPending = false; nextSensor = FRONT_SENSOR; nowMs += 70; serviceSensor(); echoPulse(1160);
  assert(samples[FRONT_SENSOR].cm == 20 && samples[FRONT_SENSOR].valid);
  nextSensor = FRONT_SENSOR; nowMs += 70; serviceSensor(); nowMs += 31; serviceSensor(); assert(!samples[FRONT_SENSOR].valid);
  levels[ECHO_PIN] = HIGH; nowMs += 70; serviceSensor(); assert(!samples[FRONT_SENSOR].valid);
  levels[ECHO_PIN] = LOW;
  activeSensor = FRONT_SENSOR; pingPending = true; echoArmed = true; echoRisen = false;
  extraUs = UINT32_MAX - nowMs * 1000U - 500U;
  echoPulse(1160); assert(samples[FRONT_SENSOR].cm == 20);
  acceptDistance(0); assert(!samples[FRONT_SENSOR].valid); acceptDistance(401); assert(!samples[FRONT_SENSOR].valid);
  command("HELLO"); start("TEST 42 180 180 1000");
  acceptDistance(-1); assert(running && remoteTestId == 42);
  nowMs += 501; serviceControlTimeout(); assert(!running && remoteTestId == 0);
  assert(Serial.output.str().find("SAFETY ") == std::string::npos);
  assert(Serial.output.str().find("MOTION_BLOCKED") == std::string::npos);
  std::cout << "PASS: logging-only sensors, acquisition, LED validity, manual/timed/control stops\n";
}
