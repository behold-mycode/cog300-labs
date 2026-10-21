#include <cassert>
#include <iostream>
#include "../src/MyRobot-V2/MyRobot-V2.ino"
#include "../src/MyRobot-V2/Motor.ino"
#include "../src/MyRobot-V2/Serial.ino"
#include "../src/MyRobot-V2/Safety.ino"
void pulse(uint8_t sensor, uint32_t width) {
  levels[echoPins[sensor]] = HIGH; captureEcho(sensor);
  extraUs += width; levels[echoPins[sensor]] = LOW; captureEcho(sensor);
}
void slot(uint32_t time, uint8_t expected) {
  nowMs = time; serviceSensor(); assert(pingPending && activeSensor == expected);
}
int main() {
  setup(); slot(0, FRONT_SENSOR);
  pulse(RIGHT_SENSOR, 1160); serviceSensor(); // Wrong-side edges ignored.
  assert(pingPending && !samples[RIGHT_SENSOR].valid && !samples[FRONT_SENSOR].valid);
  pulse(FRONT_SENSOR, 1752); serviceSensor();
  assert(samples[FRONT_SENSOR].cm == 30 && samples[FRONT_SENSOR].echoUs == 1752);
  nowMs = 69; serviceSensor(); assert(!pingPending);
  slot(70, RIGHT_SENSOR); pulse(RIGHT_SENSOR, 2900); serviceSensor();
  assert(samples[RIGHT_SENSOR].cm == 50 && samples[FRONT_SENSOR].cm == 30);
  slot(140, FRONT_SENSOR); nowMs = 171; serviceSensor(); // Front missing; right stays valid.
  assert(!samples[FRONT_SENSOR].valid && samples[RIGHT_SENSOR].valid);
  slot(210, RIGHT_SENSOR); pulse(RIGHT_SENSOR, 1160); serviceSensor();
  assert(samples[RIGHT_SENSOR].cm == 20);
  levels[ECHO_PIN] = HIGH; nowMs = 280; serviceSensor();
  assert(!pingPending && !samples[FRONT_SENSOR].valid);
  slot(350, RIGHT_SENSOR); pulse(RIGHT_SENSOR, 5800); serviceSensor();
  assert(samples[RIGHT_SENSOR].cm == 100); levels[ECHO_PIN] = LOW;
  // Separate stale-data validity: only front is refreshed.
  nowMs = 701; acceptDistance(30); serviceSensor();
  assert(samples[FRONT_SENSOR].valid && !samples[RIGHT_SENSOR].valid);
  // Fractional front/right output and no motor effects.
  acceptReading(RIGHT_SENSOR, 50, 2906);
  distanceLogEnabled = true; lastDistanceLogMs = nowMs - DISTANCE_LOG_MS;
  Serial.output.str(""); serviceDistanceLog();
  assert(Serial.output.str() == "t=0.7s | front=30.0 cm OK | right=50.1 cm OK\n");
  assert(pwm[ENA] == 0 && pwm[ENB] == 0);
  // Right echo width is also robust across micros rollover.
  activeSensor = RIGHT_SENSOR; pingPending = true; echoArmed = true; echoRisen = false;
  extraUs = UINT32_MAX - nowMs * 1000U - 500U;
  pulse(RIGHT_SENSOR, 1160); serviceSensor(); assert(samples[RIGHT_SENSOR].cm == 20);
  std::cout << "PASS: dual-sensor scheduling, independent readings, failures, logs and rollover\n";
}
