#include <cassert>
#include <iostream>
#include "../Robot/Robot.h"
#include "../Robot/Sensors.cpp"
void pulse(uint8_t sensor, uint32_t width) {
  levels[echoPins[sensor]] = HIGH; captureEcho(sensor);
  extraUs += width; levels[echoPins[sensor]] = LOW; captureEcho(sensor);
}
void slot(uint32_t time, uint8_t expected) {
  nowMs = time; serviceSensor(); assert(pingPending && activeSensor == expected);
}
int main() {
  setupSensor(); slot(0, FRONT_SENSOR);
  pulse(RIGHT_SENSOR,1160); serviceSensor(); assert(pingPending && !samples[0].valid);
  pulse(FRONT_SENSOR,1752); serviceSensor(); assert(samples[0].cm == 30);
  nowMs = 69; serviceSensor(); assert(!pingPending);
  slot(70,RIGHT_SENSOR); pulse(RIGHT_SENSOR,2900); serviceSensor(); assert(samples[1].cm == 50);
  slot(140,FRONT_SENSOR); nowMs = 171; serviceSensor();
  assert(!samples[0].valid && samples[1].valid);
  slot(210,RIGHT_SENSOR); pulse(RIGHT_SENSOR,1160); serviceSensor();
  levels[ECHO_PIN] = HIGH; nowMs = 280; serviceSensor(); assert(!pingPending);
  slot(350,RIGHT_SENSOR); pulse(RIGHT_SENSOR,5800); serviceSensor();
  levels[ECHO_PIN] = LOW; nowMs = 701; acceptReading(0,30,1740); serviceSensor();
  assert(samples[0].valid && !samples[1].valid);
  activeSensor = RIGHT_SENSOR; pingPending = true; echoArmed = true; echoRisen = false;
  extraUs = UINT32_MAX - nowMs * 1000U - 500U;
  pulse(RIGHT_SENSOR,1160); serviceSensor(); assert(samples[1].cm == 20);
  acceptReading(0,1,58); assert(sensorDistanceCm(0) == -1);
  acceptReading(0,401,23258); assert(sensorDistanceCm(0) == -1);
  acceptReading(0,20,1160); assert(levels[GREEN_LED] == HIGH);
  nowMs += STALE_MS; assert(sensorDistanceCm(0) == -1);
  // Echo completion straddling a millisecond boundary must remain valid.
  nowMs = 1000; extraUs = 0; millisStep = 1;
  activeSensor = FRONT_SENSOR; pingPending = true; echoReady = true; echoWidthUs = 1740;
  serviceSensor(); assert(samples[0].valid && samples[0].cm == 30);
  millisStep = 0;
  std::cout << "PASS sensors: alternating pings, crosstalk, timeout, stale data, range, rollover\n";
}
