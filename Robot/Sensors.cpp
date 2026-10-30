#include "Robot.h"

// Alternate sensors every 70 ms; never block waiting for an echo.
const uint8_t FRONT_SENSOR = 0, RIGHT_SENSOR = 1;
const uint8_t triggerPins[] = {TRIG_PIN, RIGHT_TRIG_PIN};
const uint8_t echoPins[] = {ECHO_PIN, RIGHT_ECHO_PIN};
struct DistanceSample {
  bool valid = false;
  int cm = -1;
  uint32_t echoUs = 0;
  uint32_t sampledMs = 0;
  uint32_t sequence = 0;
};
DistanceSample samples[2];
uint8_t nextSensor = FRONT_SENSOR;
volatile uint8_t activeSensor = FRONT_SENSOR;
uint32_t lastPingMs = 0, pingStartUs = 0;
bool pingPending = false;
volatile bool echoArmed = false, echoRisen = false, echoReady = false;
volatile uint32_t echoStartUs = 0, echoWidthUs = 0;

void captureEcho(uint8_t sensor) {
  if (!echoArmed || sensor != activeSensor) return;
  uint32_t now = micros();
  if (digitalRead(echoPins[sensor]) == HIGH) {
    echoStartUs = now; echoRisen = true;
  } else if (echoRisen) {
    echoWidthUs = (uint32_t)(now - echoStartUs);
    echoReady = true; echoArmed = false;
  }
}
void echoChanged() { captureEcho(FRONT_SENSOR); }
void rightEchoChanged() { captureEcho(RIGHT_SENSOR); }

void acceptReading(uint8_t sensor, int cm, uint32_t width) {
  ++samples[sensor].sequence;
  samples[sensor].sampledMs = millis();
  samples[sensor].valid = cm >= 2 && cm <= 400;
  samples[sensor].cm = samples[sensor].valid ? cm : -1;
  samples[sensor].echoUs = samples[sensor].valid ? width : 0;
  if (sensor == FRONT_SENSOR) {
    digitalWrite(GREEN_LED, samples[sensor].valid ? HIGH : LOW);
  }
}


void setupSensor() {
  for (uint8_t sensor = 0; sensor < 2; ++sensor) {
    digitalWrite(triggerPins[sensor], LOW); pinMode(triggerPins[sensor], OUTPUT);
    pinMode(echoPins[sensor], INPUT);
  }
  pinMode(GREEN_LED, OUTPUT); digitalWrite(GREEN_LED, LOW);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ECHO_PIN), rightEchoChanged, CHANGE);
  lastPingMs = millis() - PING_MS;
}

void serviceSensor() {
  uint32_t now = millis();
  bool ready; uint32_t width;
  noInterrupts();
  ready = echoReady; width = echoWidthUs; echoReady = false;
  interrupts();
  if (pingPending && ready) {
    pingPending = false;
    acceptReading(activeSensor, width >= 116 && width <= 23200 ? (int)(width / 58) : -1, width);
  } else if (pingPending && (uint32_t)(micros() - pingStartUs) >= ECHO_TIMEOUT_US) {
    noInterrupts(); echoArmed = false; echoReady = false; interrupts();
    pingPending = false; acceptReading(activeSensor, -1, 0);
  }
  for (uint8_t sensor = 0; sensor < 2; ++sensor) {
    if (samples[sensor].valid && (uint32_t)(millis() - samples[sensor].sampledMs) >= STALE_MS)
      acceptReading(sensor, -1, 0);
  }
  if (!pingPending && (uint32_t)(now - lastPingMs) >= PING_MS) {
    lastPingMs = now;
    noInterrupts(); echoArmed = false; activeSensor = nextSensor; interrupts();
    nextSensor = nextSensor == FRONT_SENSOR ? RIGHT_SENSOR : FRONT_SENSOR;
    // A failed sensor still yields its next slot to the other sensor.
    if (digitalRead(echoPins[activeSensor]) == HIGH) { acceptReading(activeSensor, -1, 0); return; }
    noInterrupts(); echoRisen = false; echoReady = false; echoArmed = true; interrupts();
    pingStartUs = micros(); pingPending = true;
    digitalWrite(triggerPins[activeSensor], HIGH); delayMicroseconds(10);
    digitalWrite(triggerPins[activeSensor], LOW);
  }
}

float sensorDistanceCm(uint8_t sensor) {
  if (sensor >= 2 || !samples[sensor].valid ||
      (uint32_t)(millis() - samples[sensor].sampledMs) >= STALE_MS) return -1;
  return samples[sensor].echoUs / 58.0f;
}

// Expose acquisition time/identity so control never differentiates loop repeats.
SensorReading sensorReading(uint8_t sensor) {
  if (sensor >= 2) return {-1, 0, 0};
  return {sensorDistanceCm(sensor), samples[sensor].sampledMs, samples[sensor].sequence};
}
