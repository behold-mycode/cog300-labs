// Two HC-SR04 sensors, measurement only. No sensor-based motor blocking.
// One ping every 70 ms, alternating sides: each sensor updates every 140 ms.
const uint32_t PING_MS = 70, ECHO_TIMEOUT_US = 30000, STALE_MS = 350;
const uint8_t FRONT_SENSOR = 0, RIGHT_SENSOR = 1;
const uint8_t triggerPins[] = {TRIG_PIN, RIGHT_TRIG_PIN};
const uint8_t echoPins[] = {ECHO_PIN, RIGHT_ECHO_PIN};
struct DistanceSample {
  bool valid = false;
  int cm = -1;
  uint32_t echoUs = 0;
  uint32_t sampledMs = 0;
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

void reportSensor() {
  // Retain the controller's front-reading status protocol.
  Serial.print(F("SENSOR "));
  Serial.print(samples[FRONT_SENSOR].valid ? F("VALID") : F("INVALID"));
  Serial.print(' '); Serial.println(samples[FRONT_SENSOR].cm);
}

void acceptReading(uint8_t sensor, int cm, uint32_t width) {
  bool wasValid = samples[sensor].valid;
  samples[sensor].sampledMs = millis();
  samples[sensor].valid = cm >= 2 && cm <= 400;
  samples[sensor].cm = samples[sensor].valid ? cm : -1;
  samples[sensor].echoUs = samples[sensor].valid ? width : 0;
  if (sensor == FRONT_SENSOR) {
    digitalWrite(GREEN_LED, samples[sensor].valid ? HIGH : LOW);
    if (wasValid != samples[sensor].valid) reportSensor();
  }
}

// Front sample injection used by deterministic host tests.
void acceptDistance(int cm) { acceptReading(FRONT_SENSOR, cm, cm >= 2 && cm <= 400 ? cm * 58UL : 0); }

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
    if (samples[sensor].valid && (uint32_t)(now - samples[sensor].sampledMs) >= STALE_MS)
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

const uint32_t DISTANCE_LOG_MS = 500;
bool distanceLogEnabled = false;
uint32_t lastDistanceLogMs = 0;

void setDistanceLog(bool enabled) {
  distanceLogEnabled = enabled;
  lastDistanceLogMs = millis();
  Serial.println(enabled ? F("Distance log ON (front/right cm, every 500 ms)") : F("Distance log OFF"));
}

void printDistance(uint8_t sensor) {
  if (samples[sensor].valid && (uint32_t)(millis() - samples[sensor].sampledMs) < STALE_MS) {
    uint32_t tenthsCm = (samples[sensor].echoUs * 10UL + 29UL) / 58UL;
    Serial.print(tenthsCm / 10); Serial.print('.'); Serial.print(tenthsCm % 10);
    Serial.print(F(" cm OK"));
  } else {
    Serial.print(F("-- INVALID"));
  }
}

void serviceDistanceLog() {
  uint32_t now = millis();
  if (!distanceLogEnabled || (uint32_t)(now - lastDistanceLogMs) < DISTANCE_LOG_MS) return;
  lastDistanceLogMs = now;
  Serial.print(F("t=")); Serial.print(now / 1000);
  Serial.print('.'); Serial.print((now % 1000) / 100);
  Serial.print(F("s | front=")); printDistance(FRONT_SENSOR);
  Serial.print(F(" | right=")); printDistance(RIGHT_SENSOR);
  printNavigationLog();
  Serial.println();
}

float sensorDistanceCm(uint8_t sensor) {
  if (sensor >= 2 || !samples[sensor].valid ||
      (uint32_t)(millis() - samples[sensor].sampledMs) >= STALE_MS) return -1;
  return samples[sensor].echoUs / 58.0f;
}
