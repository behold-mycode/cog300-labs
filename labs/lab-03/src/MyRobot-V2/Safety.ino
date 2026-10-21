// HC-SR04 measurement only: sensor readings never control motor motion.
const uint32_t PING_MS = 70, ECHO_TIMEOUT_US = 30000, STALE_MS = 250;
bool sensorValid = false;
int distanceCm = -1;
uint32_t lastEchoUs = 0; // Raw pulse retained for decimal distance logging only.
uint32_t lastSampleMs = 0, lastPingMs = 0, pingStartUs = 0;
bool pingPending = false;
volatile bool echoArmed = false, echoRisen = false, echoReady = false;
volatile uint32_t echoStartUs = 0, echoWidthUs = 0;

void echoChanged() {
  if (!echoArmed) return;
  uint32_t now = micros();
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoStartUs = now; echoRisen = true;
  } else if (echoRisen) {
    echoWidthUs = (uint32_t)(now - echoStartUs);
    echoReady = true; echoArmed = false;
  }
}

void reportSensor() {
  Serial.print(F("SENSOR "));
  Serial.print(sensorValid ? F("VALID") : F("INVALID"));
  Serial.print(' '); Serial.println(distanceCm);
}

void acceptDistance(int cm) {
  bool wasValid = sensorValid;
  lastSampleMs = millis();
  sensorValid = cm >= 2 && cm <= 400;
  distanceCm = sensorValid ? cm : -1;
  digitalWrite(GREEN_LED, sensorValid ? HIGH : LOW);
  if (wasValid != sensorValid) reportSensor();
}

void setupSensor() {
  // Keep the reserved right sensor quiet until its acquisition is implemented.
  digitalWrite(RIGHT_TRIG_PIN, LOW); pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoChanged, CHANGE);
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
    lastEchoUs = width;
    // Reject out-of-range pulses before rounding to whole centimetres.
    acceptDistance(width >= 116 && width <= 23200 ? (int)(width / 58) : -1);
  } else if (pingPending && (uint32_t)(micros() - pingStartUs) >= ECHO_TIMEOUT_US) {
    noInterrupts(); echoArmed = false; echoReady = false; interrupts();
    pingPending = false; acceptDistance(-1);
  }
  if (sensorValid && (uint32_t)(now - lastSampleMs) >= STALE_MS) acceptDistance(-1);
  if (!pingPending && (uint32_t)(now - lastPingMs) >= PING_MS) {
    lastPingMs = now;
    // A stuck HIGH echo is a fault; never interpret it as open space.
    if (digitalRead(ECHO_PIN) == HIGH) { acceptDistance(-1); return; }
    noInterrupts(); echoRisen = false; echoReady = false; echoArmed = true; interrupts();
    pingStartUs = micros(); pingPending = true;
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN, LOW);
  }
}

// Optional human-readable stream. Reuses acquisition; never triggers another ping.
// Two lines per second leave room for commands at the existing 9600 baud.
const uint32_t DISTANCE_LOG_MS = 500;
bool distanceLogEnabled = false;
uint32_t lastDistanceLogMs = 0;

void setDistanceLog(bool enabled) {
  distanceLogEnabled = enabled;
  lastDistanceLogMs = millis();
  Serial.println(enabled ? F("Distance log ON (cm, every 500 ms)") : F("Distance log OFF"));
}

void serviceDistanceLog() {
  uint32_t now = millis();
  if (!distanceLogEnabled || (uint32_t)(now - lastDistanceLogMs) < DISTANCE_LOG_MS) return;
  lastDistanceLogMs = now;
  Serial.print(F("t=")); Serial.print(now / 1000);
  Serial.print('.'); Serial.print((now % 1000) / 100);
  Serial.print(F("s | distance="));
  if (sensorValid && (uint32_t)(now - lastSampleMs) < STALE_MS) {
    // Round raw echo to tenths of a cm for measurement logging.
    uint32_t tenthsCm = (lastEchoUs * 10UL + 29UL) / 58UL;
    Serial.print(tenthsCm / 10); Serial.print('.'); Serial.print(tenthsCm % 10);
    Serial.println(F(" cm | OK"));
  } else {
    Serial.println(F("-- cm | NO VALID READING"));
  }
}
