// HC-SR04 front guard. No timer libraries: preserves motor PWM on D5/D10.
const uint16_t STOP_CM = 10, CLEAR_CM = 20;
const uint32_t PING_MS = 70, ECHO_TIMEOUT_US = 30000, STALE_MS = 250;
// A positive wheel can swing a chassis corner forward: block spins too.
// Reverse remains a deliberate escape; the front sensor cannot protect the rear.
bool frontBlocked = true, sensorValid = false;
uint8_t clearReadings = 0;
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

void reportSafety() {
  Serial.print(F("SAFETY "));
  Serial.print(!sensorValid ? F("UNKNOWN") : frontBlocked ? F("OBSTACLE") : F("CLEAR"));
  Serial.print(' '); Serial.println(distanceCm);
}

void applySafetyState(bool wasBlocked, bool wasValid) {
  digitalWrite(GREEN_LED, frontBlocked ? LOW : HIGH);
  // On detection stop everything, including reverse, pending steps and suite gaps.
  // A subsequent deliberate reverse request can be accepted by allowMotion().
  if (frontBlocked && (!wasBlocked || (wasValid && !sensorValid))) {
    stopDrive();
  }
  if (wasBlocked != frontBlocked || wasValid != sensorValid) reportSafety();
}

void acceptDistance(int cm) {
  bool wasBlocked = frontBlocked, wasValid = sensorValid;
  lastSampleMs = millis();
  sensorValid = cm >= 2 && cm <= 400;
  distanceCm = sensorValid ? cm : -1;
  if (!sensorValid || cm <= STOP_CM) {
    frontBlocked = true; clearReadings = 0;
  } else if (cm > CLEAR_CM) {
    if (clearReadings < 3) ++clearReadings;
    if (clearReadings >= 3) frontBlocked = false;
  } else {
    clearReadings = 0; // Hysteresis: retain blocked/clear in the middle band.
  }
  applySafetyState(wasBlocked, wasValid);
}

bool allowMotion(int left, int right) {
  if (frontBlocked && (left > 0 || right > 0)) {
    stopDrive();
    Serial.println(F("MOTION_BLOCKED"));
    return false;
  }
  return true;
}

void setupSafety() {
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

void serviceSafety() {
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
    // Round to tenths of a cm; the safety guard keeps its existing whole-cm logic.
    uint32_t tenthsCm = (lastEchoUs * 10UL + 29UL) / 58UL;
    Serial.print(tenthsCm / 10); Serial.print('.'); Serial.print(tenthsCm % 10);
    Serial.println(F(" cm | OK"));
  } else {
    Serial.println(F("-- cm | NO VALID READING"));
  }
}
