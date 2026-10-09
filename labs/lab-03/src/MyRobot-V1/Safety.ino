// HC-SR04 front guard. No timer libraries: preserves motor PWM on D9/D10.
const uint8_t TRIG_PIN = 4, ECHO_PIN = 2, GREEN_LED = 11, RED_LED = 12;
const uint16_t STOP_CM = 25, CLEAR_CM = 35;
const uint32_t PING_MS = 70, ECHO_TIMEOUT_US = 30000, STALE_MS = 250;
// A positive wheel can swing a chassis corner forward: block spins too.
// Reverse remains a deliberate escape; the front sensor cannot protect the rear.
bool frontBlocked = true, sensorValid = false;
uint8_t clearReadings = 0;
int distanceCm = -1;
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
  digitalWrite(RED_LED, frontBlocked ? HIGH : LOW);
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
  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(GREEN_LED, OUTPUT); pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW); digitalWrite(RED_LED, HIGH);
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
