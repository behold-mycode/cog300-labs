// Motor.ino — motor outputs, movement helpers, and automatic stopping.
const uint32_t SETTLE_MS = 250; // Unpowered pause before each new movement.
bool running = false, pending = false, liveDrive = false;
int pendingLeft = 0, pendingRight = 0;
uint32_t phaseStart = 0, durationMs = 0;
uint32_t remoteTestId = 0;

// Test suite state. Numbered tests are zero-indexed internally.
const uint32_t TEST_GAP_MS = 1000;
const int DEFAULT_TEST_PWM = 255;
const uint32_t DEFAULT_TEST_MS = 3000;
bool suiteActive = false, testWaiting = false;
uint8_t suiteIndex = 0;
int activeTest = -1, suiteSpeed = DEFAULT_TEST_PWM;
uint32_t suiteDuration = DEFAULT_TEST_MS, gapStart = 0;

// Runtime calibration; P changes percentages without recompiling. Reset restores 100/100.
int leftPwmPercent = 100, rightPwmPercent = 100;

int adjustedPwm(int requested, int percent) {
  long magnitude = requested < 0 ? -(long)requested : requested;
  magnitude = magnitude * percent / 100;
  if (magnitude > 255) magnitude = 255;
  return requested < 0 ? -(int)magnitude : (int)magnitude;
}

void stopOutputs() {
  // Disable both bridges BEFORE changing direction signals. This is a coast stop.
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  running = false;
  pending = false;
  liveDrive = false;
}

void stopDrive() {
  cancelNavigation();
  stopOutputs();
  remoteTestId = 0;
  if (activeTest >= 0) logTestResult(activeTest, "CANCELLED");
  activeTest = -1;
  if (suiteActive) Serial.println(F("SUITE CANCELLED"));
  suiteActive = false;
  testWaiting = false;
}

void writeMotor(uint8_t enable, uint8_t a, uint8_t b, int speed, bool invert) {
  analogWrite(enable, 0);
  if (invert) speed = -speed;
  digitalWrite(a, speed < 0 ? HIGH : LOW);
  digitalWrite(b, speed > 0 ? HIGH : LOW);
  analogWrite(enable, speed < 0 ? -speed : speed);
}

void setDrive(int left, int right) {
  // Internal helper: callers validate -255..255 before reaching here.
  writeMotor(ENA, IN1, IN2, adjustedPwm(left, leftPwmPercent), INVERT_LEFT);
  writeMotor(ENB, IN3, IN4, adjustedPwm(right, rightPwmPercent), INVERT_RIGHT);
}

void scheduleDrive(int left, int right, uint32_t duration) {
  stopDrive();
  if (left == 0 && right == 0) {
    Serial.println(F("STOP"));
    return;
  }
  pendingLeft = left; pendingRight = right;
  durationMs = duration;
  phaseStart = millis();
  pending = true;
  Serial.print(F("Queued L=")); Serial.print(left);
  Serial.print(F(" R=")); Serial.print(right);
  Serial.print(F(" actualPWM L=")); Serial.print(adjustedPwm(left, leftPwmPercent));
  Serial.print(F(" R=")); Serial.print(adjustedPwm(right, rightPwmPercent));
  Serial.print(F(" duration(ms)=")); Serial.println(duration);
}

// Repeated keyboard frames refresh the lease rather than requeueing motion.
void updateLiveDrive(int left, int right) {
  if (left == 0 && right == 0) { stopDrive(); return; }
  if (liveDrive && (pending || running)) {
    bool reverses = (left != 0 && pendingLeft != 0 && (left < 0) != (pendingLeft < 0)) ||
                    (right != 0 && pendingRight != 0 && (right < 0) != (pendingRight < 0));
    if (!reverses) {
      pendingLeft = left; pendingRight = right;
      if (running) { setDrive(left, right); phaseStart = millis(); }
      return;
    }
  }
  scheduleDrive(left, right, 500);
  liveDrive = pending;
}

void forward(int speed, uint32_t ms) { scheduleDrive(speed, speed, ms); }
void backward(int speed, uint32_t ms) { scheduleDrive(-speed, -speed, ms); }
void turnLeft(int speed, uint32_t ms) { scheduleDrive(speed / 2, speed, ms); }
void turnRight(int speed, uint32_t ms) { scheduleDrive(speed, speed / 2, ms); }
void spinLeft(int speed, uint32_t ms) { scheduleDrive(-speed, speed, ms); }
void spinRight(int speed, uint32_t ms) { scheduleDrive(speed, -speed, ms); }

// A preset is a reproducible command, not an automatic verdict on wheel motion.
void runPreset(uint8_t index, int speed, uint32_t duration) {
  int left = 0, right = 0;
  switch (index) {
    case 0: left = speed; break;
    case 1: left = -speed; break;
    case 2: right = speed; break;
    case 3: right = -speed; break;
    case 4: left = right = speed; break;
    case 5: left = right = -speed; break;
    case 6: left = speed / 2; right = speed; break;
    case 7: left = speed; right = speed / 2; break;
    case 8: left = -speed; right = speed; break;
    case 9: left = speed; right = -speed; break;
    default: stopDrive(); return;
  }
  scheduleDrive(left, right, duration);
  if (!pending) return;
  activeTest = index;
  logTestStart(index, left, right, duration);
}

void startTestSuite(int speed, uint32_t duration) {
  stopDrive();
  suiteIndex = 0; suiteSpeed = speed; suiteDuration = duration;
  Serial.println(F("SUITE START: 10 tests. S cancels. Observe actual motion; sensor logging only; no obstacle stop."));
  runPreset(suiteIndex, suiteSpeed, suiteDuration);
  suiteActive = pending;
}

void serviceTestSuite() {
  if (!suiteActive || !testWaiting ||
      (uint32_t)(millis() - gapStart) < TEST_GAP_MS) return;
  testWaiting = false;
  ++suiteIndex;
  if (suiteIndex >= 10) {
    suiteActive = false;
    Serial.println(F("SUITE COMPLETE: commands finished; record observed PASS/FAIL manually."));
    return;
  }
  // runPreset stops/cancels any previous activity; preserve this suite afterward.
  suiteActive = false;
  runPreset(suiteIndex, suiteSpeed, suiteDuration);
  suiteActive = pending;
}

void serviceMotion() {
  uint32_t now = millis();
  if (pending && (uint32_t)(now - phaseStart) >= SETTLE_MS) {
    setDrive(pendingLeft, pendingRight);
    pending = false; running = true; phaseStart = now;
    Serial.println(F("RUN"));
    if (remoteTestId) { Serial.print(F("TEST_START ")); Serial.println(remoteTestId); }
  }
  if (running && (uint32_t)(now - phaseStart) >= durationMs) {
    stopOutputs(); Serial.println(F("DONE: stopped"));
    if (remoteTestId) {
      Serial.print(F("TEST_DONE ")); Serial.println(remoteTestId); remoteTestId = 0;
    }
    if (activeTest >= 0) logTestResult(activeTest, "COMPLETE");
    activeTest = -1;
    if (suiteActive) { testWaiting = true; gapStart = millis(); }
  }
}
