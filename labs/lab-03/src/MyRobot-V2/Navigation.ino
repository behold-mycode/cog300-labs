// P-only autonomous control. Manual driving does not use distance stops.
#include <math.h>
const uint8_t MODE_MANUAL = 0, MODE_FOLLOW = 1, MODE_WALL = 2;
const float TARGET_CM = 20.0f, RESUME_CM = 22.0f, FRONT_CLEAR_CM = 25.0f;
const float FOLLOW_MAX_CM = 150.0f; // Do not chase a distant/lost target.
const uint32_t CONTROL_STEP_MS = 50, TURN_LIMIT_MS = 2500;
const int AUTO_MAX_PWM = 180, TURN_PWM = 90, PWM_STEP = 10;
uint8_t navigationMode = MODE_MANUAL;
float followKp = 6.0f, wallKp = 3.0f;
int wallBasePwm = 100, autoLeft = 0, autoRight = 0;
uint32_t lastNavigationMs = 0, turnStartedMs = 0, reversalStartedMs = 0;
bool followHolding = true, cornerTurning = false, reversalWaiting = false;
const char* navigationState = "MANUAL";

void cancelNavigation() {
  navigationMode = MODE_MANUAL;
  autoLeft = autoRight = 0;
  cornerTurning = reversalWaiting = false;
  followHolding = true;
  navigationState = "MANUAL";
}

void navigationHold(const char* reason) {
  stopOutputs(); autoLeft = autoRight = 0;
  reversalWaiting = false;
  navigationState = reason;
}

void reportNavigation() {
  Serial.print(F("MODE "));
  Serial.print(navigationMode == MODE_FOLLOW ? F("FOLLOW") : navigationMode == MODE_WALL ? F("WALL") : F("MANUAL"));
  Serial.print(F(" state=")); Serial.print(navigationState);
  Serial.print(F(" target_cm=20 kp_follow=")); Serial.print(followKp);
  Serial.print(F(" kp_wall=")); Serial.print(wallKp);
  Serial.print(F(" base_pwm=")); Serial.println(wallBasePwm);
}

void startNavigation(uint8_t mode) {
  stopDrive();
  navigationMode = mode;
  lastNavigationMs = millis() - CONTROL_STEP_MS;
  navigationState = mode == MODE_MANUAL ? "MANUAL" : "WAITING_FOR_READING";
  reportNavigation();
}

int limitedPwm(float value) {
  if (value > AUTO_MAX_PWM) return AUTO_MAX_PWM;
  if (value < -AUTO_MAX_PWM) return -AUTO_MAX_PWM;
  return (int)value;
}
int approachPwm(int current, int target) {
  if (target > current + PWM_STEP) return current + PWM_STEP;
  if (target < current - PWM_STEP) return current - PWM_STEP;
  return target;
}

void driveNavigation(int left, int right) {
  uint32_t now = millis();
  bool reverses = (autoLeft > 0 && left < 0) || (autoLeft < 0 && left > 0) ||
                  (autoRight > 0 && right < 0) || (autoRight < 0 && right > 0);
  if (reverses) {
    stopOutputs(); autoLeft = autoRight = 0;
    reversalWaiting = true; reversalStartedMs = now;
  }
  if (reversalWaiting) {
    if ((uint32_t)(now - reversalStartedMs) < SETTLE_MS) return;
    reversalWaiting = false;
  }
  autoLeft = approachPwm(autoLeft, left); autoRight = approachPwm(autoRight, right);
  setDrive(autoLeft, autoRight);
  // Existing motion watchdog also bounds output if navigation updates stop.
  pendingLeft = autoLeft; pendingRight = autoRight;
  running = autoLeft != 0 || autoRight != 0; pending = false; liveDrive = false;
  durationMs = 500; phaseStart = now;
}

void serviceNavigation() {
  if (navigationMode == MODE_MANUAL) return;
  float front = sensorDistanceCm(0), right = sensorDistanceCm(1);
  // Measurement failures and near-target stops take effect without waiting for the ramp tick.
  if (cornerTurning && (uint32_t)(millis() - turnStartedMs) >= TURN_LIMIT_MS) {
    stopDrive(); navigationState = "TURN_TIMEOUT"; reportNavigation(); return;
  }
  if (front < 0) { followHolding = true; navigationHold("FRONT_INVALID"); return; }
  if (navigationMode == MODE_FOLLOW) {
    if (front > FOLLOW_MAX_CM) { followHolding = true; navigationHold("TARGET_OUT_OF_RANGE"); return; }
    if (front <= TARGET_CM || (followHolding && front <= RESUME_CM)) {
      followHolding = true; navigationHold("AT_DISTANCE"); return;
    }
    followHolding = false;
  } else {
    if (right < 0) { navigationHold("RIGHT_INVALID"); return; }
    if (!cornerTurning && front <= TARGET_CM) {
      cornerTurning = true; turnStartedMs = millis();
      navigationHold("TURN_LEFT");
      reversalWaiting = true; reversalStartedMs = millis();
    } else if (cornerTurning && front >= FRONT_CLEAR_CM) {
      cornerTurning = false;
    }
  }
  uint32_t now = millis();
  if ((uint32_t)(now - lastNavigationMs) < CONTROL_STEP_MS) return;
  lastNavigationMs = now;
  if (navigationMode == MODE_FOLLOW) {
    int pwm = limitedPwm(followKp * (front - TARGET_CM));
    navigationState = "FOLLOWING";
    driveNavigation(pwm, pwm); // Always nonnegative: no reverse in object mode.
  } else if (cornerTurning) {
    navigationState = "TURN_LEFT";
    driveNavigation(-TURN_PWM, TURN_PWM);
  } else {
    float correction = wallKp * (right - TARGET_CM);
    // Limit correction to prevent reversing a wheel while tracking a wall.
    if (correction > wallBasePwm) correction = wallBasePwm;
    if (correction < -wallBasePwm) correction = -wallBasePwm;
    navigationState = "TRACKING_WALL";
    // Too far from right wall => faster left wheel => steer right.
    driveNavigation(limitedPwm(wallBasePwm + correction), limitedPwm(wallBasePwm - correction));
  }
}

bool parseGain(const char* text, float& value) {
  if (!text || !*text) return false;
  char* end;
  double parsed = strtod(text, &end);
  if (*end || !isfinite(parsed) || parsed < 0 || parsed > 20) return false;
  value = (float)parsed; return true;
}

bool navigationCommand(char** tokens, uint8_t count) {
  if (strcmp(tokens[0], "MODE") == 0) {
    if (count != 2) { invalidCommand(); return true; }
    for (char* c = tokens[1]; *c; ++c) *c = toupper((unsigned char)*c);
    if (strcmp(tokens[1], "MANUAL") == 0) startNavigation(MODE_MANUAL);
    else if (strcmp(tokens[1], "FOLLOW") == 0) startNavigation(MODE_FOLLOW);
    else if (strcmp(tokens[1], "WALL") == 0) startNavigation(MODE_WALL);
    else invalidCommand();
    return true;
  }
  if (strcmp(tokens[0], "KP") == 0) {
    float gain;
    if (count != 3 || !parseGain(tokens[2], gain)) { invalidCommand(); return true; }
    for (char* c = tokens[1]; *c; ++c) *c = toupper((unsigned char)*c);
    if (strcmp(tokens[1], "FOLLOW") == 0) followKp = gain;
    else if (strcmp(tokens[1], "WALL") == 0) wallKp = gain;
    else { invalidCommand(); return true; }
    stopDrive(); reportNavigation(); return true;
  }
  if (strcmp(tokens[0], "BASE") == 0) {
    long value;
    if (count != 2 || !parseNumber(tokens[1], 0, AUTO_MAX_PWM, value)) { invalidCommand(); return true; }
    stopDrive(); wallBasePwm = value; reportNavigation(); return true;
  }
  return false;
}

void printNavigationLog() {
  if (navigationMode == MODE_MANUAL) return;
  Serial.print(F(" | ")); Serial.print(navigationState);
  Serial.print(F(" L=")); Serial.print(autoLeft);
  Serial.print(F(" R=")); Serial.print(autoRight);
}
