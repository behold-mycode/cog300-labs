#include "Robot.h"
#include <math.h>

Mode mode = MANUAL;
const char* navigationState = "STOPPED";
enum WallPhase { TRACK, SEARCH, CORNER, ALIGN, BACKUP, ESCAPE };
static WallPhase phase = TRACK;
static uint32_t phaseStarted = 0, activeMs = 0, lastService = 0;
static uint8_t clearSamples = 0;

struct Observation {
  float cm = -1;
  uint32_t sampledMs = 0, sequence = 0;
  bool seen = false, newValid = false, currentValid = false;
  void update(uint8_t sensor, uint32_t now) {
    SensorReading reading = sensorReading(sensor);
    newValid = reading.cm >= 0 && (!seen || reading.sequence != sequence);
    currentValid = reading.cm >= 0;
    seen = true; sequence = reading.sequence;
    if (newValid) { cm = reading.cm; sampledMs = reading.sampledMs; }
    // A missed echo may bridge one sensor cycle, never renew the last-good age.
    if (cm >= 0 && (uint32_t)(now - sampledMs) >= SENSOR_GRACE_MS) cm = -1;
  }
};
static Observation frontSample, rightSample;
static bool pdReady = false;
static float previousRight = 0, filteredRate = 0;
static uint32_t previousRightMs = 0;
static int wallCorrection = 0;

static void resetPD() { pdReady = false; filteredRate = 0; wallCorrection = 0; }
static void enterPhase(WallPhase next) {
  phase = next; phaseStarted = millis(); activeMs = 0; clearSamples = 0;
  resetPD(); // A corner/invalid interval is not a wall-distance derivative.
}
const char* modeName() { return mode == FOLLOW ? "FOLLOW" : mode == WALL ? "WALL" : "MANUAL"; }
void cancelNavigation() {
  mode = MANUAL; enterPhase(TRACK); lastService = millis();
  frontSample = Observation(); rightSample = Observation(); navigationState = "STOPPED";
}
void stopRobot() { cancelNavigation(); stopMotor(); }
void startNavigation(Mode selected) {
  stopRobot(); mode = selected;
  navigationState = selected == MANUAL ? "STOPPED" : "WAITING_FOR_SENSOR";
}
static void hold(const char* reason) { stopMotor(); resetPD(); navigationState = reason; }
static void timedOut(const char* reason) { stopRobot(); navigationState = reason; }

static int trackCorrection() {
  if (!pdReady || rightSample.newValid) {
    if (pdReady) {
      uint32_t dt = rightSample.sampledMs - previousRightMs;
      if (dt > 0 && dt < SENSOR_GRACE_MS) {
        float rate = (rightSample.cm - previousRight) * 1000.0f / dt;
        rate = constrain(rate, -WALL_RATE_LIMIT, WALL_RATE_LIMIT);
        filteredRate += WALL_D_ALPHA * (rate - filteredRate);
      } else filteredRate = 0;
    }
    previousRight = rightSample.cm; previousRightMs = rightSample.sampledMs; pdReady = true;
    // P corrects position; D brakes approach to the wall and reacts to drift away.
    float error = rightSample.cm - settings.wallCm;
    wallCorrection = (int)(WALL_KP * error + WALL_KD * filteredRate);
  }
  return wallCorrection;
}
static void forwardSteer(int correction, int limit) {
  correction = constrain(correction, -limit, limit);
  // No wheel reversal: keep the outside wheel at full configured power.
  requestMotor(settings.power - (correction < 0 ? -correction : 0),
               settings.power - (correction > 0 ? correction : 0));
}

void serviceNavigation() {
  if (mode == MANUAL) return;
  uint32_t now = millis(), dt = now - lastService; lastService = now;
  if (mode == FOLLOW) {
    float front = sensorDistanceCm(0); // Object-following behavior is unchanged.
    if (front < 0) { hold("FRONT_INVALID"); return; }
    if (front > FOLLOW_MAX_CM) { hold("TARGET_LOST"); return; }
    float error = front - settings.followCm;
    if (fabsf(error) <= DEADBAND_CM) { hold("AT_DISTANCE"); return; }
    int power = constrain((int)(fabsf(error) * FOLLOW_KP), settings.minimum, settings.power);
    if (error < 0) power = -power;
    navigationState = error > 0 ? "APPROACHING" : "BACKING_AWAY";
    requestMotor(power, power); return;
  }
  frontSample.update(0, now); rightSample.update(1, now);
  float front = frontSample.cm, right = rightSample.cm;
  bool powered = (phase == BACKUP && motorLeft < 0 && motorRight < 0) ||
                 (phase == ESCAPE && motorLeft < 0 && motorRight > 0) ||
                 (phase == CORNER && motorLeft > 0 && motorRight > motorLeft) ||
                 (phase == ALIGN && motorLeft > 0 && motorRight > 0);
  if (powered) activeMs += dt;
  uint32_t elapsed = now - phaseStarted;
  if ((phase == CORNER || phase == BACKUP || phase == ESCAPE) && elapsed >= TURN_LIMIT_MS) {
    timedOut("TURN_TIMEOUT"); return;
  }
  if (phase == SEARCH && elapsed >= SEARCH_LIMIT_MS) { timedOut("SEARCH_TIMEOUT"); return; }
  if (phase == ALIGN && elapsed >= ALIGN_LIMIT_MS) { timedOut("ALIGN_TIMEOUT"); return; }
  if (front < 0) { clearSamples = 0; hold("FRONT_INVALID"); return; }

  // Emergency reverse is separate from ordinary forward-arc corners.
  if (phase != BACKUP && phase != ESCAPE && front <= FRONT_BACKUP_CM) enterPhase(BACKUP);
  if (phase == BACKUP) {
    if (activeMs < BACKUP_MS) {
      navigationState = "BACKING_AWAY";
      requestMotor(-settings.power, -settings.power); return;
    }
    enterPhase(ESCAPE);
  }
  if (phase != ESCAPE && phase != CORNER && front <= FRONT_TURN_CM) enterPhase(CORNER);
  if (phase == CORNER || phase == ESCAPE) {
    // Only distinct valid front echoes can confirm clearance. A missing side
    // echo must not force the robot to keep rotating past an already-clear corner.
    if (!frontSample.currentValid) clearSamples = 0;
    else if (frontSample.newValid) {
      // Near a planar wall, front/right clearance ratio grows as heading
      // becomes parallel. Absolute front clearance alone can clear at ~45 deg.
      float requiredClearance = CORNER_NO_SIDE_CLEAR_CM;
      if (rightSample.currentValid && right > 0 && right < 50)
        requiredClearance = fmaxf(FRONT_CLEAR_CM, CORNER_CLEAR_RATIO * right);
      if (activeMs >= TURN_MIN_MS && front >= requiredClearance) ++clearSamples;
      else clearSamples = 0;
    }
    if (clearSamples >= 2) enterPhase(ALIGN);
    else {
      if (phase == ESCAPE) {
        navigationState = "ESCAPE_TURN";
        requestMotor(-settings.power, settings.power);
      } else {
        navigationState = frontSample.currentValid ? "TURNING_LEFT" : "TURNING_LEFT_HELD";
        int correction = (int)(TURN_KP * (FRONT_CLEAR_CM - front));
        // Both wheels roll forward. Inner wheel stays at 35–55% of outer
        // output; turn geometry must be calibrated against actual wheel speed.
        correction = constrain(correction, settings.power * 45 / 100, settings.power * 65 / 100);
        forwardSteer(-correction, settings.power * 65 / 100);
      }
      return;
    }
  }
  if (phase == ALIGN) {
    if (right < 0) {
      resetPD();
      if ((uint32_t)(millis() - phaseStarted) < SENSOR_GRACE_MS) {
        navigationState = "REACQUIRING_WALL"; requestMotor(settings.power, settings.power);
      } else hold("RIGHT_INVALID");
      return;
    }
    if (activeMs < ALIGN_MS) {
      navigationState = "ALIGNING_WALL";
      // Do not undo a completed left corner by immediately chasing a side
      // distance that is still oblique. Continue forward; allow left correction
      // if already too close, then hand back to full PD after this short phase.
      int correction = trackCorrection();
      forwardSteer(correction > 0 ? 0 : correction, settings.power / 3); return;
    }
    enterPhase(TRACK);
  }
  if (right < 0) { hold("RIGHT_INVALID"); return; }
  if (right > WALL_MAX_CM) {
    if (phase != SEARCH) enterPhase(SEARCH);
    navigationState = "SEARCHING_WALL";
    forwardSteer(settings.power / 3, settings.power / 3); return;
  }
  if (phase == SEARCH) enterPhase(TRACK);
  navigationState = frontSample.currentValid && rightSample.currentValid ? "TRACKING_WALL" : "TRACKING_WALL_HELD";
  forwardSteer(trackCorrection(), settings.power * 3 / 4);
}
