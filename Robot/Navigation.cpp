#include "Robot.h"
#include <math.h>

Mode mode = MANUAL;
const char* navigationState = "STOPPED";
enum WallPhase { TRACK, SEARCH, BACKUP, TURN };
static WallPhase phase = TRACK;
static uint32_t phaseStarted = 0, activeMs = 0, lastService = 0;
static void enterPhase(WallPhase next) {
  phase = next; phaseStarted = millis(); activeMs = 0;
}

const char* modeName() { return mode == FOLLOW ? "FOLLOW" : mode == WALL ? "WALL" : "MANUAL"; }
void cancelNavigation() { mode = MANUAL; enterPhase(TRACK); lastService = millis(); navigationState = "STOPPED"; }
void stopRobot() { cancelNavigation(); stopMotor(); }
void startNavigation(Mode selected) {
  stopRobot(); mode = selected;
  navigationState = selected == MANUAL ? "STOPPED" : "WAITING_FOR_SENSOR";
}
static void hold(const char* reason) { stopMotor(); navigationState = reason; }
void serviceNavigation() {
  if (mode == MANUAL) return;
  float front = sensorDistanceCm(0), right = sensorDistanceCm(1);
  uint32_t now = millis(), dt = now - lastService;
  lastService = now;
  if ((phase == BACKUP && motorLeft < 0 && motorRight < 0) ||
      (phase == TURN && motorLeft < 0 && motorRight > 0)) activeMs += dt;
  if ((phase == TURN || phase == BACKUP || phase == SEARCH) &&
      (uint32_t)(now - phaseStarted) >= (phase == SEARCH ? SEARCH_LIMIT_MS : TURN_LIMIT_MS)) {
    const char* reason = phase == SEARCH ? "SEARCH_TIMEOUT" : "TURN_TIMEOUT";
    stopRobot(); navigationState = reason; return;
  }
  if (front < 0) { hold("FRONT_INVALID"); return; }
  if (mode == FOLLOW) {
    if (front > FOLLOW_MAX_CM) { hold("TARGET_LOST"); return; }
    float error = front - settings.followCm;
    if (fabsf(error) <= DEADBAND_CM) { hold("AT_DISTANCE"); return; }
    int power = (int)(fabsf(error) * FOLLOW_KP);
    power = constrain(power, settings.minimum, settings.power);
    if (error < 0) power = -power;
    navigationState = error > 0 ? "APPROACHING" : "BACKING_AWAY";
    requestMotor(power, power);
    return;
  }
  // Front clearance has priority over side-wall availability.
  if (phase != BACKUP && phase != TURN && front <= FRONT_BACKUP_CM) enterPhase(BACKUP);
  else if (phase != BACKUP && phase != TURN && front <= FRONT_TURN_CM) enterPhase(TURN);
  if (phase == BACKUP) {
    if (activeMs < BACKUP_MS) {
      navigationState = "BACKING_AWAY";
      requestMotor(-settings.power, -settings.power); return;
    }
    enterPhase(TURN);
  }
  if (phase == TURN) {
    // Time only powered rotation; allow ample clearance and side reacquisition.
    if (activeMs < TURN_MIN_MS || front < FRONT_CLEAR_CM || right < 0 || right > WALL_MAX_CM) {
      navigationState = "TURNING_LEFT";
      // P control of front-clearance error, with a motor-starting floor.
      int turnPower = (int)(TURN_KP * (FRONT_CLEAR_CM - front));
      int floor = settings.minimum > 0 ? settings.minimum : 1;
      turnPower = constrain(turnPower, floor, settings.power);
      requestMotor(-turnPower, turnPower); return;
    }
    enterPhase(TRACK);
  }
  if (right < 0) { hold("RIGHT_INVALID"); return; }
  if (right > WALL_MAX_CM) {
    if (phase != SEARCH) enterPhase(SEARCH);
    navigationState = "SEARCHING_WALL";
    requestMotor(settings.power, settings.power * 3 / 4); return;
  }
  if (phase == SEARCH) enterPhase(TRACK);
  // Pure proportional steering: error in cm, correction in PWM.
  // No derivative term or threshold jump: double error -> double correction
  // until the output limit is reached.
  float error = right - settings.wallCm;
  int correction = (int)(WALL_KP * error);
  int limit = settings.power / 2;
  correction = constrain(correction, -limit, limit);
  navigationState = "TRACKING_WALL";
  requestMotor(settings.power - (correction < 0 ? -correction : 0),
               settings.power - (correction > 0 ? correction : 0));
}
