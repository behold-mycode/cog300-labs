#include "Robot.h"
#include <math.h>

Mode mode = MANUAL;
const char* navigationState = "STOPPED";
static bool turning = false;
static uint32_t turnStarted = 0;

const char* modeName() { return mode == FOLLOW ? "FOLLOW" : mode == WALL ? "WALL" : "MANUAL"; }
void cancelNavigation() { mode = MANUAL; turning = false; navigationState = "STOPPED"; }
void stopRobot() { cancelNavigation(); stopMotor(); }
void startNavigation(Mode selected) {
  stopRobot(); mode = selected;
  navigationState = selected == MANUAL ? "STOPPED" : "WAITING_FOR_SENSOR";
}
static void hold(const char* reason) { stopMotor(); navigationState = reason; }
void serviceNavigation() {
  if (mode == MANUAL) return;
  float front = sensorDistanceCm(0), right = sensorDistanceCm(1);
  if (turning && (uint32_t)(millis() - turnStarted) >= TURN_LIMIT_MS) {
    stopRobot(); navigationState = "TURN_TIMEOUT"; return;
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
  if (right < 0) { hold("RIGHT_INVALID"); return; }
  if (right > WALL_MAX_CM) { hold("WALL_LOST"); return; }
  if (!turning && front <= 20) { turning = true; turnStarted = millis(); }
  if (turning && front >= 25) turning = false;
  if (turning) {
    navigationState = "TURNING_LEFT";
    requestMotor(-settings.power, settings.power);
  } else {
    float error = right - settings.wallCm;
    int correction = fabsf(error) <= DEADBAND_CM ? 0 : (int)(error * WALL_KP);
    correction = constrain(correction, -settings.power, settings.power);
    // Far from right wall -> left wheel faster -> steer right.
    navigationState = "TRACKING_WALL";
    requestMotor(settings.power - (correction < 0 ? -correction : 0),
                 settings.power - (correction > 0 ? correction : 0));
  }
}
