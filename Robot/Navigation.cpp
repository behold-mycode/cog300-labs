#include "Robot.h"
#include <math.h>

Mode mode = MANUAL;
const char* navigationState = "STOPPED";
enum Recovery { NONE, BACKUP, ESCAPE };
static Recovery recovery = NONE;
static uint32_t recoveryStarted = 0, poweredMs = 0, lastService = 0;
static bool searching = false, blocked = false;
static uint32_t searchStarted = 0, blockedStarted = 0;

// One rate estimator per sensor, updated only on a distinct valid measurement.
struct Observation {
  float cm = -1, rate = 0;
  uint32_t sampledMs = 0, sequence = 0;
  bool seen = false, noReturn = false;
  void update(uint8_t sensor, uint32_t now) {
    SensorReading reading = sensorReading(sensor);
    noReturn = reading.status == RANGE_NO_RETURN && (uint32_t)(now-reading.sampledMs) < SENSOR_GRACE_MS;
    if (noReturn) { cm = -1; rate = 0; }
    if (reading.cm >= 0 && (!seen || sequence != reading.sequence)) {
      uint32_t dt = reading.sampledMs - sampledMs;
      if (cm >= 0 && dt > 0 && dt < SENSOR_GRACE_MS) {
        float measured = constrain((reading.cm-cm)*1000.0f/dt, -WALL_RATE_LIMIT, WALL_RATE_LIMIT);
        rate += WALL_D_ALPHA*(measured-rate);
      } else rate = 0;
      cm = reading.cm; sampledMs = reading.sampledMs;
    }
    seen = true; sequence = reading.sequence;
    if ((uint32_t)(now-sampledMs) >= SENSOR_GRACE_MS) { cm = -1; rate = 0; }
  }
};
static Observation frontSample, rightSample;
const char* modeName() { return mode == FOLLOW ? "FOLLOW" : mode == WALL ? "WALL" : "MANUAL"; }
void cancelNavigation() {
  mode = MANUAL; recovery = NONE; searching = blocked = false;
  frontSample = Observation(); rightSample = Observation(); lastService = millis();
  navigationState = "STOPPED";
}
void stopRobot() { cancelNavigation(); stopMotor(); }
void startNavigation(Mode selected) {
  stopRobot(); mode = selected;
  navigationState = selected == MANUAL ? "STOPPED" : "WAITING_FOR_SENSOR";
}
static void hold(const char* reason) {
  stopMotor(); rightSample.rate = 0; navigationState = reason;
}
static void timeout(const char* reason) { stopRobot(); navigationState = reason; }
static void enterRecovery(Recovery next) {
  recovery = next; recoveryStarted = millis(); poweredMs = 0;
}
static void forwardSteer(int correction) {
  int floor = settings.power < WALL_INNER_PWM ? settings.power : WALL_INNER_PWM;
  correction = constrain(correction, floor-settings.power, settings.power-floor);
  requestMotor(settings.power-(correction<0 ? -correction : 0),
               settings.power-(correction>0 ? correction : 0));
}
void serviceNavigation() {
  if (mode == MANUAL) return;
  uint32_t now = millis(), dt = now-lastService; lastService = now;
  if (mode == FOLLOW) {
    float front = sensorDistanceCm(0);
    if (front < 0) { hold("FRONT_INVALID"); return; }
    if (front > FOLLOW_MAX_CM) { hold("TARGET_LOST"); return; }
    float error = front-settings.followCm;
    if (fabsf(error) <= DEADBAND_CM) { hold("AT_DISTANCE"); return; }
    int power = constrain((int)(fabsf(error)*FOLLOW_KP), settings.minimum, settings.power);
    if (error < 0) power = -power;
    navigationState = error > 0 ? "APPROACHING" : "BACKING_AWAY";
    requestMotor(power,power); return;
  }
  frontSample.update(0,now); rightSample.update(1,now);
  float front = frontSample.cm, right = rightSample.cm;
  if (recovery == BACKUP && motorLeft<0 && motorRight<0) poweredMs += dt;
  if (recovery != NONE && (uint32_t)(now-recoveryStarted) >= TURN_LIMIT_MS) {
    timeout("TURN_TIMEOUT"); return;
  }
  if (front < 0 && !frontSample.noReturn) { hold("FRONT_INVALID"); return; }
  if (recovery == NONE && front >= 0 && front <= FRONT_BACKUP_CM) enterRecovery(BACKUP);
  if (recovery == BACKUP) {
    if (poweredMs < BACKUP_MS) {
      navigationState = "BACKING_AWAY"; requestMotor(-settings.power,-settings.power); return;
    }
    enterRecovery(ESCAPE);
  }
  if (recovery == ESCAPE) {
    if (front >= 0 && front < FRONT_CLEAR_CM) {
      navigationState = "ESCAPE_TURN"; requestMotor(-settings.power,settings.power); return;
    }
    recovery = NONE; rightSample.rate = 0;
  }
  // A persistent close obstacle means the commanded curve isn't clearing it.
  if (front >= 0 && front <= FRONT_STUCK_CM) {
    if (!blocked) { blocked = true; blockedStarted = now; }
    if ((uint32_t)(now-blockedStarted) >= TURN_LIMIT_MS) { timeout("TURN_TIMEOUT"); return; }
  } else blocked = false;
  if (frontSample.noReturn && right < 0) { hold("NO_WALL_RETURN"); return; }
  if (right < 0 && !rightSample.noReturn && (front < 0 || front >= FRONT_CLEAR_CM)) { hold("RIGHT_INVALID"); return; }
  if (right > WALL_MAX_CM || rightSample.noReturn) {
    if (!searching) { searching = true; searchStarted = now; }
    if ((uint32_t)(now-searchStarted) >= SEARCH_LIMIT_MS) { timeout("SEARCH_TIMEOUT"); return; }
  } else searching = false;
  int limit = settings.power > WALL_INNER_PWM ? settings.power-WALL_INNER_PWM : 0;
  int side = searching ? settings.power/3 : right < 0 ? 0 :
             (int)(WALL_KP*(right-settings.wallCm)+WALL_KD*rightSample.rate);
  side = constrain(side,-limit,limit);
  // One continuous command: side PD plus a leftward front-obstacle term.
  // No guessed heading, timed alignment, or delayed handoff between controllers.
  int avoidance = front < 0 ? 0 : (int)(FRONT_KP*fmaxf(0,FRONT_CLEAR_CM-front));
  forwardSteer(side-avoidance);
  navigationState = avoidance > 0 ? "TURNING_LEFT" : searching ? "SEARCHING_WALL" : frontSample.noReturn ? "TRACKING_NO_FRONT_ECHO" : "TRACKING_WALL";
}
