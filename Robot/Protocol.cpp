#include "Robot.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

bool controlSession = false, controlExpired = false;
static uint32_t lastControl = 0, lastLog = 0, lastInput = 0;
static bool logging = true, discardLine = false;
static char input[96];
static uint8_t inputLength = 0;

void serviceControlTimeout() {
  if (controlSession && (uint32_t)(millis() - lastControl) >= CONTROL_TIMEOUT_MS) {
    stopRobot(); controlSession = false; controlExpired = true;
    Serial.println(F("CONTROL_TIMEOUT"));
  }
}
void reportStatus() {
  // Stable CSV fields; -1 explicitly means invalid/stale, never clear space.
  Serial.print(F("DATA,")); Serial.print(millis());
  Serial.print(','); Serial.print(modeName());
  Serial.print(','); Serial.print(navigationState);
  Serial.print(','); Serial.print(sensorDistanceCm(0));
  Serial.print(','); Serial.print(sensorDistanceCm(1));
  Serial.print(','); Serial.print(motorLeft);
  Serial.print(','); Serial.println(motorRight);
}
void serviceTelemetry() {
  if (logging && (uint32_t)(millis() - lastLog) >= LOG_MS) {
    lastLog = millis(); reportStatus(); reportLine(); reportNavigation(); reportEncoders();
  }
}
static void invalid() { stopRobot(); Serial.println(F("ERROR invalid command; stopped")); }
static bool number(const char* text, int low, int high, int& value) {
  if (!*text || strlen(text) > 6) return false;
  char* end;
  long parsed = strtol(text, &end, 10);
  if (*end || parsed < low || parsed > high) return false;
  value = (int)parsed; return true;
}
static bool realNumber(const char* text,float low,float high,float& value) {
  char* end;value=strtof(text,&end);
  return *text && !*end && isfinite(value) && value>=low && value<=high;
}
void processCommand(char* line) {
  char* args[8]; uint8_t count = 0;
  for (char* p = strtok(line, " \t"); p; p = strtok(NULL, " \t")) {
    if (count == 8) { invalid(); return; }
    args[count++] = p;
  }
  if (!count) return;
  for (uint8_t i = 0; i < count; ++i)
    for (char* p = args[i]; *p; ++p) *p = toupper((unsigned char)*p);
  const char* cmd = args[0];
  if (!strcmp(cmd, "STOP") && count == 1) {
    stopRobot(); Serial.println(F("OK STOP")); return;
  }
  if (!strcmp(cmd, "HELLO") && count == 1) {
    stopRobot(); controlSession = true; controlExpired = false;
    lastControl = millis();
    // Reconnecting a different desktop must not overwrite the calibration of a
    // live map with its startup defaults.
    if(explorer.graph.nodeCount) {
      Serial.print(F("CONFIG_REPORT,"));Serial.print(settings.power);Serial.print(',');
      Serial.print(settings.minimum);Serial.print(',');Serial.print(settings.invertA?1:0);Serial.print(',');
      Serial.print(settings.invertB?1:0);Serial.print(',');Serial.print(settings.followCm);Serial.print(',');
      Serial.print(settings.wallCm);Serial.print(',');Serial.println(lineTapeLevel);
    }
    Serial.println(F("ROBOT_READY 6")); return;
  }
  if (!strcmp(cmd, "HEARTBEAT") && count == 1) {
    if (controlSession) { lastControl = millis(); Serial.println(F("ALIVE")); }
    return;
  }
  if (!strcmp(cmd, "STATUS") && count == 1) { reportStatus(); return; }
  if (!strcmp(cmd, "HELP") && count == 1) {
    Serial.println(F("Protocol 6 / 115200 baud / Newline. STOP always stops."));
    Serial.println(F("HELLO opens 500ms watchdog session; HEARTBEAT keeps it alive."));
    Serial.println(F("DRIVE left right (-255..255; session required)"));
    Serial.println(F("MODE MANUAL|FOLLOW|WALL|LINE|MAZE|STOP_ON_TAPE|STOP_AT_END; TEST left right ms (1..5000)"));
    Serial.println(F("CONFIG power minimum invertA invertB followCm wallCm [tapeLevel: -1 unknown, 0 LOW, 1 HIGH]"));
    Serial.println(F("Example: CONFIG 255 255 1 0 20 10; power 1..255, minimum 0..power"));
    Serial.println(F("ENC kind invertL invertR (0 off,1 single,2 quadrature); ODOM leftMmPerCount rightMmPerCount trackMm"));
    Serial.println(F("GEOMETRY sensorOffsetMm tapeWidthMm nodeMatchRadiusMm turnClearanceMm"));
    Serial.println(F("DYNAMICS decelerationMmS2 reactionSeconds marginMm; MODEL hit falsePositive transitionMix"));
    Serial.println(F("MAP DUMP|RESET|CONFIRM node; geometry changes require empty map"));
    Serial.println(F("TRACE 0|1; LOG ON|OFF; STATUS. DATA,ms,mode,state,front_cm,right_cm,left_pwm,right_pwm"));
    return;
  }
  if (!strcmp(cmd, "LOG") && count == 2) {
    if (strcmp(args[1], "ON") && strcmp(args[1], "OFF")) { invalid(); return; }
    logging = !strcmp(args[1], "ON"); lastLog = millis(); Serial.println(F("OK LOG")); return;
  }
  // No delayed command may restart an expired desktop session.
  if (controlExpired) { invalid(); return; }
  // Keep streaming records a consistent snapshot; STOP and heartbeat above
  // remain available. Reject mutation until MAP_END, without clearing the map.
  if(mapExportActive()) { stopRobot();Serial.println(F("BUSY MAP_EXPORT"));return; }
  int a, b, c, d, e, f, g = -1;
  if (!strcmp(cmd,"ENC") && count==4) {
    if(explorer.graph.nodeCount || !number(args[1],0,2,a)||!number(args[2],0,1,b)||!number(args[3],0,1,c)) { invalid();return; }
    configureEncoders(a,b,c);Serial.println(F("OK ENC"));return;
  }
  if (!strcmp(cmd,"ODOM") && count==4) {
    float l,r,w;
    if(explorer.graph.nodeCount || !realNumber(args[1],.001f,100,l)||!realNumber(args[2],.001f,100,r)||!realNumber(args[3],10,1000,w)) { invalid();return; }
    stopRobot();odometry.configure(l,r,w);explorer.geometry=maze::Geometry();explorer.dynamics=maze::Dynamics();explorer.world.reset();Serial.println(F("OK ODOM"));return;
  }
  if(!strcmp(cmd,"TRACE")&&count==2&&number(args[1],0,1,a)) {
    setObservationTrace(a);Serial.println(F("OK TRACE"));return;
  }
  if(!strcmp(cmd,"DYNAMICS")&&count==4) {
    maze::Dynamics cfg;
    if(explorer.graph.nodeCount||!realNumber(args[1],1,10000,cfg.deceleration)||
       !realNumber(args[2],.001f,5,cfg.reactionSeconds)||!realNumber(args[3],1,500,cfg.margin)) { invalid();return; }
    stopRobot();explorer.dynamics=cfg;Serial.println(F("OK DYNAMICS"));return;
  }
  if(!strcmp(cmd,"MODEL")&&count==4) {
    maze::SensorModel cfg;
    if(explorer.graph.nodeCount||!realNumber(args[1],.501f,.999f,cfg.hit)||
       !realNumber(args[2],.001f,.499f,cfg.falsePositive)||!realNumber(args[3],.001f,.999f,cfg.change)) { invalid();return; }
    stopRobot();explorer.world.reset();explorer.world.model=cfg;Serial.println(F("OK MODEL"));return;
  }
  if (!strcmp(cmd,"GEOMETRY") && count==5) {
    maze::Geometry geom;
    if(explorer.graph.nodeCount || !odometry.configured ||
       !realNumber(args[1],1,1000,geom.sensorOffset)||!realNumber(args[2],1,500,geom.tapeWidth)||
       !realNumber(args[3],1,2000,geom.matchRadius)||!realNumber(args[4],1,1000,geom.turnClearance)) { invalid();return; }
    geom.distanceTolerance=fmaxf(odometry.leftScale,odometry.rightScale);
    geom.angleTolerance=2*geom.distanceTolerance/odometry.track;
    if(!geom.valid()) { invalid();return; }
    stopRobot();explorer.geometry=geom;Serial.println(F("OK GEOMETRY"));return;
  }
  if (!strcmp(cmd,"MAP") && count>=2) {
    if(!strcmp(args[1],"DUMP")&&count==2) { beginMapExport();return; }
    if(!strcmp(args[1],"RESET")&&count==2) {
      stopRobot();explorer.clear();resetOdometry();Serial.println(F("OK MAP RESET"));return;
    }
    if(!strcmp(args[1],"CONFIRM")&&count==3&&number(args[2],0,63,a)) {
      stopRobot();
      if(!explorer.confirm(a)) { invalid();return; }
      Serial.println(F("OK MAP CONFIRM"));return;
    }
    invalid();return;
  }
  if (!strcmp(cmd, "CONFIG") && (count == 7 || count == 8)) {
    if (!number(args[1], 1, 255, a) || !number(args[2], 0, a, b) ||
        !number(args[3], 0, 1, c) || !number(args[4], 0, 1, d) ||
        !number(args[5], 5, 100, e) || !number(args[6], 5, 80, f) ||
        (count == 8 && !number(args[7], -1, 1, g))) { invalid(); return; }
    if(explorer.graph.nodeCount && (settings.invertA!=c || settings.invertB!=d || lineTapeLevel!=g)) {
      invalid();return;
    }
    stopRobot(); settleMotor(); settings.power = a; settings.minimum = b;
    settings.invertA = c; settings.invertB = d;
    settings.followCm = e; settings.wallCm = f; lineTapeLevel = g;
    Serial.println(F("OK CONFIG")); return;
  }
  if (!strcmp(cmd, "MODE") && count == 2) {
    if (!strcmp(args[1], "MANUAL")) startNavigation(MANUAL);
    else if (!strcmp(args[1], "FOLLOW")) startNavigation(FOLLOW);
    else if (!strcmp(args[1], "WALL")) startNavigation(WALL);
    else if (!strcmp(args[1], "LINE")) startNavigation(LINE);
    else if (!strcmp(args[1], "MAZE")) startNavigation(MAZE);
    else if (!strcmp(args[1], "STOP_ON_TAPE")) startNavigation(STOP_ON_TAPE);
    else if (!strcmp(args[1], "STOP_AT_END")) startNavigation(STOP_AT_END);
    else { invalid(); return; }
    Serial.println(F("OK MODE")); return;
  }
  if (!strcmp(cmd, "DRIVE") && count == 3 && controlSession) {
    if (!number(args[1], -255, 255, a) || !number(args[2], -255, 255, b)) { invalid(); return; }
    if (mode != MANUAL) stopRobot();
    if(a || b) explorer.invalidate();
    cancelNavigation();
    navigationState = a || b ? "DRIVING" : "STOPPED";
    if (!a && !b) stopMotor(); else requestMotor(a, b);
    lastControl = millis(); return;
  }
  if (!strcmp(cmd, "TEST") && count == 4) {
    if (!number(args[1], -255, 255, a) || !number(args[2], -255, 255, b) ||
        !number(args[3], 1, 5000, c)) { invalid(); return; }
    if(a || b) explorer.invalidate();
    startTest(a, b, c); navigationState = "TIMED_TEST"; return;
  }
  invalid();
}
void readCommands() {
  // Bounded work prevents continuous input from starving sensors/watchdog.
  for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
    char c = (char)Serial.read(); lastInput = millis();
    if (c == '!') { stopRobot(); inputLength = 0; discardLine = true; }
    else if (c == '\r' || c == '\n') {
      if (!discardLine && inputLength) { input[inputLength] = 0; processCommand(input); }
      inputLength = 0; discardLine = false;
    } else if (!discardLine) {
      if (inputLength >= sizeof(input) - 1 || (c != '\t' && (c < 32 || c > 126))) {
        invalid(); inputLength = 0; discardLine = true;
      } else input[inputLength++] = c;
    }
  }
  if (inputLength && (uint32_t)(millis() - lastInput) >= 1000) {
    stopRobot(); inputLength = 0; discardLine = true;
  }
}
