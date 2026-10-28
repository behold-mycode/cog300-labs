#include "Robot.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

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
    lastLog = millis(); reportStatus();
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
    lastControl = millis(); Serial.println(F("ROBOT_READY 4")); return;
  }
  if (!strcmp(cmd, "HEARTBEAT") && count == 1) {
    if (controlSession) { lastControl = millis(); Serial.println(F("ALIVE")); }
    return;
  }
  if (!strcmp(cmd, "STATUS") && count == 1) { reportStatus(); return; }
  if (!strcmp(cmd, "HELP") && count == 1) {
    Serial.println(F("Protocol 4 / 115200 baud / Newline. STOP always stops."));
    Serial.println(F("HELLO opens 500ms watchdog session; HEARTBEAT keeps it alive."));
    Serial.println(F("DRIVE left right (-255..255; session required)"));
    Serial.println(F("MODE MANUAL|FOLLOW|WALL; TEST left right ms (1..5000)"));
    Serial.println(F("CONFIG power minimum invertA invertB followCm wallCm"));
    Serial.println(F("Example: CONFIG 255 255 0 1 20 20; power 1..255, minimum 0..power"));
    Serial.println(F("LOG ON|OFF; STATUS. DATA,ms,mode,state,front_cm,right_cm,left_pwm,right_pwm"));
    return;
  }
  if (!strcmp(cmd, "LOG") && count == 2) {
    if (strcmp(args[1], "ON") && strcmp(args[1], "OFF")) { invalid(); return; }
    logging = !strcmp(args[1], "ON"); lastLog = millis(); Serial.println(F("OK LOG")); return;
  }
  // No delayed command may restart an expired desktop session.
  if (controlExpired) { invalid(); return; }
  int a, b, c, d, e, f;
  if (!strcmp(cmd, "CONFIG") && count == 7) {
    if (!number(args[1], 1, 255, a) || !number(args[2], 0, a, b) ||
        !number(args[3], 0, 1, c) || !number(args[4], 0, 1, d) ||
        !number(args[5], 5, 100, e) || !number(args[6], 5, 80, f)) { invalid(); return; }
    stopRobot(); settleMotor(); settings.power = a; settings.minimum = b;
    settings.invertA = c; settings.invertB = d;
    settings.followCm = e; settings.wallCm = f;
    Serial.println(F("OK CONFIG")); return;
  }
  if (!strcmp(cmd, "MODE") && count == 2) {
    if (!strcmp(args[1], "MANUAL")) startNavigation(MANUAL);
    else if (!strcmp(args[1], "FOLLOW")) startNavigation(FOLLOW);
    else if (!strcmp(args[1], "WALL")) startNavigation(WALL);
    else { invalid(); return; }
    Serial.println(F("OK MODE")); return;
  }
  if (!strcmp(cmd, "DRIVE") && count == 3 && controlSession) {
    if (!number(args[1], -255, 255, a) || !number(args[2], -255, 255, b)) { invalid(); return; }
    if (mode != MANUAL) stopRobot();
    cancelNavigation();
    navigationState = a || b ? "DRIVING" : "STOPPED";
    if (!a && !b) stopMotor(); else requestMotor(a, b);
    lastControl = millis(); return;
  }
  if (!strcmp(cmd, "TEST") && count == 4) {
    if (!number(args[1], -255, 255, a) || !number(args[2], -255, 255, b) ||
        !number(args[3], 1, 5000, c)) { invalid(); return; }
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
