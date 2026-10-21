// Serial.ino — command parsing, help text, and serial input handling.
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

const uint32_t INPUT_TIMEOUT_MS = 1000;
uint32_t lastInputMs = 0;
char lineBuffer[64];
uint8_t lineLength = 0;
bool discardLine = false;

// Protocol 3: GUI opens a heartbeat-protected session. Legacy Serial Monitor works before HELLO.
bool controlSession = false, controlExpired = false;
uint32_t lastControlMs = 0;
const uint32_t CONTROL_TIMEOUT_MS = 500;

void serviceControlTimeout() {
  if (controlSession && (uint32_t)(millis() - lastControlMs) >= CONTROL_TIMEOUT_MS) {
    stopDrive(); controlSession = false; controlExpired = true;
    Serial.println(F("CONTROL_TIMEOUT: stopped; reconnect controller"));
  }
}

void reportStatus() {
  Serial.print(F("STATE trim=")); Serial.print(leftPwmPercent); Serial.print('/'); Serial.print(rightPwmPercent);
  Serial.print(F(" invert=")); Serial.print(INVERT_LEFT); Serial.print('/'); Serial.print(INVERT_RIGHT);
  Serial.print(F(" requested=")); Serial.print(running ? pendingLeft : 0); Serial.print('/'); Serial.print(running ? pendingRight : 0);
  Serial.print(F(" actual=")); Serial.print(running ? adjustedPwm(pendingLeft, leftPwmPercent) : 0);
  Serial.print('/'); Serial.println(running ? adjustedPwm(pendingRight, rightPwmPercent) : 0);
}

const char* const TEST_NAMES[] = {
  "LEFT_FORWARD", "LEFT_REVERSE", "RIGHT_FORWARD", "RIGHT_REVERSE",
  "FORWARD", "BACKWARD", "CURVE_LEFT", "CURVE_RIGHT", "SPIN_LEFT", "SPIN_RIGHT"
};

void logTestStart(uint8_t index, int left, int right, uint32_t duration) {
  Serial.print(F("TEST,")); Serial.print(TEST_NAMES[index]);
  Serial.print(F(",QUEUED,ms=")); Serial.print(millis());
  Serial.print(F(",left=")); Serial.print(left);
  Serial.print(F(",right=")); Serial.print(right);
  Serial.print(F(",actual_left_pwm=")); Serial.print(adjustedPwm(left, leftPwmPercent));
  Serial.print(F(",actual_right_pwm=")); Serial.print(adjustedPwm(right, rightPwmPercent));
  Serial.print(F(",duration_ms=")); Serial.println(duration);
}

void logTestResult(uint8_t index, const char* result) {
  Serial.print(F("TEST,")); Serial.print(TEST_NAMES[index]);
  Serial.print(','); Serial.print(result);
  Serial.print(F(",ms=")); Serial.println(millis());
}

void printHelp() {
  Serial.println(F("ROBOT firmware 3 — stopped on startup"));
  Serial.println(F("FORWARD / BACKWARD / LEFT / RIGHT [PWM duration_ms]"));
  Serial.println(F("LEFT/RIGHT turn in place. Example: FORWARD 180 500"));
  Serial.println(F("STOP: cancel motion. HELP: this menu. STATUS: settings."));
  Serial.println(F("TEST_ALL [PWM duration_ms]: lab tests. P left% right%: trim."));
  Serial.println(F("M left right ms: individual wheels (-255..255). S and ? still work."));
  Serial.print(F("Default PWM=")); Serial.print(DEFAULT_TEST_PWM);
  Serial.print(F(" duration_ms=")); Serial.println(DEFAULT_TEST_MS);
  Serial.println(F("LOG ON / LOG OFF: live distance readings every 500 ms."));
  Serial.println(F("Sensors: logging only; obstacle stopping DISABLED."));
  reportStatus();
  reportSensor();
}

bool parseNumber(const char* text, long low, long high, long& value) {
  if (!text || !*text || strlen(text) > 6) return false;
  char* end;
  value = strtol(text, &end, 10);
  return *end == '\0' && value >= low && value <= high;
}

void invalidCommand() {
  stopDrive();
  Serial.println(F("ERROR: stopped. Invalid command; send ? for help."));
}

void processCommand(char* line) {
  char* tokens[5];
  uint8_t count = 0;
  char* token = strtok(line, " \t");
  while (token) {
    if (count == 5) { invalidCommand(); return; }
    tokens[count++] = token;
    token = strtok(NULL, " \t");
  }
  if (count == 0) return;
  for (char* c = tokens[0]; *c; ++c) *c = toupper((unsigned char)*c);
  long speed, right, duration;
  if (strcmp(tokens[0], "HELLO") == 0 && count == 1) {
    setDistanceLog(false);
    stopDrive(); controlSession = true; controlExpired = false; lastControlMs = millis();
    Serial.println(F("ROBOT_READY 3")); reportSensor(); return;
  }
  if (strcmp(tokens[0], "HEARTBEAT") == 0 && count == 1) {
    if (controlSession) { lastControlMs = millis(); Serial.println(F("ALIVE")); }
    return;
  }
  if (strcmp(tokens[0], "STATUS") == 0 && count == 1) { reportStatus(); reportSensor(); return; }
  if (strcmp(tokens[0], "LOG") == 0) {
    if (count != 2) { invalidCommand(); return; }
    for (char* c = tokens[1]; *c; ++c) *c = toupper((unsigned char)*c);
    if (strcmp(tokens[1], "ON") == 0) setDistanceLog(true);
    else if (strcmp(tokens[1], "OFF") == 0) setDistanceLog(false);
    else invalidCommand();
    return;
  }
  bool isStop = strcmp(tokens[0], "STOP") == 0 || strcmp(tokens[0], "S") == 0;
  bool isHelp = strcmp(tokens[0], "HELP") == 0 || strcmp(tokens[0], "?") == 0;
  if (controlExpired && !isStop && !isHelp) { invalidCommand(); return; }
  if (strcmp(tokens[0], "CONFIG") == 0) {
    long leftInvert, rightInvert;
    if (count != 5 || !parseNumber(tokens[1], 1, 200, speed) ||
        !parseNumber(tokens[2], 1, 200, right) || !parseNumber(tokens[3], 0, 1, leftInvert) ||
        !parseNumber(tokens[4], 0, 1, rightInvert)) { invalidCommand(); return; }
    stopDrive(); leftPwmPercent = speed; rightPwmPercent = right;
    INVERT_LEFT = leftInvert; INVERT_RIGHT = rightInvert;
    Serial.println(F("CONFIG_OK")); reportStatus(); return;
  }
  if (strcmp(tokens[0], "TEST") == 0) {
    long requestId;
    if (!controlSession || count != 5 || !parseNumber(tokens[1], 1, 999999, requestId) ||
        !parseNumber(tokens[2], -255, 255, speed) || !parseNumber(tokens[3], -255, 255, right) ||
        !parseNumber(tokens[4], 1, 5000, duration)) { invalidCommand(); return; }
    lastControlMs = millis(); scheduleDrive(speed, right, duration);
    if (speed == 0 && right == 0) { Serial.print(F("TEST_DONE ")); Serial.println(requestId); }
    else if (pending) remoteTestId = requestId;
    return;
  }
  if (strcmp(tokens[0], "DRIVE") == 0) {
    if (!controlSession || count != 3 || !parseNumber(tokens[1], -255, 255, speed) ||
        !parseNumber(tokens[2], -255, 255, right)) { invalidCommand(); return; }
    lastControlMs = millis(); updateLiveDrive(speed, right); return;
  }
  if (strcmp(tokens[0], "LEFT") == 0 || strcmp(tokens[0], "RIGHT") == 0) {
    speed = DEFAULT_TEST_PWM; duration = DEFAULT_TEST_MS;
    if (count != 1 && (count != 3 || !parseNumber(tokens[1], 0, 255, speed) ||
        !parseNumber(tokens[2], 1, 5000, duration))) { invalidCommand(); return; }
    runPreset(strcmp(tokens[0], "LEFT") == 0 ? 8 : 9, speed, duration); return;
  }

  for (uint8_t i = 0; i < 10; ++i) {
    if (strcmp(tokens[0], TEST_NAMES[i]) != 0) continue;
    if (count == 1) { runPreset(i, DEFAULT_TEST_PWM, DEFAULT_TEST_MS); return; }
    if (count != 3 || !parseNumber(tokens[1], 0, 255, speed) ||
        !parseNumber(tokens[2], 1, 5000, duration)) { invalidCommand(); return; }
    runPreset(i, (int)speed, (uint32_t)duration);
    return;
  }
  char command;
  if (strcmp(tokens[0], "STOP") == 0) command = 'S';
  else if (strcmp(tokens[0], "HELP") == 0) command = '?';
  else if (strcmp(tokens[0], "TEST_ALL") == 0) command = 'T';
  else if (strlen(tokens[0]) == 1) command = tokens[0][0];
  else { invalidCommand(); return; }
  if (command == 'S' && count == 1) {
    stopDrive(); Serial.println(F("STOP")); return;
  }
  if (command == '?' && count == 1) {
    stopDrive(); printHelp(); return;
  }
  if (command == 'P') {
    if (count != 3 || !parseNumber(tokens[1], 1, 200, speed) ||
        !parseNumber(tokens[2], 1, 200, right)) { invalidCommand(); return; }
    stopDrive();
    leftPwmPercent = (int)speed; rightPwmPercent = (int)right;
    Serial.print(F("PWM trim L/R: ")); Serial.print(leftPwmPercent);
    Serial.print('/'); Serial.println(rightPwmPercent);
    return;
  }
  if (command == 'T') {
    if (count == 1) { startTestSuite(DEFAULT_TEST_PWM, DEFAULT_TEST_MS); return; }
    if (count != 3 || !parseNumber(tokens[1], 1, 255, speed) ||
        !parseNumber(tokens[2], 1, 5000, duration)) { invalidCommand(); return; }
    startTestSuite((int)speed, (uint32_t)duration);
    return;
  }
  if (command == 'M') {
    if (count != 4 || !parseNumber(tokens[1], -255, 255, speed) ||
        !parseNumber(tokens[2], -255, 255, right) ||
        !parseNumber(tokens[3], 1, 5000, duration)) { invalidCommand(); return; }
    scheduleDrive((int)speed, (int)right, (uint32_t)duration);
    return;
  }
  if (count != 3 || !parseNumber(tokens[1], 0, 255, speed) ||
      !parseNumber(tokens[2], 1, 5000, duration)) { invalidCommand(); return; }
  switch (command) {
    case 'F': forward(speed, duration); break;
    case 'B': backward(speed, duration); break;
    case 'L': turnLeft(speed, duration); break;
    case 'R': turnRight(speed, duration); break;
    case 'Q': spinLeft(speed, duration); break;
    case 'E': spinRight(speed, duration); break;
    default: invalidCommand(); break;
  }
}

void readCommands() {
  // Limit work per loop so continuous serial input cannot starve the stop timer.
  for (uint8_t budget = 0; budget < 16 && Serial.available(); ++budget) {
    char c = (char)Serial.read();
    lastInputMs = millis();
    if (c == '!') {
      stopDrive(); lineLength = 0; discardLine = true;
      Serial.println(F("STOP (!). Send a newline before the next command."));
    } else if (c == '\n' || c == '\r') {
      if (!discardLine && lineLength) {
        lineBuffer[lineLength] = '\0';
        processCommand(lineBuffer);
      }
      lineLength = 0; discardLine = false;
    } else if (!discardLine) {
      if (lineLength >= sizeof(lineBuffer) - 1 || (c != '\t' && (c < 32 || c > 126))) {
        invalidCommand(); lineLength = 0; discardLine = true;
      } else {
        lineBuffer[lineLength++] = c;
      }
    }
  }
  if ((lineLength || discardLine) &&
      (uint32_t)(millis() - lastInputMs) >= INPUT_TIMEOUT_MS) {
    stopDrive(); lineLength = 0;
    // Discard any late suffix until a newline restores message framing.
    discardLine = true;
  }
}
