#include "Robot.h"

Settings settings;
int motorLeft = 0, motorRight = 0;
static int wantedLeft = 0, wantedRight = 0;
static int lastLeftSign = 0, lastRightSign = 0;
static uint32_t leftOffAt = 0, rightOffAt = 0;
static bool lineDrive=false, wantBrakeLeft=false, wantBrakeRight=false;
static bool appliedBrakeLeft=false, appliedBrakeRight=false;
static bool settling = false;
static uint32_t settleStarted = 0;
static bool testPending = false, testRunning = false;
static uint32_t testStarted = 0, testDuration = 0;

static int sign(int value) { return (value > 0) - (value < 0); }
static int clampPower(int value) {
  if (value > settings.power) return settings.power;
  if (value < -settings.power) return -settings.power;
  return value;
}
static void writeMotor(uint8_t enable, uint8_t a, uint8_t b, int value, bool invert,bool brake) {
  analogWrite(enable, 0); // Disable bridge before changing direction pins.
  if (invert) value = -value;
  digitalWrite(a, value < 0 ? HIGH : LOW);
  digitalWrite(b, value > 0 ? HIGH : LOW);
  analogWrite(enable, brake && !value ? 255 : abs(value));
}
static void output(int left,int right,bool brakeLeft=false,bool brakeRight=false) {
  static bool initialized=false;
  bool changeLeft=!initialized||left!=motorLeft||brakeLeft!=appliedBrakeLeft;
  bool changeRight=!initialized||right!=motorRight||brakeRight!=appliedBrakeRight;
  initialized=true;appliedBrakeLeft=brakeLeft;appliedBrakeRight=brakeRight;
  uint32_t now = millis();
  if (motorLeft && !left) leftOffAt = now;
  if (motorRight && !right) rightOffAt = now;
  if (left) lastLeftSign = sign(left);
  if (right) lastRightSign = sign(right);
  updateEncoderDirections(left,right);
  motorLeft = left; motorRight = right;
  // On this car A drives the physical right wheel, B drives the left.
  // Keep logical wheel commands independent of channel polarity calibration.
  if(changeRight)writeMotor(ENA, IN1, IN2, right, settings.invertA,brakeRight);
  if(changeLeft)writeMotor(ENB, IN3, IN4, left, settings.invertB,brakeLeft);
}
void setupMotor() {
  const uint8_t pins[] = {ENA, IN1, IN2, IN3, IN4, ENB};
  for (uint8_t pin : pins) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
  stopMotor();
}
void stopMotor() {
  lineDrive=false;wantedLeft = wantedRight = 0;
  testPending = testRunning = false;
  output(0, 0);
}
void settleMotor() { stopMotor(); settling = true; settleStarted = millis(); }
void requestMotor(int left, int right) {
  testPending = testRunning = false;
  lineDrive=false;wantedLeft = clampPower(left); wantedRight = clampPower(right);
}
void requestLineMotor(int left,int right,bool brakeLeft,bool brakeRight){
  requestMotor(left,right);
  lineDrive=true;wantBrakeLeft=brakeLeft;wantBrakeRight=brakeRight;
}
void startTest(int left, int right, uint32_t duration) {
  stopRobot(); requestMotor(left, right);
  testDuration = duration; testPending = left || right;
}
void serviceMotor() {
  uint32_t now = millis();
  if (testRunning && (uint32_t)(now - testStarted) >= testDuration) {
    stopMotor(); Serial.println(F("TEST_DONE")); return;
  }
  if (settling) {
    if ((uint32_t)(now - settleStarted) < SETTLE_MS) return;
    settling = false;
  }
  // Preserve reversal history across STOP and configuration changes. Repeated
  // drive frames update the target without resetting the unpowered interval.
  bool leftReverses = wantedLeft && lastLeftSign && sign(wantedLeft) != lastLeftSign;
  bool rightReverses = wantedRight && lastRightSign && sign(wantedRight) != lastRightSign;
  if(lineDrive){
    // Preserve the reversal dwell on the affected wheel; the outside wheel keeps driving.
    bool waitLeft=leftReverses&&(motorLeft||(uint32_t)(now-leftOffAt)<SETTLE_MS);
    bool waitRight=rightReverses&&(motorRight||(uint32_t)(now-rightOffAt)<SETTLE_MS);
    int left=waitLeft?0:wantedLeft,right=waitRight?0:wantedRight;
    bool bl=waitLeft||(!left&&wantBrakeLeft),br=waitRight||(!right&&wantBrakeRight);
    if(left!=motorLeft||right!=motorRight||bl!=appliedBrakeLeft||br!=appliedBrakeRight)
      output(left,right,bl,br);
    return;
  }
  if ((leftReverses && motorLeft) || (rightReverses && motorRight)) output(0, 0);
  if ((leftReverses && (uint32_t)(now - leftOffAt) < SETTLE_MS) ||
      (rightReverses && (uint32_t)(now - rightOffAt) < SETTLE_MS)) return;
  if (motorLeft != wantedLeft || motorRight != wantedRight || appliedBrakeLeft || appliedBrakeRight) output(wantedLeft, wantedRight);
  if (testPending) {
    testPending = false; testRunning = true; testStarted = now;
    Serial.println(F("TEST_START"));
  }
}

bool motorDemandActive(){return wantedLeft||wantedRight||testPending||testRunning;}
