// MyRobot-V2.ino — main sketch, matching the course template.
// Arduino Uno + L298N. Starts stopped; Serial Monitor: 9600 baud, Newline.
#include <Arduino.h>
#include <stdint.h>

const uint8_t ENA = 9, IN1 = 8, IN2 = 7;   // Left motor: OUT1 / OUT2
const uint8_t ENB = 10, IN3 = 6, IN4 = 5;  // Right motor: OUT3 / OUT4
// Test one wheel at a time. Change ONLY the corresponding flag if it runs backward.
bool INVERT_LEFT = false;
bool INVERT_RIGHT = true; // Verify using the Wheels preset; configurable in the Mac controller.

// Cross-tab entry points; explicit declarations also support the host tests.
void setupSafety();
void serviceDistanceLog();
void setDistanceLog(bool enabled);
void serviceSafety();
bool allowMotion(int left, int right);
void reportSafety();
void stopDrive();
void serviceMotion();
void serviceTestSuite();
void logTestStart(uint8_t index, int left, int right, uint32_t duration);
void logTestResult(uint8_t index, const char* result);
void printHelp();
void readCommands();
void serviceControlTimeout();

void setup() {
  // LOW output latches before enabling outputs; no automatic movement on reset.
  digitalWrite(ENA, LOW); digitalWrite(ENB, LOW);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopDrive();
  Serial.begin(9600);
  Serial.println(F("Lab03Robot READY — stopped"));
  setupSafety();
  printHelp();
}

void loop() {
  serviceSafety();
  serviceControlTimeout();
  readCommands();
  serviceMotion();
  serviceTestSuite();
  serviceDistanceLog();
}
