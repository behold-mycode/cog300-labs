// MyRobot-V2.ino — main sketch, matching the course template.
// Arduino Uno + L298N. Starts stopped; Serial Monitor: 9600 baud, Newline.
#include <Arduino.h>
#include <stdint.h>

// Ordered wiring: sensors on A0-A3; motor header on D5-D10; LED on D11.
const uint8_t TRIG_PIN = A0, ECHO_PIN = A1;
const uint8_t RIGHT_TRIG_PIN = A2, RIGHT_ECHO_PIN = A3;
const uint8_t GREEN_LED = 11;
const uint8_t ENA = 5, IN1 = 6, IN2 = 7;   // Left motor: OUT1 / OUT2
const uint8_t ENB = 10, IN3 = 8, IN4 = 9;  // Right motor: OUT3 / OUT4
// Test one wheel at a time. Change ONLY the corresponding flag if it runs backward.
bool INVERT_LEFT = false;
bool INVERT_RIGHT = true; // Verify using the Wheels preset; configurable in the Mac controller.

// Cross-tab entry points; explicit declarations also support the host tests.
void setupSensor();
void serviceDistanceLog();
void setDistanceLog(bool enabled);
void serviceSensor();
void reportSensor();
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
  setupSensor();
  printHelp();
}

void loop() {
  serviceSensor();
  serviceControlTimeout();
  readCommands();
  serviceMotion();
  serviceTestSuite();
  serviceDistanceLog();
}
