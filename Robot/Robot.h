#pragma once
#include <Arduino.h>
#include <stdint.h>

// Wiring is unchanged. Directions below are logical robot directions.
const uint8_t ENA = 5, IN1 = 6, IN2 = 7, IN3 = 8, IN4 = 9, ENB = 10;
const uint8_t TRIG_PIN = A0, ECHO_PIN = A1, RIGHT_TRIG_PIN = A2, RIGHT_ECHO_PIN = A3;
const uint8_t GREEN_LED = 11;
const uint32_t SETTLE_MS = 250, CONTROL_TIMEOUT_MS = 500;
const uint32_t PING_MS = 70, ECHO_TIMEOUT_US = 30000, STALE_MS = 350;
const uint32_t LOG_MS = 100, TURN_LIMIT_MS = 2500;
const float FOLLOW_KP = 6.0f, WALL_KP = 14.0f, DEADBAND_CM = 2.0f;
// Starting gains/timing for floor calibration; distances are measured at sensors.
const float WALL_KD = 12.0f, WALL_D_ALPHA = 0.4f, WALL_RATE_LIMIT = 50.0f;
const uint32_t BACKUP_MS = 250, TURN_MIN_MS = 160, SEARCH_LIMIT_MS = 4000;
const uint32_t SENSOR_GRACE_MS = 300, ALIGN_MS = 560, ALIGN_LIMIT_MS = 1500;
const float FRONT_BACKUP_CM = 10, FRONT_TURN_CM = 32, FRONT_CLEAR_CM = 55;
const float CORNER_CLEAR_RATIO = 2.5f, CORNER_NO_SIDE_CLEAR_CM = 90.0f;
const float TURN_KP = 12.0f; // PWM per cm of front-clearance error.
const float FOLLOW_MAX_CM = 150.0f, WALL_MAX_CM = 100.0f;

struct Settings {
  int power = 255;
  int minimum = 255; // Full starting power requested for this robot.
  bool invertA = true, invertB = false; // User-tested forward polarity: A reversed, B normal.
  int followCm = 20, wallCm = 15;
};
enum Mode { MANUAL, FOLLOW, WALL };
extern Settings settings;
extern Mode mode;
extern const char* navigationState;
extern int motorLeft, motorRight;
extern bool controlSession, controlExpired;

void setupMotor();
void requestMotor(int left, int right);
void stopMotor();
void settleMotor();
void serviceMotor();
void startTest(int left, int right, uint32_t duration);
void setupSensor();
void serviceSensor();
struct SensorReading {
  float cm;
  uint32_t sampledMs;
  uint32_t sequence;
};
SensorReading sensorReading(uint8_t sensor);
float sensorDistanceCm(uint8_t sensor);
void cancelNavigation();
void stopRobot();
void startNavigation(Mode selected);
void serviceNavigation();
const char* modeName();
void readCommands();
void processCommand(char* line);
void serviceControlTimeout();
void serviceTelemetry();
void reportStatus();
