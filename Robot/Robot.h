#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "Odometry.h"
#include "Explorer.h"

const uint8_t ENA=5,IN1=6,IN2=7,IN3=8,IN4=9,ENB=10;
const uint8_t TRIG_PIN=A0,ECHO_PIN=A1,RIGHT_TRIG_PIN=A2,RIGHT_ECHO_PIN=A3;
const uint8_t GREEN_LED=11,LINE_LEFT_PIN=3,LINE_CENTER_PIN=2,LINE_RIGHT_PIN=4;
const uint32_t SETTLE_MS=250,CONTROL_TIMEOUT_MS=500;
const uint32_t PING_MS=70,ECHO_TIMEOUT_US=45000,STALE_MS=350,LOG_MS=100;
const float FOLLOW_KP=6,DEADBAND_CM=2,FOLLOW_MAX_CM=150;
struct Settings {
  int power=255,minimum=255;
  bool invertA=true,invertB=false;
  int followCm=20,wallCm=10;
};
struct NavigationConfig {
  // Existing user-requested distance. Validate physically before floor operation.
  float frontStopCm=20,wallBandCm=2;
  uint32_t stallMs=1500; // Failure deadline, never a movement duration.
};
enum Mode { MANUAL,FOLLOW,WALL,LINE,MAZE,STOP_ON_TAPE,STOP_AT_END };
extern Settings settings;
extern NavigationConfig navigationConfig;
extern Mode mode;
extern const char* navigationState;
extern int motorLeft,motorRight,lineTapeLevel;
extern bool controlSession,controlExpired;
extern maze::Odometry odometry;
extern maze::Explorer explorer;
void setupMotor();void requestMotor(int left,int right);void stopMotor();void settleMotor();void serviceMotor();
void startTest(int left,int right,uint32_t duration);
void setupSensor();void serviceSensor();
enum RangeStatus { RANGE_VALID,RANGE_NO_RETURN,RANGE_INVALID,RANGE_STALE };
struct SensorReading { float cm;uint32_t sampledMs,sequence;RangeStatus status; };
SensorReading sensorReading(uint8_t sensor);float sensorDistanceCm(uint8_t sensor);
void setupLine();uint8_t lineMask();void reportLine();
void serviceObservation();void setObservationTrace(bool enabled);
void cancelNavigation();void stopRobot();void startNavigation(Mode selected);void serviceNavigation();const char* modeName();
void readCommands();void processCommand(char* line);void serviceControlTimeout();void serviceTelemetry();void reportStatus();
bool configureEncoders(int kind,bool invertL,bool invertR);bool encodersEnabled();
void reportEncoders();void serviceOdometry();void resetOdometry();
bool mapExportActive();void beginMapExport();void serviceMapExport();void reportNavigation();
