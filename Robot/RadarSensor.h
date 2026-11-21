#pragma once
#include "Robot.h"

// UNO R4 WiFi D1 (P302/GTIOC4A). D5 and D10 motor PWM use GPT0 and GPT2.
// Calibrate these values with the scanner bracket mounted, before driving.
#ifndef RADAR_CENTER_US
#define RADAR_CENTER_US 1500
#endif
#ifndef RADAR_LEFT_SIGN
#define RADAR_LEFT_SIGN 1
#endif
static_assert(RADAR_LEFT_SIGN == 1 || RADAR_LEFT_SIGN == -1, "Use +1 or -1 for the mounting direction");
constexpr int RADAR_MAX_DEGREES = 60;
constexpr int RADAR_US_PER_DEGREE = 6;
static_assert(RADAR_CENTER_US - RADAR_MAX_DEGREES * RADAR_US_PER_DEGREE >= 1000 &&
              RADAR_CENTER_US + RADAR_MAX_DEGREES * RADAR_US_PER_DEGREE <= 2000,
              "Scanner pulse range must fit the provisional 1-2 ms limits");

bool radarServoCalibration(int rightUs,int centreUs,int leftUs);
void radarEnable();
bool radarReady();  // PWM initialized successfully; samples still require settling.
void radarAim(int degrees); // Positive left in the chassis frame.
int radarBearing();
SensorReading radarReading(); // sequence zero while waiting for a new aim's first observation.
