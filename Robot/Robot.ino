// UNO R4 WiFi / L298N. Serial Monitor: 115200 baud, Newline.
#include "Robot.h"

const char* bootResetCause = "Unknown";
uint8_t bootFlags0 = 0;
uint16_t bootFlags1 = 0;

void setup() {
#ifdef R_SYSTEM
  bootFlags0 = R_SYSTEM->RSTSR0;
  bootFlags1 = R_SYSTEM->RSTSR1;
  if (bootFlags0 & 0x0e) bootResetCause = "Low voltage";
  else if (bootFlags1 & 3) bootResetCause = "Watchdog";
  else if (bootFlags1 & 4) bootResetCause = "Software reset";
  else if (bootFlags1 & 0x1f00) bootResetCause = "CPU or memory fault";
  else if (bootFlags0 & 1) bootResetCause = "Power on";
  R_SYSTEM->RSTSR0 = 0;
  R_SYSTEM->RSTSR1 = 0;
#endif
  setupMotor();
  setupSensor();
  setupLine();
  Serial.begin(115200);
  configureHardware(hardware::CAR);lineTapeLevel=0;loadCalibration();setupStandalone();setupWireless();
  // Eight-cell PI calibration in actual PWM units; full 0..255 output available.
  #if ROBOT_EIGHT_CELL_PROFILE
  configureLineSpeeds(330,150,330,125);
  configureLineMotor(200,18.82f,.2353f,.5647f,61);
  configureLineDrive(66,0);
  #endif
  if(ROBOT_LINE_ONLY){
    armBatteryLine();
    Serial.println(F("ROBOT_BOOT 7 LINE_ONLY automatic start in 1500ms"));
  }else Serial.println(F("ROBOT_BOOT 7 stopped; HELP for commands"));
}

void loop() {
  serviceButton();
  serviceSensor();
  serviceOdometry();serviceObservation();
  serviceControlTimeout();
  readCommands();
  serviceBatteryLine();serviceNavigation();
  serviceMotor();
  serviceTelemetry();
  serviceMapExport();
  serviceWireless();
  serviceSerialOutput();
}
