// UNO R4 WiFi / L298N. Serial Monitor: 115200 baud, Newline.
#include "Robot.h"

void setup() {
  setupMotor();
  setupSensor();
  setupLine();
  Serial.begin(115200);
  configureHardware(hardware::CAR);loadCalibration();setupStandalone();setupWireless();
  Serial.println(F("ROBOT_BOOT 7 stopped; HELP for commands"));
}

void loop() {
  serviceButton();
  serviceSensor();
  serviceOdometry();serviceObservation();
  serviceControlTimeout();
  readCommands();
  serviceNavigation();
  serviceMotor();
  serviceTelemetry();
  serviceMapExport();
  serviceWireless();
}
