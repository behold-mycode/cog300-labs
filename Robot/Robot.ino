// UNO R4 WiFi / L298N. Serial Monitor: 115200 baud, Newline.
#include "Robot.h"

void setup() {
  setupMotor();
  setupSensor();
  setupLine();
  Serial.begin(115200);
  configureHardware(hardware::CAR);loadCalibration();setupStandalone();setupWireless();
  if(ROBOT_LINE_ONLY){
    lineTapeLevel=0; // Installed sensors: LOW tape, HIGH floor.
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
