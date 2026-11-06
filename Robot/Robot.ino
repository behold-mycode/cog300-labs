// UNO R4 WiFi / L298N. Serial Monitor: 115200 baud, Newline.
#include "Robot.h"

void setup() {
  setupMotor();
  setupSensor();
  setupLine();
  Serial.begin(115200);
  Serial.println(F("ROBOT_BOOT 6 stopped; HELP for commands"));
}

void loop() {
  serviceSensor();
  serviceOdometry();serviceObservation();
  serviceControlTimeout();
  readCommands();
  serviceNavigation();
  serviceMotor();
  serviceTelemetry();
  serviceMapExport();
}
