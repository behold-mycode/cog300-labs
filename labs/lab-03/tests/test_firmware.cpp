#include <cassert>
#include <iostream>
#include "../src/MyRobot-V2/Robot.h"
#include "../src/MyRobot-V2/Motor.cpp"
#include "../src/MyRobot-V2/Sensors.cpp"
#include "../src/MyRobot-V2/Navigation.cpp"
#include "../src/MyRobot-V2/Protocol.cpp"
#include "../src/MyRobot-V2/MyRobot-V2.ino"
void command(const char* text) { char line[96]; strcpy(line, text); processCommand(line); }
void readings(int front, int right) { acceptReading(0, front, front * 58UL); acceptReading(1, right, right * 58UL); }
int main() {
  setup(); assert(pwm[5] == 0 && pwm[10] == 0);
  assert(ENA == 5 && IN1 == 6 && IN2 == 7 && IN3 == 8 && IN4 == 9 && ENB == 10);
  command("HELLO"); command("DRIVE 255 255"); serviceMotor();
  assert(motorLeft == 255 && motorRight == 255);
  assert(levels[IN1] == LOW && levels[IN2] == HIGH);
  assert(levels[IN3] == HIGH && levels[IN4] == LOW);
  command("DRIVE -255 -255"); serviceMotor(); assert(motorLeft == 0);
  nowMs = 100; command("DRIVE -255 -255"); serviceMotor(); assert(motorLeft == 0);
  nowMs = 250; command("DRIVE -255 -255"); serviceMotor(); assert(motorLeft == -255);
  command("STOP"); serviceMotor(); assert(motorLeft == 0 && mode == MANUAL);
  command("CONFIG 120 40 1 0 25 30"); command("DRIVE 255 -255");
  nowMs += 250; serviceMotor(); assert(abs(motorLeft) == 120 && abs(motorRight) == 120);
  command("CONFIG 255 255 0 1 20 20");
  readings(35, 20); command("MODE FOLLOW"); serviceNavigation(); serviceMotor();
  nowMs += 250; readings(35,20); serviceNavigation(); serviceMotor(); assert(motorLeft == 255 && motorRight == 255);
  readings(20,20); serviceNavigation(); serviceMotor(); assert(motorLeft == 0);
  readings(15,20); serviceNavigation(); nowMs += 250; readings(15,20); serviceNavigation(); serviceMotor(); assert(motorLeft == -255);
  readings(-1,20); serviceNavigation(); serviceMotor(); assert(motorLeft == 0);
  readings(151,20); serviceNavigation(); serviceMotor(); assert(motorLeft == 0);
  readings(50,30); command("MODE WALL"); serviceNavigation(); nowMs += 250; readings(50,30); serviceNavigation(); serviceMotor();
  assert(motorLeft > motorRight && motorLeft == 255);
  readings(50,10); serviceNavigation(); serviceMotor(); assert(motorRight > motorLeft);
  readings(10,20); serviceNavigation(); serviceMotor(); nowMs += 250; readings(10,20); serviceNavigation(); serviceMotor(); assert(motorLeft < 0 && motorRight > 0);
  nowMs += TURN_LIMIT_MS; readings(10,20); serviceNavigation(); serviceMotor(); assert(mode == MANUAL && motorLeft == 0);
  command("HELLO"); command("DRIVE 255 255"); nowMs += 500; serviceControlTimeout(); serviceMotor(); assert(motorLeft == 0);
  command("DRIVE 255 255"); serviceMotor(); assert(motorLeft == 0);
  command("HELLO"); command("DRIVE 255 255"); command("CONFIG 999 255 0 1 20 20"); serviceMotor(); assert(motorLeft == 0);
  command("TEST 255 0 100"); nowMs += 250; serviceMotor(); assert(motorLeft == 255 && motorRight == 0);
  nowMs += 100; serviceMotor(); assert(motorLeft == 0);
  command("LOG ON"); Serial.output.str(""); serviceTelemetry(); assert(Serial.output.str().empty());
  nowMs += 500; serviceTelemetry(); assert(Serial.output.str().find("DATA,") == 0);
  Serial.output.str(""); serviceTelemetry(); assert(Serial.output.str().empty());
  readings(50,20); nowMs += STALE_MS; assert(sensorDistanceCm(0) < 0);
  std::cout << "PASS firmware: pins, power, inversion, reversal, modes, watchdog, parsing, telemetry\n";
}
