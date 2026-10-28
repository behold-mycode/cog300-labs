#include <cassert>
#include <iostream>
#include "../src/MyRobot-V2/Robot.h"
#include "../src/MyRobot-V2/Motor.cpp"
#include "../src/MyRobot-V2/Sensors.cpp"
#include "../src/MyRobot-V2/Navigation.cpp"
#include "../src/MyRobot-V2/Protocol.cpp"
#include "../src/MyRobot-V2/MyRobot-V2.ino"
void command(const char* text) { char line[96]; strcpy(line,text); processCommand(line); }
void feed(const std::string& text) {
  for(char c:text) Serial.input.push_back(c);
  while(Serial.available()) readCommands();
}
int main() {
  setup(); command("HELLO");
  command("DRIVE 255 255"); serviceMotor(); assert(motorLeft==255);
  command("CONFIG 255 255 1 1 20 20"); command("DRIVE 255 255"); serviceMotor();
  assert(motorLeft==0); nowMs=249; serviceMotor(); assert(motorLeft==0);
  nowMs=250; serviceMotor(); assert(motorLeft==255 && levels[IN1]==HIGH);
  feed(std::string(100,'x')+"DRIVE 255 255\n"); serviceMotor(); assert(motorLeft==0);
  command("DRIVE 255 255"); serviceMotor(); feed("!DRIVE 255 255\n"); serviceMotor(); assert(motorLeft==0);
  feed("DRIVE 255"); nowMs+=1000; readCommands(); feed(" 255\n"); serviceMotor(); assert(motorLeft==0);
  command("DRIVE 255 255 extra"); serviceMotor(); assert(motorLeft==0);
  command("CONFIG 100 101 0 1 20 20"); assert(settings.power==255);
  command("CONFIG 100 20 0 1 20 20"); nowMs+=250;
  acceptReading(0,25,1450); command("MODE FOLLOW"); serviceNavigation(); serviceMotor();
  assert(motorLeft==30 && motorRight==30); // Actual proportional output with battery settings.
  command("TEST 100 100 10"); serviceMotor(); command("DRIVE 100 100"); serviceMotor();
  nowMs+=20; serviceMotor(); assert(motorLeft==100); // New live command cancels old timed test.
  command("STOP"); command("DRIVE -100 -100"); serviceMotor(); nowMs+=100;
  command("STOP"); nowMs+=250; serviceMotor(); assert(motorLeft==0);
  nowMs=UINT32_MAX-200; command("HELLO"); command("DRIVE 100 100"); serviceMotor();
  nowMs+=499; serviceControlTimeout(); assert(controlSession);
  nowMs+=1; serviceControlTimeout(); serviceMotor(); assert(!controlSession && motorLeft==0);
  command("MODE WALL"); assert(mode==MANUAL);
  std::cout << "PASS protocol: framing, overflow, config atomicity, physical reversal, timed cancellation, watchdog rollover\n";
}
