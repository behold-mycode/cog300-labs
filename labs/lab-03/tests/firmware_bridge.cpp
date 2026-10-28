// Deterministic host serial transport around the actual firmware.
#include <iostream>
#include <sstream>
#include "../src/MyRobot-V2/Robot.h"
#include "../src/MyRobot-V2/Motor.cpp"
#include "../src/MyRobot-V2/Sensors.cpp"
#include "../src/MyRobot-V2/Navigation.cpp"
#include "../src/MyRobot-V2/Protocol.cpp"
#include "../src/MyRobot-V2/MyRobot-V2.ino"
int front = 100, right = 20;
void step() {
  acceptReading(0,front,front > 0 ? front * 58UL : 0);
  acceptReading(1,right,right > 0 ? right * 58UL : 0);
  serviceControlTimeout(); readCommands(); serviceNavigation(); serviceMotor(); serviceTelemetry();
}
int main() {
  setup(); Serial.output.str("");
  std::string line;
  while (std::getline(std::cin,line)) {
    if (line.rfind("@tick ",0) == 0) { nowMs += std::stoul(line.substr(6)); step(); }
    else if (line.rfind("@distance ",0) == 0) {
      std::istringstream values(line.substr(10)); values >> front >> right; step();
    } else {
      for (char c : line + "\n") Serial.input.push_back(c);
      while (Serial.available()) step();
    }
    std::cout << Serial.output.str(); Serial.output.str(""); Serial.output.clear();
    std::cout << "@state " << motorLeft << ' ' << motorRight << '\n' << "@end" << std::endl;
  }
}
