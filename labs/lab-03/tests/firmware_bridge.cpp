// Deterministic host transport for testing the actual .ino parser with the Python controller.
#include <iostream>
#include <sstream>
#include "../src/MyRobot-V2/MyRobot-V2.ino"
#include "../src/MyRobot-V2/Motor.ino"
#include "../src/MyRobot-V2/Serial.ino"
#include "../src/MyRobot-V2/Safety.ino"
int simulatedCm = 100;
void sensorLoop() {
  pingPending = false;
  acceptDistance(simulatedCm);
  loop();
}
int main() {
  setup(); Serial.output.str("");
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.rfind("@tick ", 0) == 0) { nowMs += std::stoul(line.substr(6)); sensorLoop(); }
    else if (line.rfind("@distance ", 0) == 0) { simulatedCm = std::stoi(line.substr(10)); sensorLoop(); }
    else {
      for (char c : line + "\n") Serial.input.push_back(c);
      while (Serial.available()) sensorLoop();
    }
    std::cout << Serial.output.str(); Serial.output.str(""); Serial.output.clear();
    std::cout << "@state " << pwm[ENA] << ' ' << pwm[ENB] << '\n' << "@end" << std::endl;
  }
}
