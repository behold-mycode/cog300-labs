// Deterministic host transport for testing the actual .ino parser with the Python controller.
#include <iostream>
#include <sstream>
#include "../src/MyRobot-V1/MyRobot-V1.ino"
#include "../src/MyRobot-V1/Motor.ino"
#include "../src/MyRobot-V1/Serial.ino"
int main() {
  setup(); Serial.output.str("");
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.rfind("@tick ", 0) == 0) { nowMs += std::stoul(line.substr(6)); loop(); }
    else {
      for (char c : line + "\n") Serial.input.push_back(c);
      while (Serial.available()) loop();
    }
    std::cout << Serial.output.str(); Serial.output.str(""); Serial.output.clear();
    std::cout << "@state " << pwm[9] << ' ' << pwm[10] << '\n' << "@end" << std::endl;
  }
}
