#include <cassert>
#include <iostream>
#include "../src/MyRobot-V2/MyRobot-V2.ino"
#include "../src/MyRobot-V2/Motor.ino"
#include "../src/MyRobot-V2/Serial.ino"
#include "../src/MyRobot-V2/Safety.ino"

void command(const char* text) {
  char buffer[64]; strncpy(buffer, text, sizeof(buffer)); processCommand(buffer);
}
void resetOutput() { Serial.output.str(""); Serial.output.clear(); }
int main() {
  setup(); resetOutput();
  nowMs = 500; serviceDistanceLog(); assert(Serial.output.str().empty());
  command("log on"); resetOutput();
  nowMs = 999; serviceDistanceLog(); assert(Serial.output.str().empty());
  // Exercise actual echo acquisition and conversion, including a fractional cm.
  nowMs = 1000; serviceSensor();
  levels[ECHO_PIN] = HIGH; echoChanged();
  extraUs += 1752; levels[ECHO_PIN] = LOW; echoChanged(); serviceSensor();
  resetOutput(); serviceDistanceLog();
  assert(Serial.output.str() == "t=1.0s | front=30.2 cm OK | right=-- INVALID\n");
  assert(samples[FRONT_SENSOR].cm == 30); assert(pwm[ENA] == 0 && pwm[ENB] == 0);
  resetOutput(); serviceDistanceLog(); assert(Serial.output.str().empty());
  nowMs = 1500; serviceDistanceLog(); // Stale value must not be displayed.
  assert(Serial.output.str().find("-- INVALID") != std::string::npos);
  resetOutput(); acceptDistance(-1); nowMs = 2000; serviceDistanceLog();
  assert(Serial.output.str().find("-- INVALID") != std::string::npos);
  command("LOG OFF"); resetOutput(); nowMs += 500; serviceDistanceLog();
  assert(Serial.output.str().empty());
  command("LOG ON"); command("HELLO"); assert(!distanceLogEnabled);
  command("LOG MAYBE"); assert(Serial.output.str().find("ERROR") != std::string::npos);
  // Unsigned elapsed-time calculation survives millis rollover.
  nowMs = UINT32_MAX - 200; command("LOG ON"); resetOutput();
  nowMs = 298; serviceDistanceLog(); assert(Serial.output.str().empty());
  nowMs = 299; serviceDistanceLog(); assert(!Serial.output.str().empty());
  std::cout << "Distance logging tests passed\n";
}
