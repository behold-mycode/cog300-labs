#include <cassert>
#include <iostream>
#include "../Robot/Robot.h"
#include "../Robot/Motor.cpp"
void stopRobot() { stopMotor(); }
// User-tested polarity: channel A reversed, channel B normal.
int rightWheel() { return levels[IN1] == HIGH ? pwm[ENA] : -pwm[ENA]; }
int leftWheel() { return levels[IN4] == HIGH ? pwm[ENB] : -pwm[ENB]; }
void drive(int left, int right) {
  requestMotor(left, right); serviceMotor(); nowMs += SETTLE_MS; serviceMotor();
}
int main() {
  settings.power = 255; // Exercise full-range commands independently of battery defaults.
  setupMotor();
  drive(255,255); assert(leftWheel()==255 && rightWheel()==255);
  drive(-255,255); assert(leftWheel()==-255 && rightWheel()==255); // Left arrow.
  drive(255,-255); assert(leftWheel()==255 && rightWheel()==-255); // Right arrow.
  drive(127,255); assert(leftWheel()==127 && rightWheel()==255); // Forward-left curve.
  drive(255,127); assert(leftWheel()==255 && rightWheel()==127);
  drive(-255,-255); assert(leftWheel()==-255 && rightWheel()==-255);
  stopRobot(); assert(pwm[ENA]==0 && pwm[ENB]==0);
  std::cout << "PASS physical motor mapping: forward, reverse, left/right spins, curves, stop\n";
}
