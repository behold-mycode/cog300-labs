#include <Arduino.h>
// Motor commands retain their full requested magnitude and turn ratio.
#include <cassert>
#include "../Robot/Robot.h"
#include "../Robot/Motor.cpp"
void updateEncoderDirections(int,int) {}
void stopRobot() { stopMotor(); }
int main() {
  setupMotor();
  requestMotor(255,128); serviceMotor();
  assert(motorLeft==255 && motorRight==128);
  assert(pwm[ENB]==255 && pwm[ENA]==128);
  stopMotor(); nowMs+=250;
  requestMotor(-128,255); serviceMotor();
  assert(motorLeft==-128 && motorRight==255);
  stopMotor(); nowMs+=250;
  requestMotor(0,-255); serviceMotor();
  assert(motorLeft==0 && motorRight==-255);
  stopMotor(); assert(pwm[ENA]==0 && pwm[ENB]==0);
}
