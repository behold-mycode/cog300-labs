#include <cassert>
#include "../Robot/Robot.h"
#include "../Robot/Motor.cpp"
void stopRobot() { stopMotor(); }
void tick(uint32_t ms) { nowMs += ms; serviceMotor(); }
int main() {
  setupMotor(); requestMotor(255,255); tick(0);
  // Continuous output: no imposed burst/pause cycle at any prior boundary.
  for (int i=0;i<200;++i) {
    requestMotor(255,255); tick(10);
    assert(motorLeft==255 && motorRight==255);
  }
  requestMotor(0,255); tick(0); assert(motorLeft==0 && motorRight==255);
  tick(1000); assert(motorLeft==0 && motorRight==255);
  requestMotor(255,-255); tick(0); assert(motorLeft==0 && motorRight==0);
  tick(249); assert(motorLeft==0 && motorRight==0);
  tick(1); assert(motorLeft==255 && motorRight==-255);
  tick(1000); assert(motorLeft==255 && motorRight==-255);
  stopMotor(); tick(1000); assert(motorLeft==0 && motorRight==0);
  // Timed diagnostic commands still stop, and time starts after reversal.
  startTest(255,255,100); tick(0); assert(motorLeft==255);
  tick(99); assert(motorLeft==255);
  tick(1); assert(motorLeft==0 && motorRight==0);
}
