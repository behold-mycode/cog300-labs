#include <cassert>
#include <iostream>
#include "../Robot/Motor.cpp"
#include "../Robot/Sensors.cpp"
#include "../Robot/Navigation.cpp"
void tick(int front, int right, unsigned ms = 20) {
  nowMs += ms;
  acceptReading(0, front, front > 0 ? front * 58 : 0);
  acceptReading(1, right, right > 0 ? right * 58 : 0);
  serviceNavigation(); serviceMotor();
}
void begin() { stopRobot(); nowMs += 1000; settings.power=150; settings.minimum=80; settings.wallCm=12; startNavigation(WALL); }
int main() {
  setupMotor();
  begin(); tick(80,13); assert(motorLeft-motorRight == 8);
  tick(80,14,140); assert(motorLeft-motorRight == 16);
  tick(80,10,140); assert(motorRight-motorLeft == 16);
  begin(); tick(18,12); tick(18,12,300);
  int fastTurn = motorRight;
  tick(30,12); assert(motorRight > 0 && motorRight < fastTurn);

  begin(); tick(8, -1);
  assert(motorLeft < 0 && motorRight < 0); // Front recovery overrides missing side echo.
  begin(); tick(18,12); tick(18,12,300); tick(26,12);
  assert(motorLeft < 0 && motorRight > 0); // Clearance alone must not end a new turn.
  begin(); tick(80,150);
  assert(motorLeft > motorRight && motorRight > 0); // Search toward a distant wall.
  for(int i=0;i<300 && mode==WALL;i++) tick(80,150);
  assert(mode==MANUAL && motorLeft==0 && motorRight==0); // Bounded search.
  begin(); tick(80,12); assert(motorLeft==motorRight && motorLeft>0);
  tick(80,16,140); assert(motorLeft>motorRight);
  tick(80,8,140); assert(motorRight>motorLeft);
  tick(-1,12); assert(motorLeft==0 && motorRight==0);
  begin(); tick(18,12);
  for(int i=0;i<200 && mode==WALL;i++) tick(18,12);
  assert(mode==MANUAL && motorLeft==0);
  begin(); tick(8,12);
  for(int i=0;i<40;i++) tick(18,12);
  assert(motorLeft<0 && motorRight>0); // Backup completes, then turn.
  for(int i=0;i<40;i++) tick(60,12);
  assert(mode==WALL && motorLeft>0 && motorRight>0);
  stopRobot(); tick(80,150); assert(motorLeft==0 && motorRight==0);
  std::cout << "PASS wall recovery, turn commitment, search, tracking, stop and timeouts\n";
}
