#include <cassert>
#include <iostream>
#include "../Robot/Motor.cpp"
#include "../Robot/Sensors.cpp"
#include "../Robot/Navigation.cpp"
void idle(unsigned ms) { nowMs += ms; serviceNavigation(); serviceMotor(); }
void tick(int front, int right, unsigned ms = 140) {
  nowMs += ms;
  acceptReading(0, front, front > 0 ? front * 58 : 0);
  acceptReading(1, right, right > 0 ? right * 58 : 0);
  serviceNavigation(); serviceMotor();
}
void begin() {
  stopRobot(); nowMs += 1000;
  settings.power=255; settings.minimum=255; settings.wallCm=15;
  startNavigation(WALL);
}
int steering() { return motorLeft - motorRight; }
void noAlignmentVeto() {
  begin(); tick(30,15); tick(28,18); tick(65,22); tick(65,22); tick(100,30);
  assert(motorLeft>motorRight); // A clear front must not veto right-wall correction.
}
void proportionalObstacleResponse() {
  begin(); tick(80,15); assert(motorLeft==255 && motorRight==255);
  tick(45,15); int gentle=motorRight-motorLeft;
  tick(30,15); assert(motorRight-motorLeft>gentle && gentle>0);
  assert(motorLeft>0 && motorRight==255);
  tick(100,15); assert(motorLeft==motorRight); // No latched turn after obstacle clears.
}
void pdAndSampleTiming() {
  begin(); tick(80,17); int p=steering(); tick(80,19); assert(steering()>2*p);
  int correction=steering(); idle(10); idle(10); assert(steering()==correction);
  nowMs+=20; acceptReading(0,80,80*58); serviceNavigation(); serviceMotor();
  assert(steering()==correction);
  begin(); tick(80,23); tick(80,19); assert(steering()<2*p);
}
void recoveryAndStop() {
  begin(); tick(8,-1); assert(motorLeft<0 && motorRight<0);
  tick(18,-1,260); idle(250); tick(18,-1,1); assert(motorLeft<0 && motorRight>0);
  begin(); tick(18,15); tick(18,15,2600); assert(mode==MANUAL && motorLeft==0);
  begin(); tick(80,150); assert(motorLeft>motorRight && motorRight>0);
  tick(80,150,4100); assert(mode==MANUAL && motorLeft==0);
  begin(); tick(30,15); stopRobot(); tick(30,15); assert(motorLeft==0 && motorRight==0);
}
void staleDataStops() {
  begin(); tick(80,15); tick(-1,-1); assert(motorLeft>0);
  idle(170); assert(motorLeft==0 && motorRight==0);
  tick(80,16); int resumed=steering(); begin(); tick(80,16); assert(steering()==resumed);
  begin(); tick(-1,-1); assert(motorLeft==0);
}
void noReturnIsNotAFalseDistance() {
  begin(); nowMs+=140; acceptReading(0,-1,0); samples[0].status=RANGE_NO_RETURN;
  acceptReading(1,15,15*58); serviceNavigation(); serviceMotor();
  assert(sensorDistanceCm(0)==-1 && motorLeft==255 && motorRight==255);
  idle(310); assert(motorLeft==0 && motorRight==0); // Stale no-return is not permission to continue.
  begin(); nowMs+=140; acceptReading(0,-1,0); samples[0].status=RANGE_NO_RETURN;
  acceptReading(1,-1,0); samples[1].status=RANGE_NO_RETURN;
  serviceNavigation(); serviceMotor(); assert(motorLeft==0 && motorRight==0);
  begin(); tick(-1,15); assert(motorLeft==0); // Malformed readings are still faults.
}
int main() {
  setupMotor(); noAlignmentVeto(); proportionalObstacleResponse(); pdAndSampleTiming();
  recoveryAndStop(); staleDataStops(); noReturnIsNotAFalseDistance();
  std::cout << "PASS wall: continuous front avoidance, immediate PD recovery, sample timing, expiry, emergency stop\n";
}
