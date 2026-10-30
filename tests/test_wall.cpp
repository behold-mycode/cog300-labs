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
void earlyContinuousCorner() {
  begin(); tick(80,15); tick(30,15);
  assert(motorLeft > 0 && motorLeft < motorRight && motorRight == 255);
  assert(!strcmp(navigationState,"TURNING_LEFT"));
  // No reverse/settle pause on a normal corner, even with a missing side echo.
  tick(28,-1); assert(motorLeft>0 && motorRight==255);
  tick(100,-1); assert(!strcmp(navigationState,"TURNING_LEFT"));
  // Re-reading one clearance echo must not count as a second confirmation.
  idle(10); idle(10); assert(!strcmp(navigationState,"TURNING_LEFT"));
  tick(120,-1); assert(strcmp(navigationState,"TURNING_LEFT"));
}
void pdTracksNewSamples() {
  begin(); tick(80,17); int proportional = steering();
  tick(80,19); assert(steering() > 2*proportional); // Drifting away: D adds correction.
  int correction=steering(); idle(10); idle(10); assert(steering()==correction);
  begin(); tick(80,23); tick(80,19);
  assert(steering() < 2*proportional); // Already converging: D reduces correction.
  begin(); tick(80,15); assert(motorLeft==255 && motorRight==255);
  begin(); tick(80,35); assert(steering()>127); // Allow tighter arcs than old half-power cap.
}
void boundedDropoutsAndReset() {
  begin(); tick(80,15); tick(-1,-1);
  assert(motorLeft==255 && motorRight==255); // One missed echo does not cause a stop/start.
  idle(170); assert(motorLeft==0 && motorRight==0); // Grace ends at 300 ms from good echo.
  tick(80,16); int recovered=steering();
  begin(); tick(80,16); assert(steering()==recovered); // No derivative kick after sensor loss.
  begin(); tick(-1,-1); assert(motorLeft==0 && motorRight==0); // Never assume no echo means clear.
}
void independentSensorUpdates() {
  begin(); tick(80,17); int correction = steering();
  nowMs += 70; acceptReading(0,75,75*58); serviceNavigation(); serviceMotor();
  assert(steering()==correction); // Front-only update cannot alter the side derivative.
  nowMs += 70; acceptReading(1,19,19*58); serviceNavigation(); serviceMotor();
  assert(steering()>2*correction);
  begin(); tick(30,15); tick(30,15); tick(100,15);
  assert(!strcmp(navigationState,"TURNING_LEFT"));
  tick(25,15); // One clear outlier followed by blocked data must not finish the turn.
  assert(!strcmp(navigationState,"TURNING_LEFT"));
  tick(100,15); assert(!strcmp(navigationState,"TURNING_LEFT"));
  tick(100,15); assert(!strcmp(navigationState,"ALIGNING_WALL"));
  assert(motorLeft>0 && motorRight>0);
}
void earlierCornerExit() {
  begin(); tick(30,15); tick(28,18); tick(65,22);
  assert(!strcmp(navigationState,"TURNING_LEFT")); // Still require two real echoes.
  tick(65,22);
  assert(!strcmp(navigationState,"ALIGNING_WALL")); // Previously waited for 88 cm.
  assert(motorLeft>0 && motorRight>0);
}
void recoveryAndTimeouts() {
  begin(); tick(8,-1); assert(motorLeft<0 && motorRight<0);
  tick(18,-1,260); idle(250); tick(18,-1,1);
  assert(motorLeft<0 && motorRight>0); // Emergency-only backup followed by spin.
  begin(); tick(30,15);
  for(int i=0;i<25 && mode==WALL;++i) tick(25,15);
  assert(mode==MANUAL && motorLeft==0 && motorRight==0);
  begin(); tick(80,150); assert(motorLeft>motorRight && motorRight>0);
  for(int i=0;i<40 && mode==WALL;++i) tick(80,150);
  assert(mode==MANUAL && motorLeft==0);
  begin(); tick(30,15); stopRobot(); tick(30,15);
  assert(mode==MANUAL && motorLeft==0 && motorRight==0);
}
void rollover() {
  begin(); nowMs=UINT32_MAX-200; startNavigation(WALL); tick(80,17);
  tick(80,19); assert(steering()>0 && motorLeft==255);
}
int main() {
  setupMotor();
  earlyContinuousCorner(); pdTracksNewSamples(); boundedDropoutsAndReset();
  independentSensorUpdates(); earlierCornerExit(); recoveryAndTimeouts(); rollover();
  std::cout << "PASS wall: continuous early corners, fresh-sample PD, tight steering, bounded dropouts, recovery, stop, rollover\n";
}
