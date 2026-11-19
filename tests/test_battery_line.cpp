#define ROBOT_LINE_ONLY 1
#include "firmware_fixture.h"
int main() {
  setup();
  tape(5);
  assert(mode == MANUAL && !controlSession);
  command("HELLO");
  assert(batteryLinePending && !controlSession);
  nowMs += 1499;
  serviceBatteryLine();
  assert(mode == MANUAL);
  nowMs++;
  serviceBatteryLine();
  serviceNavigation();
  serviceMotor();
  assert(mode == LINE && standaloneActive && motorLeft > 0 && motorRight > 0 &&
         !automaticSetupActive());
  assert(lineFollower.config.allowOffTapeStart && lineTapeLevel == 0);
  command("HELLO");
  assert(mode == LINE && standaloneActive && !controlSession);
  command("MODE LINE");
  assert(mode == LINE && standaloneActive && !controlSession);
  // No heartbeat, laptop or sonar is needed. Actual progress keeps the stall
  // guard satisfied.
  for (int n = 0; n < 30; ++n) {
    nowMs += 100;
    leftEncoderChanged();
    rightEncoderChanged();
    serviceOdometry();
    serviceControlTimeout();
    serviceNavigation();
    serviceMotor();
  }
  assert(mode == LINE && motorLeft > 0 && motorRight > 0);
  command("STOP");
  nowMs += 10000;
  serviceBatteryLine();
  serviceNavigation();
  serviceMotor();
  assert(mode == MANUAL && !motorDemandActive());
  // A new power-up can begin searching off tape without requiring a laptop.
  setup();
  tape(7);
  nowMs += 1500;
  serviceBatteryLine();
  serviceNavigation();
  serviceMotor();
  assert(mode == LINE && lineFollower.state == line::Follower::SEARCH);
  stopRobot();
}
