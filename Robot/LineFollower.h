#pragma once
#include "Odometry.h"

namespace line {
struct Input {
  uint32_t now = 0;
  maze::Pose pose;
  uint8_t tape =
      0; // normalized L/C/R: bit set means tape, independent of polarity
  float leftSpeed = 0, rightSpeed = 0;
  int appliedLeft = 0, appliedRight = 0;
};
// Physical wheel velocities, mm/s. Zero means coast, never active braking.
struct Command {
  float left = 0, right = 0;
};
struct Config {
  bool allowOffTapeStart = false;
  float cruiseSpeed = 250, turnSpeed = 250, correctionSpeed = 250,
        captureSpeed = 125;
};
class Follower {
public:
  // Keep SEARCH=3 for the existing telemetry wire format.
  enum State { STOPPED=0, FOLLOW=1, SEARCH=3 } state = STOPPED;
  Config config;
  const char *reason = "LINE_STOPPED";
  void stop() { state=STOPPED; lastSide=0; reason="LINE_STOPPED"; }
  bool begin(const Input &i) {
    stop();
    if (!i.tape && !config.allowOffTapeStart) {
      reason="LINE_START_ON_TAPE";
      return false;
    }
    state=FOLLOW;
    step(i);
    return true;
  }
  Command step(const Input &i) {
    if (state==STOPPED) return {};
    state=FOLLOW;
    switch (i.tape) {
      case 6: // Left and centre: gentle left arc.
        lastSide=1; reason="LINE_LEFT";
        return {fminf(config.captureSpeed,config.correctionSpeed),config.correctionSpeed};
      case 3: // Centre and right: gentle right arc.
        lastSide=-1; reason="LINE_RIGHT";
        return {config.correctionSpeed,fminf(config.captureSpeed,config.correctionSpeed)};
      case 4:
        lastSide=1; reason="LINE_LEFT";
        return {0,config.correctionSpeed};
      case 1:
        lastSide=-1; reason="LINE_RIGHT";
        return {config.correctionSpeed,0};
      case 0:
        state=SEARCH; reason="LINE_LAST_DIRECTION";
        if (lastSide>0) return {0,config.turnSpeed};
        if (lastSide<0) return {config.turnSpeed,0};
        return {config.cruiseSpeed,config.cruiseSpeed};
      default: // Centre, both outer sensors, or all three: straight.
        reason="LINE_FOLLOW";
        return {config.cruiseSpeed,config.cruiseSpeed};
    }
  }
private:
  int lastSide=0;
};
} // namespace line
