#pragma once
#include "Odometry.h"
namespace line {
struct Input {
  uint32_t now = 0;
  maze::Pose pose;
  uint8_t tape = 0;
};
// Ratios are dimensionless wheel travel requests, not millimetres/second.
struct Command {
  float left = 0, right = 0;
};
struct Config {
  bool continuousSearch = false;
  float sensorOffset = 150, outerOffset = 145, halfSpacing = 50, tapeWidth = 50;
  float gapDistance = 25, approachTolerance = 55, searchAdvance = 50,
        searchAngle = 2.8f;
  float confirmDistance = 50, recoveryDistance = 900;
  // End the pivot at the leading outer sensor; coast carries the centre toward
  // tape.
  uint32_t turnExitMs =
      400; // includes the 250 ms bridge reversal coast interval
};
class Follower {
public:
  enum State { STOPPED, FOLLOW, CORNER, SEARCH } state = STOPPED;
  Config config;
  const char *reason = "LINE_STOPPED";
  int lastSide = 1, searchLocation = 0;
  float targetHeading = 0, remaining = 0;
  void stop() {
    state = STOPPED;
    targetHeading = remaining = 0;
    handoffUntil = 0;
    reason = "LINE_STOPPED";
  }
  bool begin(const Input &i) {
    if (!i.tape && !config.continuousSearch) {
      stop();
      reason = "LINE_START_ON_TAPE";
      return false;
    }
    knownTape = i.tape != 0;
    state = FOLLOW;
    lastSide = (i.tape == 1 || i.tape == 3) ? -1 : 1;
    incoming = i.pose.heading;
    rememberCentre(i);
    lastGoodPath = i.pose.path;
    episode = false;
    confirmStart = -1;
    cornerSide = 0;
    cuePath = i.pose.path;
    handoffUntil = 0;
    previous = {1, 1};
    reason = "LINE_FOLLOW";
    if (!i.tape) {
      episode = true;
      episodePath = i.pose.path;
      searchLocation = 0;
      state = SEARCH;
      searchStep = 0;
      sweepSide = 1;
      sweepStart = i.pose.heading;
      reason = "LINE_SEARCH_TURN";
    }
    return true;
  }
  Command step(const Input &i) {
    if (state == STOPPED)
      return {};
    // A recovery episode cannot restart its budget from a fleeting tape hit.
    if (!config.continuousSearch && episode &&
        i.pose.path - episodePath > config.recoveryDistance) {
      stop();
      reason = "LINE_NOT_FOUND";
      return {};
    }
    if (state == FOLLOW) {
      // Latch the first side cue across broad/overlapping turn tape.
      if (i.tape == 2 &&
          i.pose.path - cuePath > config.sensorOffset + config.tapeWidth)
        cornerSide = 0;
      if (i.tape == 6 || i.tape == 4 || i.tape == 3 || i.tape == 1) {
        if (!cornerSide)
          cornerSide = (i.tape & 4) ? 1 : -1;
        cuePath = i.pose.path;
      }
      if (handoffUntil && (int32_t)(handoffUntil - i.now) > 0) {
        reason = "LINE_TURN_EXIT";
        return {1, 1};
      }
      if (handoffUntil) {
        handoffUntil = 0;
        confirmStart = -1;
        incoming = i.pose.heading;
        lastGoodPath = i.pose.path;
      }
      if (i.tape & 2) {
        lastGoodPath = i.pose.path;
        if (confirmStart < 0)
          confirmStart = i.pose.path;
        if (i.pose.path - confirmStart >= config.confirmDistance)
          episode = false;
        if (!episode && i.tape == 2) {
          incoming = i.pose.heading;
          rememberCentre(i);
        }
        if (i.tape == 6)
          lastSide = 1;
        else if (i.tape == 3)
          lastSide = -1;
        reason = "LINE_FOLLOW";
        previous = i.tape == 6   ? Command{.96f, 1}
                   : i.tape == 3 ? Command{1, .96f}
                                 : Command{1, 1};
        return previous;
      }
      confirmStart = -1;
      if (i.tape == 4 || i.tape == 1) {
        lastSide = i.tape == 4 ? 1 : -1;
        lastGoodPath = i.pose.path;
        previous = lastSide > 0 ? Command{0, 1} : Command{1, 0};
        reason = "LINE_FORWARD_CORRECTION";
        return previous;
      }
      if (i.tape == 5 || i.tape == 7) {
        lastGoodPath = i.pose.path;
        return {1, 1};
      }
      if (i.pose.path - lastGoodPath < config.gapDistance) {
        reason = "LINE_FORWARD_GAP";
        return previous;
      }
      beginCorner(i);
      return approach(i);
    }
    if (state == CORNER)
      return approach(i);
    // No incoming segment exists when booting off tape. A first centre hit
    // during translation or alignment is useful evidence, not old tape.
    if (!knownTape && (i.tape & 2)) {
      knownTape = true;
      state = FOLLOW;
      incoming = i.pose.heading;
      rememberCentre(i);
      lastGoodPath = i.pose.path;
      confirmStart = -1;
      previous = {1, 1};
      handoffUntil = searchStep == 2 ? 0 : i.now + config.turnExitMs;
      reason = "LINE_FIRST_ACQUISITION";
      return previous;
    }
    // SEARCH has one sweep at each of two spatially separated locations.
    if (searchStep == 0) {
      float angle = (i.pose.heading - sweepStart) * sweepSide;
      if (newTape(i) && angle > .25f &&
          maze::angleDistance(i.pose.heading, incoming + maze::PI_F) > .25f) {
        state = FOLLOW;
        handoffUntil = i.now + config.turnExitMs;
        confirmStart = i.pose.path;
        lastGoodPath = i.pose.path;
        previous = {1, 1};
        reason = "LINE_REACQUIRED";
        return previous;
      }
      remaining = config.searchAngle - angle;
      if (remaining > 0) {
        reason = "LINE_SEARCH_TURN";
        return turn(sweepSide);
      }
      if (!config.continuousSearch && searchLocation >= 1) {
        stop();
        reason = "LINE_NOT_FOUND";
        return {};
      }
      searchStep = 1;
      alignSign = 0;
    }
    if (searchStep == 1) {
      float error = maze::wrap(incoming - i.pose.heading);
      int sign = error > 0 ? 1 : -1;
      if (fabsf(error) > .12f && (!alignSign || sign == alignSign)) {
        alignSign = sign;
        targetHeading = incoming;
        reason = "LINE_SEARCH_ALIGN_FORWARD";
        return turn(sign);
      }
      searchStep = 2;
      advanceStart = i.pose.path;
    }
    if (searchStep == 2) {
      remaining = config.searchAdvance - (i.pose.path - advanceStart);
      if (remaining > 0) {
        reason = "LINE_SEARCH_ADVANCE";
        return forward(i, incoming);
      }
      searchLocation = 1;
      searchStep = 0;
      sweepSide = -sweepSide;
      sweepStart = i.pose.heading;
      targetHeading = sweepStart + sweepSide * config.searchAngle;
      reason = "LINE_SEARCH_NEW_POSITION";
      return turn(sweepSide);
    }
    return {};
  }

private:
  float incoming = 0, pointX = 0, pointY = 0, lastGoodPath = 0, episodePath = 0,
        confirmStart = -1;
  float cornerStart = 0, sweepStart = 0, advanceStart = 0;
  float cuePath = 0;
  int cornerSide = 0;
  bool episode = false, knownTape = false;
  int searchStep = 0, sweepSide = 1, alignSign = 0;
  Command previous;
  uint32_t handoffUntil = 0;
  void rememberCentre(const Input &i) {
    pointX = i.pose.x + config.sensorOffset * cosf(i.pose.heading);
    pointY = i.pose.y + config.sensorOffset * sinf(i.pose.heading);
  }
  void beginCorner(const Input &i) {
    if (!episode) {
      episode = true;
      episodePath = i.pose.path;
      searchLocation = 0;
    } else if (state == FOLLOW) { // Failed confirmation moves to a fresh search
                                  // position.
      if (!config.continuousSearch && searchLocation >= 1) {
        stop();
        reason = "LINE_NOT_FOUND";
        return;
      }
      state = SEARCH;
      searchStep = 1;
      alignSign = 0;
      return;
    }
    state = CORNER;
    cornerStart = i.pose.path;
    reason = "LINE_CORNER_APPROACH";
  }
  Command approach(const Input &i) {
    if (state == STOPPED)
      return {};
    if (state == SEARCH)
      return step(i);
    remaining = (pointX - i.pose.x) * cosf(incoming) +
                (pointY - i.pose.y) * sinf(incoming);
    if (remaining > config.approachTolerance &&
        i.pose.path - cornerStart < config.sensorOffset) {
      reason = "LINE_CORNER_APPROACH";
      return forward(i, incoming);
    }
    state = SEARCH;
    searchStep = 0;
    sweepSide = cornerSide ? cornerSide : lastSide;
    sweepStart = i.pose.heading;
    targetHeading = sweepStart + sweepSide * config.searchAngle;
    reason = "LINE_CORNER_TURN";
    return turn(sweepSide);
  }
  bool newTape(const Input &i) const {
    for (int n = 0; n < 3; ++n) {
      if (!(i.tape & (4 >> n)) || (n == 0 && sweepSide < 0) ||
          (n == 2 && sweepSide > 0))
        continue;
      float sx = n == 1 ? config.sensorOffset : config.outerOffset,
            sy = (1 - n) * config.halfSpacing;
      float dx = i.pose.x + sx * cosf(i.pose.heading) -
                 sy * sinf(i.pose.heading) - pointX;
      float dy = i.pose.y + sx * sinf(i.pose.heading) +
                 sy * cosf(i.pose.heading) - pointY;
      float along = dx * cosf(incoming) + dy * sinf(incoming);
      float across = -dx * sinf(incoming) + dy * cosf(incoming);
      // Reject the incoming tape behind the remembered corner, including an
      // outer sensor that reaches it before a backwards-facing centre sensor.
      if (along > config.tapeWidth || fabsf(across) > config.tapeWidth * .7f)
        return true;
    }
    return false;
  }
  static Command turn(int side) { return {float(-side), float(side)}; }
  static Command forward(const Input &i, float heading) {
    float e = maze::wrap(heading - i.pose.heading);
    if (e > .12f)
      return {.96f, 1};
    if (e < -.12f)
      return {1, .96f};
    return {1, 1};
  }
};
} // namespace line
