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
  bool continuousSearch = false;
  float sensorOffset = 150, outerOffset = 145, halfSpacing = 50, tapeWidth = 50,
        track = 130;
  float cruiseSpeed = 200, turnSpeed = 90, correctionSpeed = 180,
        captureSpeed = 130;
  float gapDistance = 25, approachTolerance = 55, searchAdvance = 50,
        searchAngle = 2.8f;
  float confirmDistance = 50, recoveryDistance = 900;
  uint32_t recoveryMs = 20000;
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
    reason = "LINE_STOPPED";
  }
  bool begin(const Input &i) {
    stop();
    if (!i.tape && !config.continuousSearch) {
      reason = "LINE_START_ON_TAPE";
      return false;
    }
    knownTape = i.tape != 0;
    episode = false;
    confirmStart = -1;
    cueSide = 0;

    lastSide = (i.tape == 1 || i.tape == 3) ? -1 : 1;
    incoming = i.pose.heading;
    rememberCentre(i);
    lastGoodPath = i.pose.path;
    previous = {config.cruiseSpeed, config.cruiseSpeed};
    searchLocation = 0;
    state = FOLLOW;
    reason = "LINE_FOLLOW";
    if (!knownTape) {
      beginEpisode(i);
      beginSweep(i, lastSide);
    }
    return true;
  }
  Command step(const Input &i) {
    if (state == STOPPED)
      return {};
    if (episode && !config.continuousSearch &&
        (i.pose.path - episodePath > config.recoveryDistance ||
         (uint32_t)(i.now - episodeTime) > config.recoveryMs)) {
      stop();
      reason = "LINE_NOT_FOUND";
      return {};
    }
    if (state == FOLLOW)
      return follow(i);
    if (state == CORNER) {
      // A gap is not a mandatory corner. A restored, forward-facing centre
      // continuation cancels approach before another motor command is issued.
      if (i.tape == 2 && maze::angleDistance(i.pose.heading, incoming) < .45f)
        return acquire(i);
      return approach(i);
    }
    // Only incoming-tape rejection constrains acquisition; there is no blind
    // turn-exit timer. Reacquisition also works during alignment/translation.
    const bool ahead =
        maze::angleDistance(i.pose.heading, incoming + maze::PI_F) > .35f;
    if ((!knownTape && (i.tape & 2)) ||
        (knownTape && ahead && newTape(i) &&
         (searchStep != 0 || (i.pose.heading - sweepStart) * sweepSide > .15f)))
      return acquire(i);
    if (searchStep == 0) {
      remaining =
          config.searchAngle - (i.pose.heading - sweepStart) * sweepSide;
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
      if (!alignSign) {
        const float error = maze::wrap(incoming - i.pose.heading);
        alignTarget = i.pose.heading + error;
        alignSign = error > 0 ? 1 : -1;
      }
      const float error = alignTarget - i.pose.heading;
      if (fabsf(error) > .12f && error * alignSign > 0) {
        targetHeading = alignTarget;
        reason = "LINE_SEARCH_ALIGN_FORWARD";
        return turn(alignSign);
      }
      searchStep = 2;
      advanceStart = i.pose.path;
    }
    remaining = config.searchAdvance - (i.pose.path - advanceStart);
    if (remaining > 0) {
      reason = "LINE_SEARCH_ADVANCE";
      return forward(i, incoming);
    }
    ++searchLocation;
    beginSweep(i, -sweepSide);
    reason = "LINE_SEARCH_NEW_POSITION";
    return turn(sweepSide);
  }

private:
  float incoming = 0, pointX = 0, pointY = 0, lastGoodPath = 0, episodePath = 0,
        confirmStart = -1;
  float cornerStart = 0, sweepStart = 0, advanceStart = 0, alignTarget = 0,
        confirmHeading = 0;
  uint32_t episodeTime = 0;
  bool episode = false, knownTape = false;
  int searchStep = 0, sweepSide = 1, alignSign = 0, cueSide = 0;
  float cueX = 0, cueY = 0, cueHeading = 0, cuePath = 0;
  Command previous;
  void rememberCentre(const Input &i) {
    pointX = i.pose.x + config.sensorOffset * cosf(i.pose.heading);
    pointY = i.pose.y + config.sensorOffset * sinf(i.pose.heading);
  }
  void beginEpisode(const Input &i) {
    episode = true;
    episodePath = i.pose.path;
    episodeTime = i.now;
    searchLocation = 0;
    confirmStart = -1;
  }
  void beginSweep(const Input &i, int side) {
    state = SEARCH;
    searchStep = 0;
    sweepSide = side;
    sweepStart = i.pose.heading;
    targetHeading = sweepStart + sweepSide * config.searchAngle;
    reason = "LINE_CORNER_TURN";
  }
  Command acquire(const Input &i) {
    knownTape = true;
    state = FOLLOW;
    lastGoodPath = i.pose.path;
    confirmStart = -1;
    cueSide = 0;

    return follow(i);
  }
  Command follow(const Input &i) {
    // An opposite outer hit may be the edge of the incoming strip, not a
    // new bend. Compare its world position with the line at the first cue.
    if (cueSide &&
        i.pose.path - cuePath > config.sensorOffset + config.tapeWidth)
      cueSide = 0;
    if (confirmStart >= 0 &&
        maze::angleDistance(i.pose.heading, confirmHeading) > .45f)
      confirmStart = -1;

    const int side = (i.tape == 4 || i.tape == 6)   ? 1
                     : (i.tape == 1 || i.tape == 3) ? -1
                                                    : 0;
    if (!episode && side) {
      if (!cueSide) {
        cueSide = side;
        cueX = pointX;
        cueY = pointY;
        cueHeading = incoming;
        cuePath = i.pose.path;
      } else if (cueSide && side != cueSide &&
                 i.pose.path - cuePath > config.tapeWidth * .2f) {
        const float sx = i.pose.x + config.outerOffset * cosf(i.pose.heading) -
                         side * config.halfSpacing * sinf(i.pose.heading);
        const float sy = i.pose.y + config.outerOffset * sinf(i.pose.heading) +
                         side * config.halfSpacing * cosf(i.pose.heading);
        const float across =
            -(sx - cueX) * sinf(cueHeading) + (sy - cueY) * cosf(cueHeading);
        if (across * side >= config.tapeWidth) {
          cueSide = side;
          cuePath = i.pose.path;
        }
      }
    }
    if (i.tape == 4 || i.tape == 6)
      lastSide = 1;
    if (i.tape == 1 || i.tape == 3)
      lastSide = -1;
    if (i.tape & 2) {
      lastGoodPath = i.pose.path;
      if (confirmStart < 0) {
        confirmStart = i.pose.path;
        confirmHeading = i.pose.heading;
      }
      if (i.pose.path - confirmStart >= config.confirmDistance) {
        if (episode) {
          cueSide = 0;
          incoming = i.pose.heading;
          rememberCentre(i);
        }
        episode = false;
      }
      if (!episode && i.tape == 2) {
        incoming = i.pose.heading;
        rememberCentre(i);
      }
      reason = "LINE_FOLLOW";
      // Geometric lookahead steering: omega = v*2*y/(L^2+y^2).
      // Wheel-derived yaw damps momentum when the centre is regained.
      const float y = i.tape == 6   ? config.halfSpacing * .5f
                      : i.tape == 3 ? -config.halfSpacing * .5f
                                    : 0;
      const float v = episode ? fminf(config.cruiseSpeed, config.captureSpeed)
                              : config.cruiseSpeed;
      const float yaw = (i.rightSpeed - i.leftSpeed) / config.track;
      float omega =
          v * 2 * y / (config.sensorOffset * config.sensorOffset + y * y) -
          .25f * yaw;
      const float delta =
          fmaxf(-v * .7f, fminf(v * .7f, omega * config.track * .5f));
      previous = {v - delta, v + delta};
      return previous;
    }
    if (i.tape == 4 || i.tape == 1) {
      lastGoodPath = i.pose.path;
      // Outer-only needs stronger curvature; stopping the inner wheel avoids
      // demanding an unachievable crawl and avoids reversing during tracking.
      const float speed =
          episode ? fminf(config.correctionSpeed, config.captureSpeed)
                  : config.correctionSpeed;
      // A disabled inner wheel still coasts. Preserve the requested wheel
      // speed difference instead of letting its momentum turn the car away.
      const float inner = lastSide > 0 ? i.leftSpeed : i.rightSpeed;
      const float outer = speed + fmaxf(0, inner);
      previous = lastSide > 0 ? Command{0, outer} : Command{outer, 0};
      reason = "LINE_FORWARD_CORRECTION";
      return previous;
    }
    if (i.tape == 5) {
      lastGoodPath = i.pose.path;
      reason = "LINE_WIDE_TAPE";
      return {config.cruiseSpeed, config.cruiseSpeed};
    }
    if (i.pose.path - lastGoodPath < config.gapDistance) {
      reason = "LINE_FORWARD_GAP";
      return previous;
    }
    confirmStart = -1;
    if (!episode) {
      beginEpisode(i);
      state = CORNER;
      cornerStart = i.pose.path;
      return approach(i);
    }
    // A failed acquisition does not replenish the episode budget.
    if (!config.continuousSearch && searchLocation >= 1) {
      stop();
      reason = "LINE_NOT_FOUND";
      return {};
    }
    state = SEARCH;
    searchStep = 1;
    alignSign = 0;
    return step(i);
  }
  Command approach(const Input &i) {
    remaining = (pointX - i.pose.x) * cosf(incoming) +
                (pointY - i.pose.y) * sinf(incoming);
    if (remaining > config.approachTolerance &&
        i.pose.path - cornerStart < config.sensorOffset) {
      reason = "LINE_CORNER_APPROACH";
      return forward(i, incoming);
    }
    beginSweep(i, cueSide ? cueSide : lastSide);
    return turn(sweepSide);
  }
  bool newTape(const Input &i) const {
    for (int n = 0; n < 3; ++n) {
      if (!(i.tape & (4 >> n)) || (n == 0 && sweepSide < 0) ||
          (n == 2 && sweepSide > 0))
        continue;
      const float sx = n == 1 ? config.sensorOffset : config.outerOffset,
                  sy = (1 - n) * config.halfSpacing;
      const float dx = i.pose.x + sx * cosf(i.pose.heading) -
                       sy * sinf(i.pose.heading) - pointX;
      const float dy = i.pose.y + sx * sinf(i.pose.heading) +
                       sy * cosf(i.pose.heading) - pointY;
      const float along = dx * cosf(incoming) + dy * sinf(incoming);
      const float across = -dx * sinf(incoming) + dy * cosf(incoming);
      // Both the stored centre hit and this hit can lie at a tape edge:
      // their relative cross-track uncertainty spans a full tape width.
      if (along > config.tapeWidth || fabsf(across) > config.tapeWidth)
        return true;
    }
    return false;
  }
  Command turn(int side) const {
    return {-side * config.turnSpeed, side * config.turnSpeed};
  }
  Command forward(const Input &i, float heading) const {
    const float error = maze::wrap(heading - i.pose.heading);
    const float yaw = (i.rightSpeed - i.leftSpeed) / config.track;
    const float delta =
        fmaxf(-config.cruiseSpeed * .6f,
              fminf(config.cruiseSpeed * .6f,
                    (2 * error - .25f * yaw) * config.track * .5f));
    return {config.cruiseSpeed - delta, config.cruiseSpeed + delta};
  }
};
} // namespace line
