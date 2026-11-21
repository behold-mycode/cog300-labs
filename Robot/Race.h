#pragma once
#include "WallNavigator.h"
namespace race {
struct Config {
  float speed = 0, sensorOffset = 0, searchAngle = 3.f;
  bool valid() const {
    return isfinite(speed) && isfinite(sensorOffset) && isfinite(searchAngle) &&
           speed > 0 && speed <= 300 && sensorOffset > 0 &&
           sensorOffset < 1000 && searchAngle >= .5f && searchAngle <= 3.1f;
  }
};
class Button {
  bool armed = false, pressed = false, inhibit = false, pending = false;
  uint32_t down = 0, up = 0;

public:
  enum Event { NONE, START, STOP };
  Event update(bool low, uint32_t now, bool moving) {
    if (low) {
      if (!pressed) {
        pressed = true;
        down = now;
        pending = false;
      }
      if (moving && !inhibit) {
        inhibit = true;
        return STOP;
      }
      return NONE;
    }
    if (pressed) {
      pressed = false;
      up = now;
      pending = armed && !inhibit && (uint32_t)(now - down) >= 800;
    }
    if ((uint32_t)(now - up) >= 40) {
      armed = true;
      inhibit = false;
      if (pending) {
        pending = false;
        return START;
      }
    }
    return NONE;
  }
};
class Navigator {
public:
  enum State { IDLE = 0, TRACK = 1, WALL = 8 } state = IDLE;
  const char *reason = "RACE_IDLE";
  bool wallStage() const { return state == WALL; }
  void stop() { state = IDLE; }
  void lineStage() {
    state = TRACK;
    entrySamples = 0;
    entrySequence = 0;
  }
  void enterWall() { state = WALL; }
  bool corridor(const wall::Config &c, const wall::Input &i, uint8_t tape) {
    const auto &left = i.ranges[2];
    const auto &right = i.ranges[1];
    float width = left.mm + right.mm + 2 * c.sideY;
    if (tape || !wall::fresh(left, i.now) || !wall::fresh(right, i.now) ||
        left.mm >= wall::SIDE_RANGE_MM || right.mm >= wall::SIDE_RANGE_MM ||
        width <= 2 * (c.halfWidth + c.margin) + 20) {
      entrySamples = 0;
      return false;
    }
    if (entrySequence != left.sequence) {
      entrySequence = left.sequence;
      ++entrySamples;
    }
    return entrySamples >= 3;
  }

private:
  unsigned entrySamples = 0;
  uint32_t entrySequence = 0;

public:
  wall::Speeds guarded(const wall::Config &c, const wall::Input &i,
                       wall::Speeds d) {

    bool curve =
        fabsf(d.left - d.right) > 1 || fabsf(i.leftSpeed - i.rightSpeed) > 1;
    float v = fmaxf(fabsf(i.leftSpeed), fabsf(i.rightSpeed));
    float momentum = v * c.reaction + v * v / (2 * c.deceleration);
    float front = wall::corrected(c, i, 0);
    if (front < 0) {
      reason = "RACE_FRONT_UNKNOWN";
      return {};
    }
    bool pivot = d.left * d.right < 0;
    float room =
        front + c.frontSensor - (pivot ? c.radius() : c.front) - c.margin;
    for (int n = 1; n < 3; ++n) {
      float side = wall::corrected(c, i, n);
      float sideRoom =
          side + c.sideY - (curve ? c.radius() : c.halfWidth) - c.margin;
      if (side < 0 || sideRoom <= 0 || (curve && sideRoom <= momentum)) {
        reason = "RACE_SIDE_CLEARANCE_REQUIRED";
        return {};
      }
      if (curve)
        room = fminf(room, sideRoom);
    }
    if (room <= momentum) {
      reason = "RACE_BRAKING";
      return {};
    }
    // Solve v*reaction + v²/(2*deceleration) <= available clearance.
    // Scale both wheels together so slowing does not change the requested arc.
    float ar = c.deceleration * c.reaction;
    float cap = 2 * c.deceleration * room /
                (sqrtf(ar * ar + 2 * c.deceleration * room) + ar);
    float requested = fmaxf(fabsf(d.left), fabsf(d.right));
    if (requested > cap) {
      float scale = cap / requested;
      d.left *= scale;
      d.right *= scale;
      reason = "RACE_SLOWING_FOR_CLEARANCE";
    }
    return d;
  }
};
} // namespace race
