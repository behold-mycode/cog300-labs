#pragma once
#include "LineFollower.h"
#include "Velocity.h"

namespace line {
class WheelControl {
  uint32_t lastEdge=0;
  uint32_t lastStep = 0;
  bool stepping = false;
  float integral[2] = {}, lastTarget[2] = {};
  uint32_t started[2] = {};
  wall::Powers output;

public:
  int cruise = 240, minimum = 0;
  float referenceSpeed = 200, feedForwardOffset = 160, kp = .12f, ki = .8f;
  int startPower = 220;
  bool saturated = false;
  void reset() {
    lastEdge=0;
    stepping = false;
    output = {};
    saturated = false;
    for (int n = 0; n < 2; ++n)
      integral[n] = lastTarget[n] = 0;
  }
  wall::Powers step(Command want, Input i, int maximum) {
    if (!want.left && !want.right) {
      integral[0] = integral[1] = lastTarget[0] = lastTarget[1] = 0;
      output = {};
      saturated = false;
      lastStep = i.now;
      stepping = true;
      return output;
    }
    // Update on a new target or encoder edge. With neither, refresh the
    // no-edge speed estimate and integral every 10 ms.
    const uint32_t elapsed = stepping ? (uint32_t)(i.now - lastStep) : 10;
    if (stepping && elapsed < 10 && lastEdge==i.edgeSequence && want.left == lastTarget[0] &&
        want.right == lastTarget[1])
      return output;
    const float dt = fminf(elapsed * .001f, .05f);
    lastStep = i.now;
    lastEdge=i.edgeSequence;
    stepping = true;
    saturated = false;
    output = {wheel(0, want.left, i.leftSpeed, i.appliedLeft,
                    fabsf(want.left) >= fabsf(want.right), dt, i.now, maximum),
              wheel(1, want.right, i.rightSpeed, i.appliedRight,
                    fabsf(want.right) >= fabsf(want.left), dt, i.now, maximum)};
    return output;
  }

private:
  int wheel(int n, float want, float measured, int applied,
            bool allowStartBoost, float dt, uint32_t now, int maximum) {
    if (!want) {
      integral[n] = lastTarget[n] = 0;
      return 0;
    }
    const int sign = want > 0 ? 1 : -1;
    if (!lastTarget[n] || want * lastTarget[n] < 0) {
      integral[n] = 0;
      started[n] = now;
    }
    lastTarget[n] = want;
    if (applied * sign <= 0)
      started[n] = now;
    const float speed = fabsf(want), error = speed - measured * sign;
    const float ff = feedForwardOffset +
                     (cruise - feedForwardOffset) * speed / referenceSpeed;
    float raw = ff + kp * error + integral[n];
    // Never accumulate a reversal delay as a speed deficit.
    const int previousRequest = n == 0 ? output.left : output.right;
    const bool executing = applied * sign > 0 ||
                           (previousRequest == 0 && applied == 0 && error > 0);
    if (executing && (raw < maximum || error < 0) &&
        (raw > minimum || error > 0))
      integral[n] = wall::limit(integral[n] + ki * error * dt, -100, 100);
    raw = ff + kp * error + integral[n];
    // A startup boost on the slower wheel would straighten a tight arc.
    if (allowStartBoost && (uint32_t)(now - started[n]) < 120 &&
        measured * sign < 30)
      raw = fmaxf(raw, startPower);
    const int low = minimum < maximum ? minimum : maximum;
    saturated |= raw < low || raw > maximum;
    return sign * (int)wall::limit(raw, low, maximum);
  }
};
} // namespace line
