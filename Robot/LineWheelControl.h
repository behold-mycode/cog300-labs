#pragma once
#include "LineFollower.h"
#include "Velocity.h"
namespace line {
class WheelControl {
  Command last;
  float startL = 0, startR = 0;

public:
  int cruise = 240, minimum = 230;
  bool saturated = false;
  void reset() {
    last = {};
    saturated = false;
  }
  wall::Powers step(Command d, const Input &i, int maximum) {
    if (d.left == 0 && d.right == 0) {
      reset();
      return {};
    }
    if (d.left != last.left || d.right != last.right) {
      startL = i.pose.left;
      startR = i.pose.right;
      last = d;
    }
    float l = (i.pose.left - startL) * (d.left < 0 ? -1 : 1),
          r = (i.pose.right - startR) * (d.right < 0 ? -1 : 1);
    float error =
        d.left && d.right ? l * fabsf(d.right) - r * fabsf(d.left) : 0;
    float trim = wall::limit(error * 1.5f, -20, 20);
    auto power = [&](float ratio, float correction) {
      if (!ratio)
        return 0;
      float requested = cruise * fabsf(ratio) + correction;
      int low = minimum < maximum ? minimum : maximum;
      saturated |= requested < low || requested > maximum;
      int pwm = (int)wall::limit(requested, low, maximum);
      return ratio < 0 ? -pwm : pwm;
    };
    saturated = false;
    return {power(d.left, -trim), power(d.right, trim)};
  }
};
} // namespace line
