#include "../Robot/LineWheelControl.h"
#include <cassert>
#include <cstdio>
// A target is physical wheel speed, not a ratio normalized to full PWM.
int main() {
  line::WheelControl c;
  line::Input i;
  float vl = 0, vr = 0, sumL = 0, sumR = 0;
  int samples = 0;
  for (int n = 0; n < 1000; ++n) {
    i.now += 10;
    i.pose.left += vl * .01f;
    i.pose.right += vr * .01f;
    const auto p = c.step({180, 180}, i, 255);
    i.appliedLeft = p.left;
    i.appliedRight = p.right;
    auto motor = [](int pwm, float v, float gain) {
      float target = abs(pwm) >= (fabsf(v) < 1 ? 220 : 200)
                         ? (abs(pwm) - 180) * 4.f * gain
                         : 0;
      if (pwm < 0)
        target = -target;
      return v + wall::limit(target - v, -9, 18);
    };
    vl = motor(p.left, vl, .9f);
    vr = motor(p.right, vr, 1.1f);
    if (n > 700) {
      sumL += vl;
      sumR += vr;
      ++samples;
    }
  }
  printf("mean speeds %.1f %.1f\n", sumL / samples, sumR / samples);
  assert(fabsf(sumL / samples - 180) < 15 && fabsf(sumR / samples - 180) < 15);
  auto p = c.step({}, i, 255);
  assert(p.left == 0 && p.right == 0);
}
