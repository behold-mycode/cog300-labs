#include "../Robot/LineWheelControl.h"
#include <cassert>
int main() {
  line::WheelControl c;
  line::Input i;
  auto p = c.step({200, 200}, i, 255);
  assert(p.left > 0 && p.right > 0);
  i.now = 100;
  i.pose.left = 20;
  i.pose.right = 10;
  i.appliedLeft = p.left;
  i.appliedRight = p.right;
  p = c.step({200, 200}, i, 255);
  assert(p.left < p.right); // More output to the actually slower wheel.
  p = c.step({0, 180}, i, 255);
  assert(p.left == 0 && p.right > 0);
  p = c.step({}, i, 255);
  assert(p.left == 0 && p.right == 0);
  p = c.step({200, 200}, i, 180);
  assert(p.left == 180 && p.right == 180 && c.saturated);
  // The reversal interlock must not consume the launch effort before it runs.
  c.reset();
  i = line::Input();
  for (int n = 0; n < 35; ++n) {
    i.now += 10;
    p = c.step({-90, 90}, i, 255);
  }
  assert(p.left <= -220 && p.right >= 220);
  // No integral accumulated during that unavailable actuation. Once observed
  // speed equals demand, output returns to the feed-forward operating point.
  i.appliedLeft = -220;
  i.appliedRight = 220;
  for (int n = 0; n < 30; ++n) {
    i.now += 10;
    i.pose.left -= .9f;
    i.pose.right += .9f;
    p = c.step({-90, 90}, i, 255);
  }
  assert(p.left > -220 && p.right < 220);
  // Changing forward curvature never resets the encoder estimator.
  i.now += 10;
  i.pose.left += 1;
  i.pose.right += 1;
  c.observe(i);
  assert(i.leftSpeed < 0 && i.rightSpeed > 0);
  p = c.step({120, 180}, i, 255);
  assert(p.left > 0 && p.right > 0);
  p = c.step({}, i, 255);
  assert(p.left == 0 && p.right == 0);
  // Permitted zero-offset/zero-launch tuning must recover from lower-rail
  // saturation after overspeed instead of permanently requesting zero.
  c = line::WheelControl();
  c.feedForwardOffset = 0;
  c.kp = 0;
  c.ki = 10;
  c.startPower = 0;
  i = line::Input();
  for (int k = 0; k < 500; ++k) {
    i.now += 10;
    if (k < 120) {
      i.pose.left += 3;
      i.pose.right += 3;
    }
    const float v = k < 100 ? 200 : 30;
    p = c.step({v, v}, i, 255);
    i.appliedLeft = p.left;
    i.appliedRight = p.right;
  }
  assert(p.left > 0 && p.right > 0);
}
