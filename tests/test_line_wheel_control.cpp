#include "../Robot/LineWheelControl.h"
#include <cassert>
int main() {
  line::WheelControl c;
  line::Input i;
  auto p = c.step({1, 1}, i, 255);
  assert(p.left == 240 && p.right == 240);
  i.pose.left = 20;
  i.pose.right = 10;
  i.now = 100;
  p = c.step({1, 1}, i, 255);
  assert(p.left < p.right && p.left >= 230 && p.right <= 255);
  p = c.step({0, 1}, i, 255);
  assert(p.left == 0 && p.right >= 230);
  p = c.step({-1, 1}, i, 255);
  assert(p.left <= -230 && p.right >= 230);
  p = c.step({}, i, 255);
  assert(p.left == 0 && p.right == 0);
  p = c.step({1, 1}, i, 180);
  assert(p.left == 180 && p.right == 180 && c.saturated);
  c.reset();
  p = c.step({1, 1}, i, 255);
  assert(p.left == 240 && p.right == 240);
}
