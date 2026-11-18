#include "../Robot/LineFollower.h"
#include <cassert>
#include <cstdio>
#include <cstring>
// Breaks caught: approach ignores restored tape; old side beats new evidence;
// fixed exit interval masks an opposite-side correction.
int main(int argc, char **argv) {
  assert(argc == 2);
  line::Follower f;
  line::Input i;
  i.tape = 2;
  assert(f.begin(i));
  if (!strcmp(argv[1], "gap")) {
    i.tape = 0;
    i.pose.x = i.pose.path = 30;
    f.step(i);
    i.tape = 2;
    i.pose.x = i.pose.path = 40;
    i.now = 50;
    f.step(i);
    i.pose.x = i.pose.path = 100;
    i.now = 300;
    auto d = f.step(i);
    assert(f.state == line::Follower::FOLLOW && d.left > 0 && d.right > 0);
  } else if (!strcmp(argv[1], "side")) {
    i.tape = 4;
    f.step(i);
    i.pose.x = i.pose.path = 60;
    i.tape = 2;
    f.step(i);
    i.pose.x = i.pose.path = 100;
    i.tape = 1;
    f.step(i);
    i.pose.x = i.pose.path = 210;
    i.tape = 0;
    auto d = f.step(i);
    assert(d.left > d.right);
  } else if (!strcmp(argv[1], "coasting")) {
    i.tape = 1;
    i.leftSpeed = 180;
    i.rightSpeed = 250;
    auto d = f.step(i);
    assert(d.right == 0 && d.left > i.rightSpeed);
  } else if (!strcmp(argv[1], "confirmation")) {
    i.tape = 6;
    f.step(i);
    i.tape = 0;
    i.pose.x = i.pose.path = 100;
    f.step(i);
    i.now = 10;
    i.tape = 4;
    i.pose.heading = 1.2f;
    f.step(i);
    for (int n = 0; n < 110; ++n) {
      i.now += 50;
      i.pose.path += 10;
      i.pose.x += 10 * cosf(1.2f);
      i.pose.y += 10 * sinf(1.2f);
      i.tape = n % 2 ? 4 : 2;
      f.step(i);
      assert(f.state == line::Follower::FOLLOW);
    }
  } else if (!strcmp(argv[1], "alignment")) {
    f = line::Follower();
    i = line::Input();
    f.config.continuousSearch = true;
    f.begin(i);
    i.pose.heading = 3.f;
    auto d = f.step(i);
    assert(d.left > d.right);
    i.pose.heading = 3.2f;
    d = f.step(i);
    assert(d.left > d.right); // Passing pi is not passing the forward target.
  } else {
    i.tape = 6;
    f.step(i);
    i.tape = 0;
    i.pose.x = i.pose.path = 100;
    f.step(i);
    i.now = 10;
    i.tape = 4;
    i.pose.heading = 1.2f;
    f.step(i);
    i.now = 200;
    i.tape = 1;
    auto d = f.step(i);
    assert(d.left > d.right);
  }
  puts("PASS feedback regression");
}
