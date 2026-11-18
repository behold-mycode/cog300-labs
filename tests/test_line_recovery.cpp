#include "../Robot/LineFollower.h"
#include <cassert>
#include <cstring>
static void corner(line::Follower &f, line::Input &i) {
  i.tape = 2;
  assert(f.begin(i));
  i.tape = 6;
  f.step(i); // First cue left.
  i.tape = 3;
  f.step(i); // Overlapping tape later appears right.
  i.tape = 0;
  i.pose.x = i.pose.path = 100;
  f.step(i);
  assert(f.state == line::Follower::SEARCH);
}
int main() {
  line::Follower f;
  line::Input i;
  corner(f, i);
  i.pose.heading = 3;
  f.step(i); // First, leftward sweep exhausted.
  i.pose.heading = 0;
  f.step(i); // Realign and advance.
  i.pose.x = i.pose.path = 160;
  auto d = f.step(i);
  assert(d.left > 0 &&
         d.right < 0); // Next location must sweep right, not left again.

  f = line::Follower();
  i = line::Input();
  corner(f, i);
  i.now = 10;
  i.tape = 4;
  i.pose.heading = 1.2f;
  f.step(i); // Fleeting leading-sensor contact.
  i.now = 100;
  i.tape = 0;
  i.pose.path = 160;
  f.step(i); // No tape during coast/exit.
  i.now = 410;
  i.tape = 2;
  f.step(i); // One isolated centre sample is not confirmation.
  i.now = 415;
  i.tape = 0;
  i.pose.path = 190;
  f.step(i);
  assert(f.state ==
         line::Follower::SEARCH); // Failed confirmation; budget not renewed.
  // Battery mode keeps translating between sweeps instead of ending the run.
  f = line::Follower();
  f.config.continuousSearch = true;
  i = line::Input();
  assert(f.begin(i));
  int advances = 0;
  bool advancing = false;
  for (int n = 0; n < 2000; ++n) {
    auto move = f.step(i);
    i.now += 20;
    float dl = move.left * 4, dr = move.right * 4, ds = (dl + dr) * .5f;
    i.pose.heading += (dr - dl) / 130;
    i.pose.x += ds * cosf(i.pose.heading);
    i.pose.y += ds * sinf(i.pose.heading);
    i.pose.path += fabsf(ds);
    i.pose.left += dl;
    i.pose.right += dr;
    bool next = !strcmp(f.reason, "LINE_SEARCH_ADVANCE");
    if (next && !advancing)
      ++advances;
    advancing = next;
    assert(f.state != line::Follower::STOPPED);
  }
  assert(advances >= 3 && i.pose.path > 150);
  // Cold-start acquisition must not discard centre tape during a forward
  // search.
  f = line::Follower();
  f.config.continuousSearch = true;
  i = line::Input();
  assert(f.begin(i));
  i.pose.heading = 3;
  f.step(i);
  i.pose.heading = 0;
  f.step(i);
  i.tape = 2;
  i.pose.x = i.pose.path = 10;
  auto acquired = f.step(i);
  assert(f.state == line::Follower::FOLLOW && acquired.left > 0 &&
         acquired.right > 0);
}
