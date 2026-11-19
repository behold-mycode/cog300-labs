#include "../Robot/LineFollower.h"
#include <cassert>
#include <initializer_list>
// Lost-line continuation has no coordinate/angle/time veto. Each new reading
// immediately steers, and stopping/restarting clears the old direction.
int main() {
 for(int side:{1,-1}) {
  line::Follower f; line::Input i; i.tape=2; assert(f.begin(i));
  i.tape=side>0?4:1; f.step(i);
  i.tape=0; i.now=60000; i.pose.path=10000; i.pose.heading=100;
  auto d=f.step(i); assert((d.right-d.left)*side>0);
  assert(d.left>=0 && d.right>=0);
  i.tape=side>0?1:4; d=f.step(i); assert((d.right-d.left)*side<0);
  i.tape=2; d=f.step(i); assert(d.left==d.right && d.left>0);
  f.stop(); d=f.step(i); assert(d.left==0 && d.right==0);
  i.tape=0; assert(!f.begin(i));
  f.config.allowOffTapeStart=true; assert(f.begin(i));
  d=f.step(i); assert(d.left==d.right && d.left>0);
 }
}
