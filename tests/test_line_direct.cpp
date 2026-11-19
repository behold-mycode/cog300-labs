#include "../Robot/LineFollower.h"
#include <cassert>

// Catches stale slow/turn states overriding a fresh centre reading, withholding
// steering while wheels coast, and pose/history rejecting current tape.
int main() {
  line::Follower f;
  f.config.cruiseSpeed=250; f.config.correctionSpeed=250;
  f.config.captureSpeed=125; f.config.turnSpeed=250;
  line::Input i; i.tape=2; assert(f.begin(i));
  auto d=f.step(i); assert(d.left==250 && d.right==250);
  i.tape=6; d=f.step(i); assert(d.left==125 && d.right==250);
  i.tape=2; d=f.step(i); assert(d.left==250 && d.right==250);
  i.tape=4; i.leftSpeed=i.rightSpeed=400;
  d=f.step(i); assert(d.left==0 && d.right==250);
  i.tape=0; d=f.step(i); assert(d.left==0 && d.right==250);
  i.tape=1; d=f.step(i); assert(d.left==250 && d.right==0);
  i.tape=0; i.now=60000; i.pose.path=10000; i.pose.heading=100;
  d=f.step(i); assert(d.left==250 && d.right==0);
  i.tape=3; d=f.step(i); assert(d.left==250 && d.right==125);
  i.tape=2; d=f.step(i); assert(d.left==250 && d.right==250);
  i.tape=5; d=f.step(i); assert(d.left==250 && d.right==250);
  i.tape=7; d=f.step(i); assert(d.left==250 && d.right==250);
  f.stop(); d=f.step(i); assert(d.left==0 && d.right==0);
  i.tape=2; assert(f.begin(i)); i.tape=0;
  d=f.step(i); assert(d.left==250 && d.right==250); // New run forgets old side.
}
