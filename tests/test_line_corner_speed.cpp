#include "../Robot/LineFollower.h"
#include <cassert>
#include <initializer_list>
// A turn must never latch reduced cruise after fresh centre tape. Coasting
// speed must not suppress the outer-wheel command needed to steer.
int main() {
 for(int side:{1,-1}) {
  line::Follower f; line::Input i; i.tape=2; assert(f.begin(i));
  i.tape=side>0?6:3; auto d=f.step(i);
  assert(d.left>0 && d.right>0 && (d.right-d.left)*side>0);
  i.tape=side>0?4:1; i.leftSpeed=i.rightSpeed=1000;
  d=f.step(i); assert((d.right-d.left)*side>0);
  i.tape=2; d=f.step(i);
  assert(d.left==f.config.cruiseSpeed && d.right==f.config.cruiseSpeed);
 }
}
