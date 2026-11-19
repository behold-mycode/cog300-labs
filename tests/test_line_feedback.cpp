#include "../Robot/LineFollower.h"
#include <cassert>
#include <cstring>
// Regression scenarios formerly implemented with geometry and search phases.
// Sensor changes now take precedence over all pose and wheel-speed history.
int main(int argc,char**argv) {
 assert(argc==2); line::Follower f; line::Input i; i.tape=2; assert(f.begin(i));
 if(!strcmp(argv[1],"gap")) {
  i.tape=0; auto d=f.step(i); assert(d.left>0 && d.left==d.right);
  i.tape=2; d=f.step(i); assert(d.left==f.config.cruiseSpeed);
 } else if(!strcmp(argv[1],"side") || !strcmp(argv[1],"exit")) {
  i.tape=4; f.step(i); i.tape=0; f.step(i);
  i.tape=1; auto d=f.step(i); assert(d.left>0 && d.right==0);
 } else if(!strcmp(argv[1],"coasting")) {
  i.leftSpeed=i.rightSpeed=1000; i.tape=1;
  auto d=f.step(i); assert(d.left>0 && d.right==0);
 } else if(!strcmp(argv[1],"confirmation")) {
  for(int n=0;n<100;++n) {
   i.tape=4; f.step(i); i.tape=2; auto d=f.step(i);
   assert(d.left==f.config.cruiseSpeed && d.left==d.right);
  }
 } else {
  i.tape=4; f.step(i); i.tape=0; i.pose.heading=100;
  auto d=f.step(i); assert(d.left==0 && d.right>0);
  i.tape=2; d=f.step(i); assert(d.left==d.right && d.left>0);
 }
}
