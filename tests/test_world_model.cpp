#include "../Robot/WorldModel.h"
#include <cassert>
using namespace maze;
int main() {
  WorldModel w;Observation o;o.sequence=1;o.stamp=20;o.tape=2;
  o.front={500,20,true};o.right={500,20,true};
  assert(w.update(o,1,40));assert(w.belief[WorldModel::CENTRE]>w.belief[WorldModel::MISSING]);
  float centre=w.belief[1];assert(!w.update(o,1,40));
  for(int i=0;i<20;++i) { ++o.sequence;o.stamp+=20;assert(w.update(o,1,40)); }
  assert(w.belief[1]==centre&&w.updates==1); // No manufactured stationary certainty.
  o.tape=0;++o.sequence;o.stamp+=20;assert(w.update(o,1,40));
  assert(w.belief[WorldModel::MISSING]>w.belief[WorldModel::LEFT]);
  float sum=0;for(float b:w.belief) { assert(isfinite(b)&&b>0);sum+=b; }assert(fabsf(sum-1)<.001f);
  Dynamics d{100, .1f,10};o.front={250,o.stamp,true};o.right={250,o.stamp,true};
  ++o.sequence;o.stamp+=20;o.pose.left+=10;o.pose.right+=10;w.update(o,1,40);
  assert(!guardMotion({1,1},w,d,o.stamp,200,80).allowed); // Speed requires >20 cm stopping distance.
  ++o.sequence;o.stamp+=20;w.update(o,1,40);
  assert(guardMotion({1,1},w,d,o.stamp,200,80).allowed);
  w.latest.right.valid=false;
  assert(guardMotion({1,1},w,d,o.stamp,200,80).allowed);
  assert(!guardMotion({-1,1},w,d,o.stamp,200,80).allowed); // Check proposed command, not prior state.
  assert(!guardMotion({1,1},w,d,o.stamp+101,200,80).allowed);
  assert(!guardMotion({-1,-1},w,d,o.stamp,200,80).allowed);
  assert(!guardMotion({1,1},w,Dynamics(),o.stamp,200,80).allowed);
}
