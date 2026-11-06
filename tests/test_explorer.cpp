#include "../Robot/Explorer.h"
#include <cassert>
using namespace maze;
struct Rig {
  Explorer e;Observation o;
  Rig() { e.geometry={40,10,20,80,1,.02f};e.dynamics={200,.1f,10};o.front={600,0,true};o.right={600,0,true};assert(e.begin(o.pose)); }
  Drive step(int tape,float l=0,float r=0) {
    ++o.sequence;o.stamp+=20;o.tape=tape;
    float dh=(r-l)/100,ds=(l+r)/2;
    o.pose.x+=ds*cosf(o.pose.heading+dh/2);o.pose.y+=ds*sinf(o.pose.heading+dh/2);
    o.pose.heading+=dh;o.pose.path+=fabsf(ds);o.pose.left+=l;o.pose.right+=r;
    e.observe(o);return e.step(o.stamp);
  }
};
int main() {
  Rig r;auto d=r.step(2);assert(d.left==1&&d.right==1);
  r.step(4,1,1);assert(r.e.state==Explorer::TRACK); // Outer-only does not invent a junction.
  d=r.step(0,1,1);assert(d.left==0&&d.right==0&&r.e.state==Explorer::SETTLE);
  r.step(0,1,1);assert(r.e.state==Explorer::SETTLE); // Moving wheels prevent a scan.
  for(int i=0;i<3;++i) r.step(0);
  assert(r.e.state==Explorer::SEARCH);
  d=r.step(0);assert(d.left==-1&&d.right==1); // Left evidence guides the search.
  auto updates=r.e.world.updates;r.e.step(r.o.stamp);assert(r.e.world.updates==updates);
  r.step(2,-10,10);r.step(2,-1,1);assert(r.e.state==Explorer::VERIFY);
  r.step(0);assert(r.e.state==Explorer::SEARCH&&r.e.graph.nodeCount==1); // Reject a transient hit.
  r.e.pause();int nodes=r.e.graph.nodeCount;assert(r.e.begin(r.o.pose));assert(r.e.graph.nodeCount==nodes);
  r.e.invalidate();assert(!r.e.begin(r.o.pose)&&r.e.graph.nodeCount==nodes);
  Rig restored;restored.step(2);restored.step(0);for(int i=0;i<3;++i) restored.step(2);
  assert(restored.e.state==Explorer::TRACK&&restored.e.graph.nodeCount==1);
  Rig unknown;unknown.e.dynamics=Dynamics();assert(!unknown.e.begin(unknown.o.pose));
  // No encoder movement cannot finish an angular search, regardless of elapsed time.
  Rig still;still.step(2);still.step(0);for(int i=0;i<100;++i) still.step(0);
  assert(still.e.state==Explorer::SEARCH&&still.e.graph.nodeCount==1);
}
