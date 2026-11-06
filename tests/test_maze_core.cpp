#include "../Robot/MazeGraph.h"
#include "../Robot/Odometry.h"
#include "../Robot/Motion.h"
#include <cassert>
using namespace maze;
int main() {
  Graph g; float root[]={0,PI_F/2}, entry[]={PI_F}, junction[]={PI_F,0,PI_F/2};
  assert(g.addNode(0,0,root,2)==0);
  auto c=g.plan(0); assert(c.port==0&&c.exploring);
  assert(g.addNode(100,0,entry,1)==1);assert(g.connect(0,0,1,0,100));
  c=g.plan(1,0);assert(c.port==0&&!c.exploring); // Backtrack to untried root exit.
  assert(g.addNode(0,100,junction,3)==2);assert(g.connect(0,1,2,0,100));
  c=g.plan(2,0);assert(c.port==1&&c.exploring);
  auto before=g.edgeCount;assert(!g.connect(0,0,2,1,30));assert(g.edgeCount==before);
  assert(g.connect(2,1,2,2,300)); // Explicitly identified cycle/self-loop is legal.
  assert(g.plan(2,0).port==-1); // Whole connected graph explored.
  assert(!g.connect(-1,0,2,0,1));
  assert(g.nearestPort(0,0,.1f)==0);
  for(int i=g.nodeCount;i<Graph::MAX_NODES;++i) assert(g.addNode(i,0,entry,1)==i);
  assert(g.addNode(0,0,entry,1)==-1&&g.nodeCount==64);
  Odometry o;assert(!o.configure(0,1,100));assert(o.configure(1,1,100));
  o.reset(0,0,0);o.update(100,100,1);assert(fabsf(o.pose.x-100)<.01f&&fabsf(o.pose.heading)<.01f);
  o.update(50,150,2);assert(fabsf(o.pose.heading-1)<.01f);
  Motion m;m.turn(1,o.pose);auto d=m.step(o.pose,1,.01f);assert(d.left==-1&&d.right==1&&!m.done);
  o.update(0,200,3);d=m.step(o.pose,1,.01f);assert(m.done&&d.left==0);
  m.move(100,o.pose);o.update(100,300,4);m.step(o.pose,1,.01f);assert(m.done);
  o.reset(INT32_MAX-2,INT32_MAX-2,5);o.update(INT32_MIN+2,INT32_MIN+2,6);assert(o.pose.x==5);
  ProgressGuard guard;assert(guard.check(o.pose,{1,1},0,100));
  Pose p=o.pose;p.left+=1;assert(!guard.check(p,{1,1},100,100)); // Right stalled.
}
