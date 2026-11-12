#include "../Robot/WallNavigator.h"
#include <cassert>
#include <cstring>
#include <cstdio>
using namespace wall;
Config cfg(){Config c;c.front=90;c.rear=70;c.halfWidth=60;c.frontSensor=80;c.sideX=30;c.sideY=50;c.margin=15;c.deceleration=250;c.reaction=.08f;c.cruise=80;c.turnSpeed=45;c.matchRadius=60;c.distanceTolerance=2;c.angleTolerance=.04f;c.track=120;c.corridorWidth=400;return c;}
void observe(Input&i,float f,float r,float l){i.now+=80;for(int n=0;n<3;++n){auto&x=i.ranges[n];x.valid=true;x.stamp=i.now;x.sequence++;x.pose=i.pose;x.mm=n==0?f:n==1?r:l;}}
int main(){
 maze::Graph g;Navigator n(g);n.config=cfg();Input i;observe(i,600,130,170);
 assert(n.begin(i));observe(i,600,130,170);auto d=n.step(i);assert(d.right>d.left&&d.left>0);
 // Invalid left sonar must hold, not pivot toward apparently absent wall.
 i.ranges[2].valid=false;d=n.step(i);assert(d.left==0&&d.right==0);
 // A front wall requires settling and measured-position assessment, not a blind arc.
 i.pose.x=i.pose.path=300;i.pose.left=i.pose.right=300;
 for(int k=0;k<8;++k){observe(i,55,150,150);d=n.step(i);}
 if(n.state!=Navigator::TURNING)fprintf(stderr,"state %d %s p %f %f %f nodes %d\n",n.state,n.reason,n.beliefs[0].wallProbability,n.beliefs[1].wallProbability,n.beliefs[2].wallProbability,g.nodeCount);assert(n.state==Navigator::TURNING);assert(g.nodeCount==2&&g.edgeCount==1);
 d=n.step(i);assert(d.left*d.right<0); // Dead-end return is a pivot.
 // An obstacle entering the sweep vetoes further turn movement immediately.
 observe(i,55,30,150);d=n.step(i);assert(d.left==0&&d.right==0);
 // Export/stop preserves route; manual relocation invalidates resumption.
 n.pause();assert(g.nodeCount==2);n.invalidate();assert(!n.begin(i));
 // Returning along a known edge must brake BEFORE its endpoint at high speed.
 maze::Graph g3;Navigator returning(g3);returning.config=cfg();Input j;observe(j,600,150,150);assert(returning.begin(j));
 j.pose.path=j.pose.x=j.pose.left=j.pose.right=300;
 for(int k=0;k<8;++k){observe(j,55,150,150);returning.step(j);}
 assert(g3.edgeCount==1);
 returning.state=Navigator::CORRIDOR; // Isolate route-distance braking after completed turn.
 j.pose.heading=maze::PI_F;j.pose.path=560;j.pose.x=40;j.leftSpeed=j.rightSpeed=150;
 observe(j,600,150,150);auto braking=returning.step(j);assert(braking.left==0&&braking.right==0);
 // Wider, ordinary corridors must be classifiable at a dead end.
 maze::Graph g4;Navigator wide(g4);wide.config=cfg();wide.config.corridorWidth=500;
 Input w;observe(w,600,200,200);assert(wide.begin(w));w.pose.path=w.pose.x=300;
 for(int k=0;k<12;++k){observe(w,55,200,200);wide.step(w);}
 assert(wide.state==Navigator::TURNING);
 // An unexpected matching location requires explicit confirmation, then resumes.
 maze::Graph g5;Navigator loop(g5);loop.config=cfg();Input z;observe(z,600,150,150);assert(loop.begin(z));
 float back=maze::PI_F;g5.addNode(300,0,&back,1);z.pose.x=z.pose.path=300;
 for(int k=0;k<8;++k){observe(z,55,150,150);loop.step(z);}
 assert(loop.state==Navigator::CONFIRM&&g5.edgeCount==0);
 assert(!loop.confirm(0)&&loop.confirm(1));assert(g5.edgeCount==1);assert(loop.begin(z));loop.step(z);assert(loop.state==Navigator::TURNING);
 // Never evict route history or guess between two nearby nodes.
 maze::Graph g6;Navigator ambiguous(g6);ambiguous.config=cfg();Input a;observe(a,600,150,150);assert(ambiguous.begin(a));
 g6.addNode(280,0,&back,1);g6.addNode(320,0,&back,1);a.pose.x=a.pose.path=300;
 for(int k=0;k<8;++k){observe(a,55,150,150);ambiguous.step(a);}
 assert(ambiguous.state==Navigator::FAULT&&g6.edgeCount==0&&g6.nodeCount==3);
 maze::Graph g7;Navigator full(g7);full.config=cfg();Input m;observe(m,600,150,150);assert(full.begin(m));
 for(int k=1;k<64;++k)g7.addNode(k*1000,1000,&back,1);
 m.pose.x=m.pose.path=300;for(int k=0;k<8;++k){observe(m,55,150,150);full.step(m);}
 assert(full.state==Navigator::FAULT&&g7.nodeCount==64&&g7.edgeCount==0);
 // Configuration must be real, not guessed zeros.
 maze::Graph g2;Navigator unconfigured(g2);assert(!unconfigured.begin(i));
}
