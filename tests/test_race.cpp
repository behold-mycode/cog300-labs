#include "../Robot/Race.h"
#include <cassert>
int main(){
 race::Button b;assert(b.update(true,0,false)==race::Button::NONE);
 assert(b.update(false,1000,false)==race::Button::NONE);b.update(false,1050,false);
 b.update(true,1100,false);b.update(false,1200,false);assert(b.update(false,1250,false)==race::Button::NONE);
 b.update(true,1300,false);b.update(false,2200,false);assert(b.update(false,2250,false)==race::Button::START);
 assert(b.update(true,2300,true)==race::Button::STOP);
 b.update(false,3500,false);assert(b.update(false,3550,false)==race::Button::NONE);
 race::Navigator n;wall::Config c;c.front=90;c.rear=70;c.halfWidth=60;c.frontSensor=80;c.sideX=30;c.sideY=50;c.margin=15;c.deceleration=200;c.reaction=.15;c.cruise=80;c.turnSpeed=45;c.track=120;c.distanceTolerance=2;c.angleTolerance=.04;c.matchRadius=60;c.corridorWidth=400;
 race::Config r;r.speed=60;r.sensorOffset=100;r.searchAngle=1.8f;
 wall::Input i;i.now=100;for(auto &s:i.ranges){s.valid=true;s.mm=500;s.stamp=100;}
 assert(n.begin(r,c,i,2));auto d=n.step(r,c,i,2);assert(d.left>0&&d.right>0);
 i.now+=20;d=n.step(r,c,i,4);assert(d.right>d.left);
 // Side ray clears the rectangular half-width but not the turning footprint.
 for(auto &s:i.ranges){s.stamp=i.now;s.pose=i.pose;}i.ranges[1].mm=60;
 d=n.step(r,c,i,4);assert(d.left==0&&d.right==0);
 i.leftSpeed=0;i.rightSpeed=60;d=n.step(r,c,i,2);assert(d.left==0&&d.right==0); // Residual turn/coast also guarded.
 i.leftSpeed=i.rightSpeed=0;i.ranges[1].mm=500;
 i.now+=20;d=n.step(r,c,i,0);assert(d.left==0&&d.right==0&&!n.wallStage());
 // Combined line/maze pivots still need swept-body clearance.
 race::Navigator wide;auto wideBody=c;wideBody.halfWidth=120;
 i.ranges[0].mm=60;for(auto &s:i.ranges){s.stamp=i.now;s.pose=i.pose;}
 assert(wide.begin(r,wideBody,i,2,true));auto blocked=wide.step(r,wideBody,i,4);
 assert(blocked.left==0&&blocked.right==0);
 i.ranges[0].mm=500;
 // Tape disappearance alone must not immediately switch to maze.
 for(int k=0;k<5;++k){i.now+=20;n.step(r,c,i,0);}assert(!n.wallStage());
}
