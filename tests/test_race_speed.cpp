#include "../Robot/Race.h"
#include <cassert>
int main(){
 wall::Config c;c.front=90;c.rear=70;c.halfWidth=60;c.frontSensor=80;c.sideX=30;c.sideY=50;c.margin=15;c.deceleration=200;c.reaction=.15;c.cruise=80;c.turnSpeed=45;c.track=120;c.distanceTolerance=2;c.angleTolerance=.04;c.matchRadius=60;c.corridorWidth=400;
 race::Config r;r.speed=60;r.sensorOffset=100;wall::Input i;i.now=100;
 for(auto &s:i.ranges){s.valid=true;s.mm=500;s.stamp=i.now;}
 race::Navigator n;assert(n.begin(r,c,i,2));
 i.ranges[0].mm=35;auto d=n.step(r,c,i,2);
 assert(d.left>0&&d.left<60&&d.left==d.right);
 float room=35+c.frontSensor-c.front-c.margin;
 assert(d.left*c.reaction+d.left*d.left/(2*c.deceleration)<=room+.01f);
 i.ranges[0].mm=500;i.ranges[1].mm=100;
 d=n.step(r,c,i,4);assert(d.left<0&&d.right>0&&d.right<=45);
 assert(fabsf(d.left+d.right)<.001f);
 room=100+c.sideY-c.radius()-c.margin;
 assert(d.right*c.reaction+d.right*d.right/(2*c.deceleration)<=room+.01f);
 // Existing momentum that already consumes clearance demands a stop, not a slow command.
 i.leftSpeed=60;i.rightSpeed=81;d=n.step(r,c,i,4);assert(d.left==0&&d.right==0);
 i.leftSpeed=i.rightSpeed=0;i.ranges[1].valid=false;d=n.step(r,c,i,2);assert(d.left==0&&d.right==0);
}
