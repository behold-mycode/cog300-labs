#include "../Robot/Race.h"
#include <cassert>
int main(){
 wall::Config c;c.front=150;c.rear=70;c.halfWidth=80;c.frontSensor=150;c.sideX=45;c.sideY=65;c.margin=23;c.deceleration=200;c.reaction=.35f;c.cruise=50;c.turnSpeed=35;c.track=130;c.distanceTolerance=7.657632f;c.angleTolerance=.11781f;c.matchRadius=60;c.corridorWidth=400;
 race::Config cfg;cfg.speed=50;cfg.sensorOffset=150;
 race::Navigator nav;wall::Input i;
 auto step=[&](uint32_t time,float heading,int tape){
  i.now=time;i.pose.heading=heading;i.pose.left=-heading*65;i.pose.right=heading*65;
  for(auto&s:i.ranges){s.valid=true;s.mm=1000;s.stamp=time;s.pose=i.pose;++s.sequence;}
  return nav.step(cfg,c,i,tape);
 };
 assert(nav.begin(cfg,c,i,2,false));
 step(20,0,0);step(300,0,0);step(320,0,0);
 const float quantum=5.105088f/130;
 step(340,quantum,2);step(360,2*quantum,0);
 assert(nav.state==race::Navigator::CENTRE_LINE);
 // Midpoint is 1.5 encoder steps. At one step, the physical sensor detects
 // tape, but the old .01-radian tolerance would demand an unattainable count.
 auto d=step(380,quantum,2);
 assert(nav.state==race::Navigator::SETTLE&&d.left==0&&d.right==0);
}
