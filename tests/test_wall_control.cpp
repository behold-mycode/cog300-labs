// Regressions: a clear front ray must not allow a chassis corner to hit a wall;
// wheel feedback must correct unequal speed, and turns must settle on measured pose.
#include "../Robot/WallNavigator.h"
#include "../Robot/Velocity.h"
#include <cassert>
#include <cstring>
using namespace wall;
Config config() {
 Config c;c.front=90;c.rear=70;c.halfWidth=60;c.frontSensor=80;c.sideX=30;c.sideY=50;
 c.margin=15;c.deceleration=250;c.reaction=.08f;c.cruise=80;c.turnSpeed=45;
 c.matchRadius=60;c.distanceTolerance=2;c.angleTolerance=.04f;c.track=120;c.corridorWidth=400;
 return c;
}
Input sample(float front=600,float left=150,float right=150) {
 Input i;i.now=100;for(auto &r:i.ranges){r.stamp=100;r.sequence=1;r.valid=true;}
 i.ranges[0].mm=front;i.ranges[1].mm=right;i.ranges[2].mm=left;return i;
}
int main() {
 Config c=config();assert(c.valid());
 // Radius is sqrt(90^2+60^2)=108.17; margin makes minimum axle clearance 123.17.
 assert(c.radius()>108&&c.radius()<109);
 Input i=sample(500,50,150); // Left wall 100mm from axle: front is clear, corner is not.
 assert(!turnClear(c,i,300));
 i=sample();assert(turnClear(c,i,300));assert(!turnClear(c,i,20)); // Unobserved rear.
 i.ranges[2].valid=false;assert(!turnClear(c,i,300));
 i=sample();i.now=500;assert(!turnClear(c,i,300));
 Belief b;Ray r;r.valid=true;r.mm=100;r.sequence=1;r.stamp=100;
 b.update(r,100,200,400);float first=b.wallProbability;
 for(int n=0;n<100;++n)b.update(r,100,200,400);
 assert(b.wallProbability==first);
 r.sequence++;r.stamp+=80;b.update(r,180,200,400);assert(b.wallProbability>first);
 r.sequence++;r.stamp+=80;r.mm=600;b.update(r,260,200,400);assert(b.wallProbability<.9f);
 // Speed control must lower effort for an overspeed wheel and raise the slower one.
 VelocityConfig v;v.fullLeft=200;v.fullRight=200;v.kp=1;v.ki=.2f;v.acceleration=1000;
 Velocity controller;controller.config=v;controller.reset();
 auto pwm=controller.step(80,80,120,20,.1f,255);assert(pwm.left<pwm.right);
 pwm=controller.step(0,0,100,100,.02f,255);assert(pwm.left==0&&pwm.right==0);
 // A spin has equal/opposite wheel travel; slowing the leading wheel preserves centre.
 Turn t;t.begin(maze::PI_F/2,i.pose);i=sample();i.pose.left=-30;i.pose.right=10;
 auto d=t.step(c,i);assert(d.left<0&&d.right>0);assert(-d.left<d.right);
 i.pose.left=-94.25f;i.pose.right=94.25f;i.pose.heading=maze::PI_F/2;
 i.leftSpeed=-40;i.rightSpeed=40;d=t.step(c,i);assert(!t.done&&d.left==0&&d.right==0);
 i.leftSpeed=i.rightSpeed=0;for(int n=0;n<20;++n){i.now+=20;d=t.step(c,i);}assert(t.done);
 // A large asymmetric displacement is a fault, never a completed turn.
 t.begin(maze::PI_F/2,maze::Pose());i=sample();i.pose.left=-50;i.pose.right=5;
 d=t.step(c,i);assert(t.failed&&d.left==0&&d.right==0);
}
