#include "../Robot/ObjectSearch.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
wall::Config body(){wall::Config c;c.front=90;c.rear=70;c.halfWidth=60;c.frontSensor=80;c.sideX=30;c.sideY=50;c.margin=15;c.deceleration=200;c.reaction=.15;c.cruise=60;c.turnSpeed=45;c.track=120;c.distanceTolerance=2;c.angleTolerance=.04;c.matchRadius=60;c.corridorWidth=400;return c;}
int main(){
 for(float bearing:{0.f,1.57f,-1.57f,3.14f}){
  object::Search search;auto c=body();wall::Input i;search.begin(i);wall::Speeds d;bool approaching=false;
  for(int k=0;k<20000&&!approaching;++k){
   i.now+=20;float l=d.left*.02f,r=d.right*.02f;i.pose.left+=l;i.pose.right+=r;i.pose.heading+=(r-l)/120;i.leftSpeed=d.left;i.rightSpeed=d.right;
   auto&s=i.ranges[0];s.valid=true;s.noReturn=false;s.stamp=i.now;s.sequence++;s.pose=i.pose;
   // A second, more distant object must not win the nearest-object selection.
   s.mm=maze::angleDistance(i.pose.heading,bearing)<.12f?500:maze::angleDistance(i.pose.heading,bearing+1.f)<.12f?900:2000;
   d=search.step(c,i,0);approaching=search.state==object::Search::APPROACH;
  }
  if(!approaching)fprintf(stderr,"object %f %s\n",bearing,search.reason);
  assert(approaching);assert(maze::angleDistance(i.pose.heading,bearing)<.2f);
  assert(search.bestRange<600);
  auto d2=search.step(c,i,1);assert(d2.left==0&&d2.right==0&&search.state==object::Search::BOUNDARY);
 }
 object::Search empty;auto c=body();wall::Input i;empty.begin(i);wall::Speeds d;
 for(int k=0;k<20000&&empty.state!=object::Search::FAULT;++k){
  i.now+=20;float l=d.left*.02f,r=d.right*.02f;i.pose.left+=l;i.pose.right+=r;i.pose.heading+=(r-l)/120;i.leftSpeed=d.left;i.rightSpeed=d.right;
  auto&s=i.ranges[0];s.valid=false;s.noReturn=true;s.stamp=i.now;s.sequence++;s.pose=i.pose;
  d=empty.step(c,i,0);
 }
 assert(empty.state==object::Search::FAULT);
}
