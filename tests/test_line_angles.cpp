#include "../Robot/Race.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
float segment(float x,float y,float ax,float ay,float bx,float by){float dx=bx-ax,dy=by-ay;float t=wall::limit(((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy),0,1);return hypotf(x-ax-t*dx,y-ay-t*dy);}
int main(){
 for(float degrees:{45.f,-45.f,90.f,-90.f,135.f,-135.f,150.f,-150.f}){
  wall::Config c;c.front=100;c.rear=70;c.halfWidth=60;c.frontSensor=80;c.sideX=30;c.sideY=50;c.margin=15;c.deceleration=200;c.reaction=.15;c.cruise=50;c.turnSpeed=35;c.track=120;c.distanceTolerance=2;c.angleTolerance=.04;c.matchRadius=60;c.corridorWidth=400;
  race::Config cfg;cfg.speed=50;cfg.sensorOffset=90;cfg.searchAngle=3.f;
  race::Navigator nav;wall::Input i;wall::Speeds d;assert(nav.begin(cfg,c,i,2,false));
  float angle=degrees*maze::PI_F/180;bool reached=false;
  for(int k=0;k<15000&&!reached;++k){
   float l=d.left*.02f,r=d.right*.02f,dh=(r-l)/120,ds=(l+r)*.5f;
   i.pose.x+=ds*cosf(i.pose.heading+dh*.5f);i.pose.y+=ds*sinf(i.pose.heading+dh*.5f);i.pose.heading+=dh;i.pose.left+=l;i.pose.right+=r;i.pose.path+=fabsf(ds);i.now+=20;i.leftSpeed=d.left;i.rightSpeed=d.right;
   auto&s=i.ranges[0];s.valid=true;s.mm=3000;s.stamp=i.now;s.pose=i.pose;s.sequence++;
   unsigned mask=0;for(int n=0;n<3;++n){float y=15.f*(1-n),x=i.pose.x+90*cosf(i.pose.heading)-y*sinf(i.pose.heading),wy=i.pose.y+90*sinf(i.pose.heading)+y*cosf(i.pose.heading);
    if(fminf(segment(x,wy,-200,0,400,0),segment(x,wy,400,0,400+500*cosf(angle),500*sinf(angle)))<10)mask|=4>>n;
   }
   d=nav.step(cfg,c,i,mask);
   reached=hypotf(i.pose.x-(400+250*cosf(angle)),i.pose.y-250*sinf(angle))<45;
   if(nav.state==race::Navigator::FAULT)break;
  }
  if(!reached)fprintf(stderr,"angle %.0f: %s x %.1f y %.1f heading %.2f\n",degrees,nav.reason,i.pose.x,i.pose.y,i.pose.heading);
  assert(reached);
 }
}
