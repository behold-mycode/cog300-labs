// Closed loop through actual firmware, encoder odometry, velocity control and
// L298N coast/reversal behavior. Walls use full rectangular chassis collision
// checks, not merely the three sensor rays. Sonar beam effects remain unmodelled.
#include "firmware_fixture.h"
#include <cstdio>
struct Point{float x,y;};
const Point polygon[]={{-300,-200},{1100,-200},{1100,200},{800,200},{800,900},{400,900},{400,200},{-300,200}};
bool inside(float x,float y){bool in=false;for(int n=0,j=7;n<8;j=n++){
 auto a=polygon[n],b=polygon[j];if(((a.y>y)!=(b.y>y))&&(x<(b.x-a.x)*(y-a.y)/(b.y-a.y)+a.x))in=!in;
}return in;}
float ray(float x,float y,float angle){
 float dx=cosf(angle),dy=sinf(angle),best=10000;
 for(int n=0;n<8;++n){auto a=polygon[n],b=polygon[(n+1)%8];float sx=b.x-a.x,sy=b.y-a.y;
  float den=dx*sy-dy*sx;if(fabsf(den)<.0001f)continue;
  float t=((a.x-x)*sy-(a.y-y)*sx)/den,u=((a.x-x)*dy-(a.y-y)*dx)/den;
  if(t>0&&u>=0&&u<=1)best=fminf(best,t);
 }return best;
}
bool intersects(Point a,Point b,Point c,Point d){
 float sx=b.x-a.x,sy=b.y-a.y,tx=d.x-c.x,ty=d.y-c.y,den=sx*ty-sy*tx;
 if(fabsf(den)<1e-6f)return false;
 float u=((c.x-a.x)*ty-(c.y-a.y)*tx)/den,v=((c.x-a.x)*sy-(c.y-a.y)*sx)/den;
 return u>=0&&u<=1&&v>=0&&v<=1;
}
void sonar(const maze::Pose&p,int n){
 float sx=n==0?80:30,sy=n==0?0:n==1?-50:50;
 float x=p.x+sx*cosf(p.heading)-sy*sinf(p.heading),y=p.y+sx*sinf(p.heading)+sy*cosf(p.heading);
 float a=p.heading+(n==0?0:n==1?-maze::PI_F/2:maze::PI_F/2);
 float mm=ray(x,y,a);acceptReading(n,(int)(mm/10),(uint32_t)(mm*5.8f));
}
int main(){
 setup();command("SONAR 1 40");command("ENC 1 0 0");command("ODOM 1 1 120");
 command("BODY 90 70 60 80 30 50");command("WALLCFG 80 45 15 200 0.15 140 400");command("SPEEDCFG 200 190 0.6 0.3 150");
 serviceOdometry();maze::Pose actual;for(int n=0;n<3;++n)sonar(actual,n);
 command("MODE WALL_MAZE");assert(mode==WALL_MAZE);
 float vl=0,vr=0;bool complete=false;int sensor=0;
 for(int k=0;k<12000;++k){
  auto accelerate=[](float v,float target){return v+wall::limit(target-v,-4,4);};
  vl=accelerate(vl,motorLeft*200/255.f);vr=accelerate(vr,motorRight*190/255.f);
  float dl=vl*.02f,dr=vr*.02f,dh=(dr-dl)/120,ds=(dl+dr)*.5f;
  actual.x+=ds*cosf(actual.heading+dh*.5f);actual.y+=ds*sinf(actual.heading+dh*.5f);
  actual.heading+=dh;actual.left+=dl;actual.right+=dr;
  leftTicks=(uint32_t)(int32_t)lroundf(actual.left);rightTicks=(uint32_t)(int32_t)lroundf(actual.right);
  nowMs+=20;serviceOdometry();
  if(k%2==0){sonar(actual,sensor);sensor=(sensor+1)%3;}
  serviceObservation();serviceNavigation();serviceMotor();
  Point corners[4];const Point local[]={{-70,-60},{90,-60},{90,60},{-70,60}};
  for(int n=0;n<4;++n)corners[n]={actual.x+local[n].x*cosf(actual.heading)-local[n].y*sinf(actual.heading),actual.y+local[n].x*sinf(actual.heading)+local[n].y*cosf(actual.heading)};
  for(int a=0;a<4;++a)for(int b=0;b<8;++b)assert(!intersects(corners[a],corners[(a+1)%4],polygon[b],polygon[(b+1)%8]));
  for(float x:{-70.f,90.f})for(float y:{-60.f,60.f}){
   float wx=actual.x+x*cosf(actual.heading)-y*sinf(actual.heading);
   float wy=actual.y+x*sinf(actual.heading)+y*cosf(actual.heading);
   if(!(inside(wx,wy)))fprintf(stderr,"collision %s at %.1f %.1f %.2f\n",navigationState,actual.x,actual.y,actual.heading);
   assert(inside(wx,wy));
  }
  if(wallNavigator.state==wall::Navigator::COMPLETE){complete=true;break;}
  if(wallNavigator.state==wall::Navigator::FAULT)break;
 }
 if(!complete)fprintf(stderr,"replay stopped %s state %d at %.1f %.1f %.2f targets %.1f %.1f actual %.1f %.1f\n",navigationState,wallNavigator.state,actual.x,actual.y,actual.heading,targets.left,targets.right,vl,vr);
 assert(complete);assert(fabsf(actual.x)<140&&fabsf(actual.y)<40);
 assert(explorer.graph.edgeCount==3);for(int n=0;n<3;++n)assert(explorer.graph.edges[n].traversals==2);
}
