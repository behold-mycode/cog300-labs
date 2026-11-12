// Closed loop through actual firmware, encoder odometry, velocity control and
// L298N coast/reversal behavior. Walls use full rectangular chassis collision
// checks, not merely the three sensor rays. Sonar beam effects remain unmodelled.
#include "firmware_fixture.h"
#include <cstdio>
float ray(float x,float y,float angle){
 float dx=cosf(angle),dy=sinf(angle),best=10000;
 if(dx>.0001f)best=fminf(best,(650-x)/dx);
 if(dx<-.0001f)best=fminf(best,(-300-x)/dx);
 if(dy>.0001f)best=fminf(best,(200-y)/dy);
 if(dy<-.0001f)best=fminf(best,(-200-y)/dy);
 return best;
}
void sonar(const maze::Pose&p,int n){
 float sx=n==0?80:30,sy=n==0?0:n==1?-50:50;
 float x=p.x+sx*cosf(p.heading)-sy*sinf(p.heading),y=p.y+sx*sinf(p.heading)+sy*cosf(p.heading);
 float a=p.heading+(n==0?0:n==1?-maze::PI_F/2:maze::PI_F/2);
 float mm=ray(x,y,a);acceptReading(n,(int)(mm/10),(uint32_t)(mm*5.8f));
}
int main(int argc,char** argv){
 int scenario=argc>1?atoi(argv[1]):0;
 float wheelL=0,wheelR=0;
 setup();command("SONAR 1 40");command("ENC 1 0 0");command("ODOM 1 1 120");
 command("BODY 90 70 60 80 30 50");command("WALLCFG 80 45 15 200 0.15 60 400");command("SPEEDCFG 200 190 0.6 0.3 150");
 if(scenario==2)command("WALLCFG 150 45 15 100 0.15 60 400");
 if(scenario==3)command("WALLCFG 150 45 15 100 0.5 40 400");
 serviceOdometry();maze::Pose actual;for(int n=0;n<3;++n)sonar(actual,n);
 command("MODE WALL_MAZE");assert(mode==WALL_MAZE);
 float vl=0,vr=0;bool turned=false,complete=false;int sensor=0;
 for(int k=0;k<12000;++k){
  auto accelerate=[scenario](float v,float target){float a=scenario>=2?2:4;return v+wall::limit(target-v,-a,a);};
  vl=accelerate(vl,motorLeft*200/255.f);vr=accelerate(vr,motorRight*190/255.f);
  wheelL+=vl*.02f;wheelR+=vr*.02f;
  float dl=vl*.02f,dr=vr*.02f*(scenario==1?.98f:1),dh=(dr-dl)/120,ds=(dl+dr)*.5f;
  actual.x+=ds*cosf(actual.heading+dh*.5f);actual.y+=ds*sinf(actual.heading+dh*.5f);
  actual.heading+=dh;actual.left+=dl;actual.right+=dr;
  leftTicks=(uint32_t)(int32_t)lroundf(wheelL);rightTicks=(uint32_t)(int32_t)lroundf(wheelR);
  nowMs+=20;serviceOdometry();
  if(k%2==0){sonar(actual,sensor);sensor=(sensor+1)%3;}
  serviceObservation();serviceNavigation();serviceMotor();
  for(float x:{-70.f,90.f})for(float y:{-60.f,60.f}){
   float wx=actual.x+x*cosf(actual.heading)-y*sinf(actual.heading);
   float wy=actual.y+x*sinf(actual.heading)+y*cosf(actual.heading);
   if(!(wx<650&&wx>-300&&fabsf(wy)<200))fprintf(stderr,"collision %s at %.1f %.1f %.2f\n",navigationState,actual.x,actual.y,actual.heading);
   assert(wx<650&&wx>-300&&fabsf(wy)<200);
  }
  turned|=fabsf(actual.heading)>2.8f;
  if(wallNavigator.state==wall::Navigator::COMPLETE){complete=true;break;}
  if(wallNavigator.state==wall::Navigator::FAULT)break;
 }
 if(!complete)fprintf(stderr,"replay stopped %s state %d at %.1f %.1f %.2f targets %.1f %.1f actual %.1f %.1f\n",navigationState,wallNavigator.state,actual.x,actual.y,actual.heading,targets.left,targets.right,vl,vr);
 assert(turned&&complete);assert(fabsf(actual.x)<60&&fabsf(actual.y)<40);
 assert(explorer.graph.edgeCount==1&&explorer.graph.edges[0].traversals==2);
}
