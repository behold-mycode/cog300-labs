#define ROBOT_ENABLE_STANDALONE 1
// Fresh boot -> automatic commissioning -> line -> wall -> dead-end -> return.
// Actual production firmware, synthetic mechanics and ray sonar; no USB setup commands.
#include "firmware_fixture.h"
#include <cstdio>
float cast(float x,float y,float a){float dx=cosf(a),dy=sinf(a),best=4000;
 if(dx>.0001f)best=fminf(best,(1100-x)/dx);if(dx<-.0001f)best=fminf(best,(-400-x)/dx);
 if(dy>.0001f)best=fminf(best,(200-y)/dy);if(dy<-.0001f)best=fminf(best,(-200-y)/dy);return best;}
void reading(const maze::Pose&p,int n){float sx=n==0?80:30,sy=n==0?0:n==1?-50:50;
 float x=p.x+sx*cosf(p.heading)-sy*sinf(p.heading),y=p.y+sx*sinf(p.heading)+sy*cosf(p.heading);
 float mm=cast(x,y,p.heading+(n==0?0:n==1?-maze::PI_F/2:maze::PI_F/2));acceptReading(n,(int)(mm/10),(uint32_t)(mm*5.8f));}
int main(){
 setup();hardware::Profile p;p.leftMmPerEdge=p.rightMmPerEdge=1;p.track=120;p.floorSensorOffset=90;p.leftSonarInstalled=true;p.front=90;p.rear=70;p.halfWidth=60;p.frontSonar=80;p.sideSonarX=30;p.sideSonarY=50;
 assert(configureHardware(p));maze::Pose actual;for(int n=0;n<3;++n)reading(actual,n);tape(2);
 assert(startStandalone());float vl=0,vr=0,l=0,r=0;bool learned=false,wall=false,turned=false,finished=false;int sensor=0;uint32_t lastPing=nowMs;
 for(int k=0;k<18000;++k){
  vl+=wall::limit(motorLeft*200/255.f-vl,-4.f,4.f);vr+=wall::limit(motorRight*190/255.f-vr,-4.f,4.f);
  float dl=vl*.02f,dr=vr*.02f,dh=(dr-dl)/120,ds=(dl+dr)*.5f;l+=dl;r+=dr;
  actual.x+=ds*cosf(actual.heading+dh*.5f);actual.y+=ds*sinf(actual.heading+dh*.5f);actual.heading+=dh;
  leftTicks=(int32_t)lroundf(l);rightTicks=(int32_t)lroundf(r);nowMs+=20;serviceOdometry();
  // Short straight line ends before the maze wall section.
  float sensorY=actual.y+90*sinf(actual.heading);tape(actual.x<400&&fabsf(sensorY)<15?2:0);
  if((uint32_t)(nowMs-lastPing)>=70){reading(actual,sensor);sensor=(sensor+1)%3;lastPing=nowMs;}
  serviceObservation();serviceNavigation();serviceMotor();
  learned|=calibrationSaved();wall|=raceWallStarted;turned|=fabsf(actual.heading)>2.8f;
  for(float x:{-70.f,90.f})for(float y:{-60.f,60.f}){
   float wx=actual.x+x*cosf(actual.heading)-y*sinf(actual.heading),wy=actual.y+x*sinf(actual.heading)+y*cosf(actual.heading);
   assert(wx<1100&&wx>-400&&fabsf(wy)<200);
  }
  if(wallNavigator.state==wall::Navigator::COMPLETE){finished=true;break;}
  // Returning to the line is also a successful test endpoint: route remains.
  if(turned&&!raceWallStarted&&resumeWallGraph){finished=true;break;}
  if(mode==MANUAL)break;
 }
 if(!finished)fprintf(stderr,"auto-course %s at %.1f %.1f %.2f learned=%d wall=%d turned=%d\n",navigationState,actual.x,actual.y,actual.heading,learned,wall,turned);
 assert(learned&&wall&&turned&&finished);assert(explorer.graph.nodeCount>=2);
}
