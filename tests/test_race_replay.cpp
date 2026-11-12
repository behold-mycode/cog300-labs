#include "../Robot/Race.h"
#include <cassert>
#include <cstring>
struct Course {
 race::Navigator nav;race::Config race;wall::Config body;wall::Input in;wall::Speeds last;
 Course(){
  auto&c=body;c.front=90;c.rear=70;c.halfWidth=60;c.frontSensor=80;c.sideX=30;c.sideY=50;c.margin=15;c.deceleration=200;c.reaction=.15;c.cruise=80;c.turnSpeed=45;c.track=120;c.distanceTolerance=2;c.angleTolerance=.04;c.matchRadius=60;c.corridorWidth=400;
  race.speed=60;race.sensorOffset=100;race.searchAngle=1.8f;assert(nav.begin(race,body,in,2));
 }
 void tick(int tape,bool corridor=true){
  constexpr float dt=.02f;in.now+=20;
  // Ideal differential-drive plant. Deliberately no slip or sonar noise here.
  float l=last.left*dt,r=last.right*dt,angle=(r-l)/body.track,travel=(l+r)*.5f;
  in.pose.x+=travel*cosf(in.pose.heading+angle*.5f);in.pose.y+=travel*sinf(in.pose.heading+angle*.5f);in.pose.heading+=angle;
  in.pose.left+=l;in.pose.right+=r;in.pose.path+=fabsf(travel);in.leftSpeed=last.left;in.rightSpeed=last.right;
  for(int n=0;n<3;++n){auto&s=in.ranges[n];s.valid=true;s.mm=n&&corridor?150:500;s.stamp=in.now;s.sequence++;s.pose=in.pose;}
  last=nav.step(race,body,in,tape);
 }
 void approach(){for(int k=0;k<300;++k)tick(2);assert(in.pose.x>300);}
};
int main(){
 Course end;end.approach();int steps=0;
 while(!end.nav.wallStage()&&end.nav.state!=race::Navigator::FAULT&&steps++<6000)end.tick(0);
 assert(end.nav.wallStage());assert(end.in.pose.x>400&&end.in.pose.x<480);assert(fabsf(end.in.pose.heading)<.1f);
 Course missed;missed.approach();bool recovered=false;
 for(int k=0;k<2000&&!recovered;++k){
  bool onBend=missed.in.pose.heading>.65f&&missed.in.pose.heading<.85f;
  missed.tick(onBend?2:0);
  recovered=missed.nav.state==race::Navigator::TRACK;
  if(k==0)recovered=false;
 }
 assert(recovered&&!missed.nav.wallStage());
 Course open;open.approach();for(int k=0;k<6000&&open.nav.state!=race::Navigator::FAULT;++k)open.tick(0,false);
 assert(open.nav.state==race::Navigator::FAULT&&!open.nav.wallStage());
 assert(!strcmp(open.nav.reason,"RACE_TRANSITION_UNCONFIRMED"));
}
