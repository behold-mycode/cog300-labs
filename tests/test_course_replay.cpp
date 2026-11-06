// Closed-loop differential-drive replay: sensors are generated from chassis pose
// and finite-width tape segments, not scripted state transitions. Synthetic only.
#include "../Robot/Explorer.h"
#include <cassert>
#include <cstdio>
using namespace maze;
struct Segment { float ax,ay,bx,by; };
float distance(float x,float y,Segment s) {
  float dx=s.bx-s.ax,dy=s.by-s.ay;
  float t=fmaxf(0,fminf(1,((x-s.ax)*dx+(y-s.ay)*dy)/(dx*dx+dy*dy)));
  return hypotf(x-s.ax-t*dx,y-s.ay-t*dy);
}
void run(bool noisy,float rightSlip,bool sharp=false,bool endpoint=false) {
  Segment course[]={{-100,0,300,0},{300,0,300,200},{300,200,500,200}};
  if(sharp) { course[1]={300,0,150,150};course[2]={150,150,500,150}; }
  if(endpoint) for(auto &s:course) s={-100,0,200,0};
  Explorer e;e.geometry={40,20,25,80,.25f,.005f};e.dynamics={400,.04f,10};
  Pose p,estimated;assert(e.begin(p));float vl=0,vr=0;Drive command;bool reached=false;
  for(int tick=1;tick<12000;++tick) {
    auto velocity=[](float current,float target) { float delta=target-current;return current+fmaxf(-8,fminf(8,delta)); };
    vl=velocity(vl,command.left*100);vr=velocity(vr,command.right*100);
    float dl=vl*.02f,dr=vr*.02f*rightSlip,dh=(dr-dl)/100,ds=(dl+dr)*.5f;
    p.x+=ds*cosf(p.heading+dh*.5f);p.y+=ds*sinf(p.heading+dh*.5f);
    p.heading+=dh;p.path+=fabsf(ds);p.left+=dl;p.right+=dr;
    // Encoders report wheel rotation; traction loss affects the chassis only.
    float measuredRight=vr*.02f,measuredYaw=(measuredRight-dl)/100,measuredTravel=(dl+measuredRight)*.5f;
    estimated.x+=measuredTravel*cosf(estimated.heading+measuredYaw*.5f);
    estimated.y+=measuredTravel*sinf(estimated.heading+measuredYaw*.5f);
    estimated.heading+=measuredYaw;estimated.path+=fabsf(measuredTravel);
    estimated.left+=dl;estimated.right+=measuredRight;
    Observation o;o.sequence=tick;o.stamp=tick*20;o.pose=estimated;
    for(int sensor=0;sensor<3;++sensor) {
      float lateral=(1-sensor)*12;
      float x=p.x+40*cosf(p.heading)-lateral*sinf(p.heading);
      float y=p.y+40*sinf(p.heading)+lateral*cosf(p.heading);
      float nearest=10000;for(auto s:course) nearest=fminf(nearest,distance(x,y,s));
      if(nearest<=10) o.tape|=4>>sensor;
    }
    if(noisy&&tick%137==0) o.tape^=2;
    o.front={600,o.stamp,true};o.right={600,o.stamp,true};e.observe(o);
    Drive next=e.step(o.stamp);command=guardMotion(next,e.world,e.dynamics,o.stamp,200,80).allowed?next:Drive();
    if((!endpoint&&p.x>430&&fabsf(p.y-(sharp?150:200))<30)||(endpoint&&e.backtracking&&p.x<120&&angleDistance(p.heading,PI_F)<.5f)) { reached=true;break; }
    if(e.state==Explorer::FAULT||e.state==Explorer::NEED_CONFIRM) break;
  }
  if(!reached) fprintf(stderr,"replay failed: %s at %.1f, %.1f heading %.2f\n",e.reason,p.x,p.y,p.heading);
  assert(reached);
}

int main() { run(false,1);run(true,1);run(false,.98f);run(false,1,true);run(false,1,false,true); }
