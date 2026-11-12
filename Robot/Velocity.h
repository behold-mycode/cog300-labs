#pragma once
#include <math.h>
namespace wall {
inline float limit(float x,float lo,float hi){return fmaxf(lo,fminf(hi,x));}
struct Speeds { float left=0,right=0; };
struct Powers { int left=0,right=0; };
struct VelocityConfig {
 float fullLeft=0,fullRight=0,kp=.6f,ki=.3f,acceleration=150;
 bool valid()const{return isfinite(fullLeft)&&isfinite(fullRight)&&isfinite(kp)&&isfinite(ki)&&isfinite(acceleration)&&fullLeft>0&&fullRight>0&&kp>=0&&ki>=0&&acceleration>0;}
};
// Physical targets are mm/s, not PWM. Full-speed measurements are at PWM255.
// A zero target immediately coasts; braking distance is handled by the planner.
class Velocity {
 float integral[2]={},target[2]={};
 public:
 VelocityConfig config;
 void reset(){for(int n=0;n<2;++n)integral[n]=target[n]=0;}
 Powers step(float left,float right,float measuredL,float measuredR,float dt,int maximum){
  if(!config.valid()||!isfinite(dt)||dt<=0||dt>.3f){reset();return {};}
  return {wheel(0,left,measuredL,config.fullLeft,dt,maximum),wheel(1,right,measuredR,config.fullRight,dt,maximum)};
 }
 private:
 int wheel(int n,float want,float measured,float full,float dt,int maximum){
  if(!isfinite(want)||!isfinite(measured)||want==0){target[n]=integral[n]=0;return 0;}
  if(want*target[n]<0){target[n]=integral[n]=0;return 0;}
  target[n]+=limit(want-target[n],-config.acceleration*dt,config.acceleration*dt);
  float direction=want>0?1:-1,error=(target[n]-measured)*direction;
  float ff=fabsf(target[n])*255/full;
  float raw=ff+config.kp*error+integral[n];
  // Conditional integration avoids winding up against either PWM rail.
  if((raw<maximum||error<0)&&(raw>0||error>0))integral[n]=limit(integral[n]+config.ki*error*dt,-maximum,maximum);
  return (int)(direction*limit(ff+config.kp*error+integral[n],0,maximum));
 }
};
}
