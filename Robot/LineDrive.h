#pragma once
#include "Velocity.h"
// Open-floor line driving: full power by default. Keep the inner wheel at
// least 80% power for steering, rather than asking it to crawl below friction.
inline wall::Powers lineDrive(wall::Speeds d,int maximum){
 float peak=fmaxf(fabsf(d.left),fabsf(d.right));
 if(peak==0)return {};
 auto wheel=[&](float value){
  if(value==0)return 0;
  int power=(int)(maximum*fmaxf(.8f,fabsf(value)/peak));
  return value<0?-power:power;
 };
 return {wheel(d.left),wheel(d.right)};
}

namespace line {
// Pause between strong corrections; count powered time only after the motor
// bridge's direction-change interlock has actually enabled the wheels.
class PulseDrive {
 enum Phase { IDLE,WAIT,POWER,REST,RESUME } phase=IDLE;
 uint32_t since=0;
 bool powered=false;
 float startHeading=0;
 public:
 void reset(){phase=IDLE;powered=false;}
 void pause(uint32_t now){phase=REST;since=now;powered=false;}
 wall::Powers step(uint32_t now,wall::Speeds d,int maximum,bool precise,
                   int actualLeft,int actualRight,float heading){
  if(d.left==0&&d.right==0){reset();return {};}
  if(!precise){
   if(phase!=IDLE&&phase!=RESUME){phase=RESUME;since=now;}
   if(phase==RESUME&&(uint32_t)(now-since)<120)return {};
   reset();return lineDrive(d,maximum);
  }
  if(phase==IDLE||phase==RESUME){phase=WAIT;since=now;powered=false;}
  if(phase==WAIT||phase==REST){
   if((uint32_t)(now-since)<120)return {};
   phase=POWER;powered=false;
  }
  auto p=lineDrive(d,maximum);
  bool enabled=(p.left==0||actualLeft*p.left>0)&&(p.right==0||actualRight*p.right>0);
  if(!powered&&enabled){powered=true;since=now;startHeading=heading;}
  if(powered&&((uint32_t)(now-since)>=35||fabsf(heading-startHeading)>=.04f)){
   phase=REST;since=now;powered=false;return {};
  }
  return p;
 }
};
}
