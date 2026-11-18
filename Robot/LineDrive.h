#pragma once
#include "Velocity.h"
// Open-floor line driving: slightly reduced straight power. Keep the inner wheel at
// least 90% power for steering, rather than asking it to crawl below friction.
inline wall::Powers lineDrive(wall::Speeds d,int maximum){
 float peak=fmaxf(fabsf(d.left),fabsf(d.right));
 if(peak==0)return {};
 if(d.left>0&&d.right>0&&fabsf(d.left-d.right)<.01f){
  int straight=maximum<240?maximum:240;
  return {straight,straight};
 }
 auto wheel=[&](float value){
  if(value==0)return 0;
  int power=(int)(maximum*fmaxf(230.0f/255.0f,fabsf(value)/peak));
  return value<0?-power:power;
 };
 return {wheel(d.left),wheel(d.right)};
}
