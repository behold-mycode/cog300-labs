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
