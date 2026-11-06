#pragma once
#include "Odometry.h"
namespace maze {
struct Drive { int8_t left=0,right=0; };
// Measured completion only. These classes do not read clocks to estimate angle.
class Motion {
 public:
  enum Kind { IDLE, TRANSLATE, ROTATE } kind=IDLE;
  bool done=false;
  void cancel() { kind=IDLE;done=false; }
  void move(float mm,const Pose& pose) { kind=TRANSLATE;done=false;startL=pose.left;startR=pose.right;target=mm; }
  void turn(float radians,const Pose& pose) { kind=ROTATE;done=false;target=pose.heading+radians;direction=radians>=0?1:-1; }
  Drive step(const Pose& pose,float distanceTolerance,float angleTolerance) {
    if(kind==IDLE) return {};
    if(kind==TRANSLATE) {
      float progress=((pose.left-startL)+(pose.right-startR))*0.5f;
      if((target>=0 && progress>=target-distanceTolerance)||(target<0 && progress<=target+distanceTolerance)) {
        kind=IDLE;done=true;return {};
      }
      int8_t d=target>=0?1:-1;return {d,d};
    }
    if(direction*(target-pose.heading)<=angleTolerance) { kind=IDLE;done=true;return {}; }
    return {(int8_t)-direction,(int8_t)direction};
  }
 private: float target=0,startL=0,startR=0;int direction=1;
};
// Detect commanded-but-stalled wheels separately, including during translation.
class ProgressGuard {
 public:
  void reset() { leftActive=rightActive=false; }
  bool check(const Pose& p,Drive actual,uint32_t now,uint32_t deadline) {
    return wheel(p.left,actual.left,now,deadline,leftActive,lastL,timeL)&&
           wheel(p.right,actual.right,now,deadline,rightActive,lastR,timeR);
  }
 private:
  bool leftActive=false,rightActive=false;float lastL=0,lastR=0;uint32_t timeL=0,timeR=0;
  static bool wheel(float position,int command,uint32_t now,uint32_t deadline,bool& active,float& last,uint32_t& stamp) {
    if(!command) { active=false;return true; }
    if(!active || position!=last) { active=true;last=position;stamp=now; }
    return (uint32_t)(now-stamp)<deadline;
  }
};
}
