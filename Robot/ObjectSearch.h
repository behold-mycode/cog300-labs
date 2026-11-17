#pragma once
#include "WallNavigator.h"
namespace object {
// Open-arena search. Front sonar cannot distinguish a target from a wall;
// the nearest reflecting object is the target. Start with pivot clearance.
class Search {
 public:
 enum State {IDLE,SAMPLE,TURN,ALIGN,VERIFY,APPROACH,DONE,BOUNDARY,FAULT} state=IDLE;
 const char* reason="OBJECT_IDLE";
 float bestRange=INFINITY,bestBearing=0;
 void stop(){state=IDLE;}
 void begin(const wall::Input&i){
  origin=i.pose.heading;index=0;bestRange=INFINITY;bestBearing=origin;
  sample(i);reason="OBJECT_SWEEPING";
 }
 wall::Speeds step(const wall::Config&c,const wall::Input&i,uint8_t tape){
  if(state==IDLE||state==DONE||state==BOUNDARY||state==FAULT)return {};
  if(tape){state=BOUNDARY;reason="OBJECT_ARENA_BOUNDARY";return {};}
  const auto&r=i.ranges[0];
  bool noReturn=r.noReturn&&(uint32_t)(i.now-r.stamp)<300;
  float distance=wall::corrected(c,i,0);
  if(distance<0&&!noReturn){
   if((uint32_t)(i.now-lastFresh)>1500)fail("OBJECT_RANGE_LOST");
   else reason="OBJECT_WAITING_FOR_RANGE";
   return {};
  }
  lastFresh=i.now;
  float speed=fmaxf(fabsf(i.leftSpeed),fabsf(i.rightSpeed));
  if(state==TURN||state==ALIGN){
   if(!noReturn&&distance+c.frontSensor<=c.radius()+c.margin){fail("OBJECT_PIVOT_BLOCKED");return {};}
   auto d=turn.step(c,i);
   if(turn.failed){fail("OBJECT_TURN_DRIFT");return {};}
   if(turn.done){bool aligned=state==ALIGN;sample(i);if(aligned){state=VERIFY;reason="OBJECT_VERIFYING_TARGET";}}
   return d;
  }
  if(state==SAMPLE||state==VERIFY){
   if(speed>=1){quietAt=i.now;return {};}
   if((uint32_t)(i.now-quietAt)<200)return {};
   if(r.sequence==sequence||(int32_t)(r.stamp-quietAt)<=0)return {};
   sequence=r.sequence;values[count++]=noReturn?INFINITY:distance;
   if(count<3)return {};
   float a=values[0],b=values[1],d=values[2];
   float median=fmaxf(fminf(a,b),fminf(fmaxf(a,b),d));
   if(state==VERIFY){
    if(!isfinite(median)||median>bestRange+fmaxf(100,bestRange*.3f)){fail("OBJECT_TARGET_LOST");return {};}
    state=APPROACH;reason="OBJECT_APPROACHING";return {};
   }
   if(median<bestRange){bestRange=median;bestBearing=i.pose.heading;}
   // 0 -> +180 degrees, then back through 0 to -180. Ten-degree samples.
   if(index==54){
    if(!isfinite(bestRange)){fail("OBJECT_NOT_FOUND");return {};}
    turn.begin(maze::wrap(bestBearing-i.pose.heading),i.pose);state=ALIGN;reason="OBJECT_ALIGNING";
   }else{
    ++index;float target=origin+(index<=18?index:36-index)*maze::PI_F/18;
    turn.begin(target-i.pose.heading,i.pose);state=TURN;reason="OBJECT_SWEEPING";
   }
   return {};
  }
  if(state==APPROACH){
   if(noReturn){fail("OBJECT_TARGET_LOST");return {};}
   float room=distance+c.frontSensor-c.front-200; // Stop 20 cm before target.
   if(room<=c.distanceTolerance){state=DONE;reason="OBJECT_REACHED";return {};}
   float stopping=speed*c.reaction+speed*speed/(2*c.deceleration);
   if(room<=stopping){reason="OBJECT_BRAKING";return {};}
   float ar=c.deceleration*c.reaction;
   float cap=2*c.deceleration*room/(sqrtf(ar*ar+2*c.deceleration*room)+ar);
   float v=fminf(c.cruise,fminf(cap,room*2));
   reason="OBJECT_APPROACHING";return {v,v};
  }
  return {};
 }
 private:
 wall::Turn turn;float origin=0,values[3]={};int index=0,count=0;
 uint32_t quietAt=0,sequence=0,lastFresh=0;
 void sample(const wall::Input&i){state=SAMPLE;quietAt=lastFresh=i.now;sequence=i.ranges[0].sequence;count=0;}
 void fail(const char*why){state=FAULT;reason=why;}
};
}
