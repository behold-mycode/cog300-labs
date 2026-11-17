#pragma once
#include "HardwareProfile.h"
#include "WallNavigator.h"
namespace commissioning {
struct Input {uint32_t now=0;float left=0,right=0;uint8_t rawTape=0;wall::Ray ranges[3];};
struct Result {int tape=-1;float fullLeft=0,fullRight=0,deceleration=0,cruise=0,corridorWidth=0;};
class Setup {
 public:
 enum State{IDLE,CHECK,RAMP,MEASURE,COAST,DONE,FAULT}state=IDLE;
 Result result;const char*reason="SETUP_IDLE";
 bool done()const{return state==DONE;}bool failed()const{return state==FAULT;}
 void cancel(){state=IDLE;last={};}
 bool begin(const hardware::Profile&p,const Input&i){
  if(!p.valid()){fail("HARDWARE_DIMENSIONS_REQUIRED");return false;}
  profile=p;result=Result();state=CHECK;reason="SETUP_CHECKING_SENSORS";
  since=lastAt=quietAt=i.now;originL=sampleL=lastL=i.left;originR=sampleR=lastR=i.right;
  raw=i.rawTape;count=0;lastSequence=0;exposureL=exposureR=0;last={};return true;
 }
 wall::Powers step(const Input&source){
  Input i=source;
  for(auto&r:i.ranges)if(r.noReturn&&(uint32_t)(i.now-r.stamp)<300){r.valid=true;r.mm=4000;}
  if(state==IDLE||state==DONE||state==FAULT)return {};
  float dt=(uint32_t)(i.now-lastAt)*.001f;
  if(dt>.3f){fail("SETUP_LOOP_DELAY");return {};}
  for(int n=0;n<(profile.leftSonarInstalled?3:2);++n)if(!wall::fresh(i.ranges[n],i.now)){fail("SETUP_SENSOR_UNKNOWN");return {};}
  float dl=i.left-originL,dr=i.right-originR;
  if(!isfinite(dl)||!isfinite(dr)||dl<-profile.leftMmPerEdge||dr<-profile.rightMmPerEdge){fail("SETUP_ENCODER_DIRECTION");return {};}
  if(fabsf(dl-dr)>profile.track*.18f){fail("SETUP_WHEEL_IMBALANCE");return {};}
  float radius=hypotf(fmaxf(profile.front,profile.rear),profile.halfWidth);
  float front=i.ranges[0].mm+profile.frontSonar-profile.front;
  if(front<350||i.ranges[1].mm+profile.sideSonarY<radius+30||(profile.leftSonarInstalled&&i.ranges[2].mm+profile.sideSonarY<radius+30)){fail("SETUP_NEEDS_CLEAR_STRAIGHT_SPACE");return {};}
  if(fmaxf(dl,dr)>300){fail("SETUP_TRAVEL_LIMIT");return {};}
  if(state==MEASURE){exposureL+=last.left*dt;exposureR+=last.right*dt;}
  lastAt=i.now;
  if(state==CHECK){
   if(front<650){fail("SETUP_NEEDS_CLEAR_STRAIGHT_SPACE");return {};}
   if(i.rawTape!=raw){raw=i.rawTape;since=i.now;count=0;}
   if(i.ranges[0].sequence!=lastSequence){lastSequence=i.ranges[0].sequence;++count;}
   if((uint32_t)(i.now-since)<400||count<3)return {};
   // Initial placement: centre on narrow tape (010/101) or all on bare floor.
   if(raw==2||raw==0)result.tape=1;
   else if(raw==5||raw==7)result.tape=0;
   else{fail("SETUP_CENTRE_LINE_OR_BARE_FLOOR");return {};}
   result.corridorWidth=profile.leftSonarInstalled?i.ranges[1].mm+i.ranges[2].mm+2*profile.sideSonarY:0;
   state=RAMP;since=i.now;reason="SETUP_MEASURING_WHEELS";
  }
  if(state==RAMP){
   uint32_t elapsed=i.now-since;
   if(elapsed>=800){
    if(dl<5*profile.leftMmPerEdge||dr<5*profile.rightMmPerEdge){fail("SETUP_ENCODERS_NOT_MOVING");return {};}
    state=MEASURE;since=i.now;sampleL=i.left;sampleR=i.right;exposureL=exposureR=0;
   }
   int power=(int)fminf(255,64+16*(elapsed/40));
   return balance(power,dl-dr);
  }
  if(state==MEASURE){
   if((uint32_t)(i.now-since)>=400){
    float l=i.left-sampleL,r=i.right-sampleR,seconds=(i.now-since)*.001f;
    if(l<5*profile.leftMmPerEdge||r<5*profile.rightMmPerEdge||exposureL<=0||exposureR<=0){fail("SETUP_ENCODERS_NOT_MOVING");return {};}
    result.fullLeft=l*255/exposureL;result.fullRight=r*255/exposureR;
    speedL=fmaxf(0,l-2*profile.leftMmPerEdge)/seconds;speedR=fmaxf(0,r-2*profile.rightMmPerEdge)/seconds;
    state=COAST;since=quietAt=i.now;lastL=sampleL=i.left;lastR=sampleR=i.right;reason="SETUP_MEASURING_COAST";last={};return {};
   }
   return balance(255,dl-dr);
  }
  if(state==COAST){
   if(i.left!=lastL||i.right!=lastR){quietAt=i.now;lastL=i.left;lastR=i.right;}
   if((uint32_t)(i.now-since)>3000){fail("SETUP_COAST_UNRESOLVED");return {};}
   if((uint32_t)(i.now-quietAt)>=500){
    float a=speedL*speedL/(2*(fabsf(i.left-sampleL)+3*profile.leftMmPerEdge));
    float b=speedR*speedR/(2*(fabsf(i.right-sampleR)+3*profile.rightMmPerEdge));
    result.deceleration=.5f*fminf(a,b); // Deliberately below the measured coast estimate.
    result.cruise=fminf(80,.4f*fminf(result.fullLeft,result.fullRight));
    if(result.deceleration<5||result.cruise<10){fail("SETUP_MOTION_UNRELIABLE");return {};}
    state=DONE;reason="SETUP_COMPLETE";
   }
  }
  return {};
 }
 private:
 hardware::Profile profile;wall::Powers last;
 uint32_t since=0,lastAt=0,quietAt=0,lastSequence=0;int count=0;uint8_t raw=0;
 float originL=0,originR=0,sampleL=0,sampleR=0,lastL=0,lastR=0,exposureL=0,exposureR=0,speedL=0,speedR=0;
 void fail(const char*why){state=FAULT;reason=why;last={};}
 wall::Powers balance(int pwm,float error){
  float correction=wall::limit(error*2,-30,30);
  last={(int)(pwm-fmaxf(0,correction)),(int)(pwm+fminf(0,correction))};return last;
 }
};
}
