#pragma once
#include "Motion.h"
namespace maze {
struct RangeObservation {
  float mm=-1;
  uint32_t stamp=0;
  bool valid=false; // No return is unknown, not a free-space measurement.
};
struct Observation {
  uint32_t sequence=0,stamp=0;
  Pose pose;
  uint8_t tape=0;
  RangeObservation front,right;
};
// Parameters are explicit engineering priors, not fitted probabilities.
struct SensorModel {
  float hit=.85f,falsePositive=.15f,change=.12f;
  bool valid() const { return hit>.5f&&hit<1&&falsePositive>0&&falsePositive<.5f&&change>0&&change<1; }
};
class WorldModel {
 public:
  enum Hypothesis { LEFT,CENTRE,RIGHT,WIDE,MISSING,COUNT };
  float belief[COUNT]={.2f,.2f,.2f,.2f,.2f};
  SensorModel model;
  Observation latest;
  float leftSpeed=0,rightSpeed=0,positionUncertainty=0,headingUncertainty=0;
  bool initialized=false,fresh=false,informative=false;
  uint32_t updates=0;
  void reset() { SensorModel saved=model;*this=WorldModel();model=saved; }
  bool update(const Observation& o,float resolution,float offset) {
    fresh=informative=false;
    if(initialized && ((int32_t)(o.sequence-latest.sequence)<=0 || (int32_t)(o.stamp-latest.stamp)<=0)) return false;
    if(!isfinite(o.pose.left)||!isfinite(o.pose.right)||!isfinite(o.pose.heading)||o.tape>7) return false;
    float travel=0,dh=0;
    if(initialized) {
      float dt=(uint32_t)(o.stamp-latest.stamp)*.001f;
      float dl=o.pose.left-latest.pose.left,dr=o.pose.right-latest.pose.right;
      leftSpeed=dl/dt;rightSpeed=dr/dt;travel=(fabsf(dl)+fabsf(dr))*.5f;
      dh=o.pose.heading-latest.pose.heading;
      // Conservative uncertainty growth, not a claim that slip is observable.
      positionUncertainty+=travel*.05f;
      headingUncertainty+=fabsf(dh)*.05f;
    }
    // Unchanged stationary samples are correlated. They update freshness and
    // speed, but cannot repeatedly multiply the same likelihood into certainty.
    informative=!initialized||o.tape!=evidenceMask ||
      fabsf(o.pose.left-evidencePose.left)+fabsf(o.pose.right-evidencePose.right)>=2*resolution;
    if(informative && model.valid()) {
      float predicted[COUNT],sum=0;
      for(int i=0;i<COUNT;++i) predicted[i]=(1-model.change)*belief[i]+model.change/COUNT;
      // Positive yaw sweeps a forward-mounted sensor left: tape shifts right
      // relative to the sensor row. Transfer mass instead of relabelling truth.
      float shift=fminf(.8f,fabsf(dh)*offset/fmaxf(resolution,offset*.25f));
      if(dh>0) { float a=predicted[LEFT]*shift,b=predicted[CENTRE]*shift;
        predicted[LEFT]-=a;predicted[CENTRE]+=a-b;predicted[RIGHT]+=b; }
      else { float a=predicted[RIGHT]*shift,b=predicted[CENTRE]*shift;
        predicted[RIGHT]-=a;predicted[CENTRE]+=a-b;predicted[LEFT]+=b; }
      for(int i=0;i<COUNT;++i) {
        // Left/right each marginalize two possible binary patterns.
        float likelihood=i==LEFT?.5f*(emission(o.tape,4)+emission(o.tape,6)):
          i==RIGHT?.5f*(emission(o.tape,1)+emission(o.tape,3)):
          i==WIDE?.5f*(emission(o.tape,5)+emission(o.tape,7)):
          emission(o.tape,i==CENTRE?2:0);
        belief[i]=predicted[i]*likelihood;sum+=belief[i];
      }
      for(float &v:belief) v/=sum;
      evidencePose=o.pose;evidenceMask=o.tape;++updates;
    }
    latest=o;initialized=true;fresh=true;return true;
  }
  float lateral() const { return belief[LEFT]-belief[RIGHT]; }
  float speed() const { return fmaxf(fabsf(leftSpeed),fabsf(rightSpeed)); }
 private:
  Pose evidencePose;uint8_t evidenceMask=0;
  float emission(uint8_t measured,uint8_t expected) const {
    float p=1;
    for(int bit=1;bit<=4;bit<<=1) {
      float on=expected&bit?model.hit:model.falsePositive;
      p*=measured&bit?on:1-on;
    }
    return p;
  }
};
// Explicit measured braking inputs. Unconfigured dynamics cannot enable motion.
struct Dynamics {
  float deceleration=0,reactionSeconds=0,margin=0;
  bool valid() const { return isfinite(deceleration)&&isfinite(reactionSeconds)&&isfinite(margin)&&deceleration>0&&reactionSeconds>0&&margin>0; }
};
struct GuardResult { bool allowed=false;const char* reason="DYNAMICS_REQUIRED"; };
inline GuardResult guardMotion(Drive d,const WorldModel& w,const Dynamics& dynamics,
    uint32_t now,float forwardLimit,float turnLimit,uint32_t maxAge=350) {
  if(!d.left&&!d.right) return {true,"STOPPED"};
  if(!dynamics.valid()) return {};
  if(!w.initialized||(uint32_t)(now-w.latest.stamp)>=100) return {false,"OBSERVATION_STALE"};
  bool rotate=d.left!=d.right;
  const auto& f=w.latest.front;const auto& r=w.latest.right;
  if(!f.valid||(uint32_t)(now-f.stamp)>=maxAge) return {false,"FRONT_UNKNOWN"};
  if(rotate&&(!r.valid||(uint32_t)(now-r.stamp)>=maxAge)) return {false,"RIGHT_UNKNOWN"};
  float age=(uint32_t)(now-f.stamp)*.001f;
  float v=w.speed();
  float stop=v*(dynamics.reactionSeconds+age)+v*v/(2*dynamics.deceleration)+dynamics.margin;
  if(f.mm<=fmaxf(rotate?turnLimit:forwardLimit,stop)) return {false,"FRONT_BLOCKED"};
  if(rotate) {
    float sideStop=v*(dynamics.reactionSeconds+(uint32_t)(now-r.stamp)*.001f)+v*v/(2*dynamics.deceleration)+dynamics.margin;
    if(r.mm<=fmaxf(turnLimit,sideStop)) return {false,"RIGHT_BLOCKED"};
  }
  // Autonomous reverse translation needs rear sensing; pivots are distinct.
  if(d.left<0&&d.right<0) return {false,"REAR_UNOBSERVED"};
  return {true,"CLEAR"};
}
}
