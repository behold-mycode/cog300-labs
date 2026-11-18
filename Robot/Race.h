#pragma once
#include "WallNavigator.h"
namespace race {
struct Config {
 float speed=0,sensorOffset=0,searchAngle=3.f;
 bool valid()const{return isfinite(speed)&&isfinite(sensorOffset)&&isfinite(searchAngle)&&speed>0&&speed<=300&&sensorOffset>0&&sensorOffset<1000&&searchAngle>=.5f&&searchAngle<=3.1f;}
};
class Button {
 bool armed=false,pressed=false,inhibit=false,pending=false;uint32_t down=0,up=0;
 public:
 enum Event{NONE,START,STOP};
 Event update(bool low,uint32_t now,bool moving){
  if(low){
   if(!pressed){pressed=true;down=now;pending=false;}
   if(moving&&!inhibit){inhibit=true;return STOP;}
   return NONE;
  }
  if(pressed){pressed=false;up=now;pending=armed&&!inhibit&&(uint32_t)(now-down)>=800;}
  if((uint32_t)(now-up)>=40){armed=true;inhibit=false;if(pending){pending=false;return START;}}
  return NONE;
 }
};
class Navigator {
 public:
 enum State{IDLE,TRACK,SETTLE,SWEEP_FIRST,SWEEP_SECOND,ALIGN,PROBE,ENTRY,WALL,FAULT,CENTRE_LINE}state=IDLE;
 const char*reason="RACE_IDLE";
 bool wallStage()const{return state==WALL;}
 void stop(){state=IDLE;}
 bool begin(const Config&r,const wall::Config&c,const wall::Input&i,uint8_t mask,bool enterWalls=true){
  wallCourse=enterWalls;
  if(!r.valid()||!c.valid()){reason="RACE_CALIBRATION_REQUIRED";return false;}
  if(mask==0){reason="RACE_START_ON_LINE";return false;}
  state=TRACK;lastTape=i.pose;heading=i.pose.heading;lastSide=1;pivotSide=0;onBand=false;reason="RACE_LINE";return true;
 }
 wall::Speeds step(const Config&r,const wall::Config&c,const wall::Input&original,uint8_t tape){
  if(state==IDLE||state==FAULT||state==WALL)return {};
  wall::Input i=original;
  // Completed no-return pulses are usable in the open line section, unlike a
  // missing/stuck echo. They still do not certify a collision-free swept area.
  for(auto &x:i.ranges)if(x.noReturn&&(uint32_t)(i.now-x.stamp)<300){x.valid=true;x.mm=4000;}
  float f=wall::corrected(c,i,0);if(wallCourse&&f<0){reason="RACE_FRONT_UNKNOWN";return {};}
  float v=fmaxf(fabsf(i.leftSpeed),fabsf(i.rightSpeed));
  float stop=v*c.reaction+v*v/(2*c.deceleration);
  bool turning=state==SWEEP_FIRST||state==SWEEP_SECOND||state==ALIGN||state==CENTRE_LINE;
  if(wallCourse&&!turning&&f+c.frontSensor<=c.front+c.margin+stop){reason="RACE_FRONT_BLOCKED";return {};}
  if(state==TRACK){
   if(tape==0){
    heading=lastTape.heading;pointX=lastTape.x+r.sensorOffset*cosf(heading);pointY=lastTape.y+r.sensorOffset*sinf(heading);
    pivotSide=0;probed=false;quietSince=i.now;state=SETTLE;reason="RACE_LINE_LOST_ASSESSING";return {};
   }
   if(tape&2){lastTape=i.pose;pivotSide=0;}
   // Finish an outer-sensor pivot until the centre reaches tape. At a tight
   // bend the opposite outer sensor can still see the incoming segment;
   // immediately obeying it would reverse the pivot on every control step.
   if(!pivotSide&&(tape==4||tape==1))pivotSide=tape==4?1:-1;
   if(pivotSide){
    lastSide=pivotSide;
    if(!turnClear(c,i)){reason="RACE_TURN_CLEARANCE_REQUIRED";return {};}
    reason="RACE_LINE_PIVOT";
    return guarded(c,i,{-pivotSide*c.turnSpeed,pivotSide*c.turnSpeed});
   }
   if(tape==4||tape==6)lastSide=1;else if(tape==1||tape==3)lastSide=-1;
   float direction=(tape==4||tape==6)?1:(tape==1||tape==3)?-1:0;
   // A broad stripe/intersection is crossed slowly; it is not a sensor fault.
   float speed=(tape==5||tape==7)?fminf(r.speed,c.turnSpeed):r.speed;
   reason="RACE_LINE";return guarded(c,i,{speed*(1-.6f*direction),speed*(1+.6f*direction)});
  }
  bool stopped=v<1;
  if(state==SETTLE){
   if(!stopped){quietSince=i.now;return {};}
   if((uint32_t)(i.now-quietSince)<250)return {};
   if(tape&2){state=TRACK;lastTape=i.pose;reason="RACE_LINE_RESTORED";return {};}
   turn.begin(lastSide*r.searchAngle,i.pose);state=SWEEP_FIRST;onBand=false;bandArmed=false;reason="RACE_SEARCHING_LINE";return {};
  }
  if(turning){
   if(!turnClear(c,i)){reason="RACE_TURN_CLEARANCE_REQUIRED";return {};}
   if(state==CENTRE_LINE){
    // Both tape edges were observed. Do not accept the first edge again.
    if((tape&2)&&fabsf(bandCentre-i.pose.heading)<=bandTolerance){
     state=SETTLE;quietSince=i.now;reason="RACE_SETTLING_ON_LINE";return {};
    }
    auto precise=c;precise.angleTolerance=bandTolerance;
    auto d=turn.step(precise,i);
    if(turn.failed){fail("RACE_TURN_DRIFT");return {};}
    if(turn.done){state=SETTLE;quietSince=i.now;}
    reason="RACE_CENTRING_ON_LINE";return d;
   }
   bool searching=state==SWEEP_FIRST||state==SWEEP_SECOND;
   if(searching){
    if(tape&2){if(bandArmed&&!onBand){onBand=true;bandStart=i.pose.heading;}}
    else {bandArmed=true;if(onBand&&centreOnBand(c,i))return {};}
   }
   auto d=turn.step(c,i);
   if(turn.failed){fail("RACE_TURN_DRIFT");return {};}
   if(!turn.done){reason="RACE_SEARCHING_LINE";return d;}
   // A stripe cut off by the sweep limit has no observed second edge.
   // Skip that partial band, including until it exits after reversing.
   onBand=false;bandArmed=false;
   if(state==SWEEP_FIRST){turn.begin(-lastSide*2*r.searchAngle,i.pose);state=SWEEP_SECOND;}
   else if(state==SWEEP_SECOND){turn.begin(maze::wrap(heading-i.pose.heading),i.pose);state=ALIGN;}
   else{state=probed?ENTRY:PROBE;quietSince=i.now;entrySamples=0;entrySequence=0;}
   return {};
  }
  if(state==PROBE){
   if(tape==2||tape==3||tape==6){state=SETTLE;quietSince=i.now;return {};}
   float remaining=(pointX-i.pose.x)*cosf(heading)+(pointY-i.pose.y)*sinf(heading);
   if(remaining<=c.distanceTolerance){probed=true;state=SETTLE;quietSince=i.now;return {};}
   if(remaining>r.sensorOffset+c.margin){fail("RACE_TAPE_POINT_UNREACHABLE");return {};}
   if(remaining<=v*c.reaction+v*v/(2*c.deceleration)){reason="RACE_PROBE_BRAKING";return {};}
   float speed=fminf(r.speed,remaining*2);
   reason="RACE_PROBING_LAST_TAPE_POINT";return guarded(c,i,{speed,speed});
  }
  if(state==ENTRY){
   if(!wallCourse){fail("LINE_END_OR_LOST");return {};}
   if(tape){state=SETTLE;quietSince=i.now;return {};}
   if(!stopped){quietSince=i.now;return {};}
   if((uint32_t)(i.now-quietSince)<250)return {};
   const auto&left=original.ranges[2];const auto&right=original.ranges[1];
   float width=left.mm+right.mm+2*c.sideY;
   bool corridor=wall::fresh(left,i.now)&&wall::fresh(right,i.now)&&width>2*(c.halfWidth+c.margin)&&left.mm<wall::SIDE_RANGE_MM&&right.mm<wall::SIDE_RANGE_MM;
   if(corridor&&entrySequence!=left.sequence){entrySequence=left.sequence;++entrySamples;}
   if(!corridor)entrySamples=0;
   if(entrySamples>=3){state=WALL;reason="RACE_ENTERING_WALL_MAZE";return {};}
   if((uint32_t)(i.now-quietSince)>3000){fail("RACE_TRANSITION_UNCONFIRMED");return {};}
   reason="RACE_VERIFYING_MAZE_ENTRANCE";
  }
  return {};
 }
 private:
 maze::Pose lastTape;float heading=0,pointX=0,pointY=0;int lastSide=1,pivotSide=0,entrySamples=0;bool probed=false;
 bool wallCourse=true,onBand=false,bandArmed=false;
 float bandStart=0,bandCentre=0,bandTolerance=0;
 uint32_t quietSince=0,entrySequence=0;wall::Turn turn;
 bool centreOnBand(const wall::Config&c,const wall::Input&i){
  onBand=false;bandCentre=(bandStart+i.pose.heading)*.5f;
  // Firmware distanceTolerance is 1.5 encoder edges. Half a heading step
  // may separate the band midpoint from the closest attainable wheel count.
  float halfHeadingStep=c.distanceTolerance/(3*c.track);
  bandTolerance=fmaxf(halfHeadingStep+1e-6f,fminf(c.angleTolerance,fabsf(i.pose.heading-bandStart)*.25f));
  // Reject the incoming stripe using the whole detected band, not its edge.
  if(maze::angleDistance(bandCentre,heading+maze::PI_F)<=.1f+c.angleTolerance)return false;
  if(state==SWEEP_SECOND)lastSide=-lastSide;
  turn.begin(bandCentre-i.pose.heading,i.pose);state=CENTRE_LINE;
  reason="RACE_CENTRING_ON_LINE";return true;
 }
 bool turnClear(const wall::Config&c,const wall::Input&i)const{
  if(!wallCourse)return true; // Explicit open-floor line mode does not use sonar.
  // The line section is an open floor area. There is no rear sensor; a path
  // length threshold cannot certify rear clearance and used to deadlock bends.
  for(int n=0;n<(wallCourse?3:1);++n){
   float d=wall::corrected(c,i,n);
   if(d<0||d+(n==0?c.frontSensor:c.sideY)<=c.radius()+c.margin)return false;
  }
  return true;
 }
 void fail(const char*why){state=FAULT;reason=why;}
 wall::Speeds guarded(const wall::Config&c,const wall::Input&i,wall::Speeds d){
  if(!wallCourse)return d;
  bool curve=fabsf(d.left-d.right)>1||fabsf(i.leftSpeed-i.rightSpeed)>1;
  float v=fmaxf(fabsf(i.leftSpeed),fabsf(i.rightSpeed));
  float momentum=v*c.reaction+v*v/(2*c.deceleration);
  float front=wall::corrected(c,i,0);
  if(front<0){reason="RACE_FRONT_UNKNOWN";return {};}
  float room=front+c.frontSensor-c.front-c.margin;
  for(int n=1;wallCourse&&n<3;++n){
   float side=wall::corrected(c,i,n);
   float sideRoom=side+c.sideY-(curve?c.radius():c.halfWidth)-c.margin;
   if(side<0||sideRoom<=0||(curve&&sideRoom<=momentum)){reason="RACE_SIDE_CLEARANCE_REQUIRED";return {};}
   if(curve)room=fminf(room,sideRoom);
  }
  if(room<=momentum){reason="RACE_BRAKING";return {};}
  // Solve v*reaction + v²/(2*deceleration) <= available clearance.
  // Scale both wheels together so slowing does not change the requested arc.
  float ar=c.deceleration*c.reaction;
  float cap=2*c.deceleration*room/(sqrtf(ar*ar+2*c.deceleration*room)+ar);
  float requested=fmaxf(fabsf(d.left),fabsf(d.right));
  if(requested>cap){float scale=cap/requested;d.left*=scale;d.right*=scale;reason="RACE_SLOWING_FOR_CLEARANCE";}
  return d;
 }
};
}
