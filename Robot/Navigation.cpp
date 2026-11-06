#include "Robot.h"
#include <math.h>

Mode mode=MANUAL;
NavigationConfig navigationConfig;
maze::Explorer explorer;
const char* navigationState="STOPPED";
static maze::ProgressGuard progress;
static bool endArmed=false;
static bool wallTurning=false;
static maze::Motion wallMotion;
static void drive(maze::Drive d) { requestMotor(d.left*settings.power,d.right*settings.power); }
static void hold(const char* reason) { stopMotor();navigationState=reason; }
static bool feedbackReady() { return encodersEnabled()&&odometry.configured&&odometry.sampled; }
const char* modeName() {
  switch(mode) {
    case FOLLOW:return "FOLLOW";case WALL:return "WALL";case LINE:return "LINE";
    case MAZE:return "MAZE";case STOP_ON_TAPE:return "STOP_ON_TAPE";case STOP_AT_END:return "STOP_AT_END";
    default:return "MANUAL";
  }
}
void cancelNavigation() {
  explorer.pause();mode=MANUAL;progress.reset();wallMotion.cancel();wallTurning=false;
  navigationState="STOPPED";
}
void stopRobot() { cancelNavigation();stopMotor(); }
void startNavigation(Mode selected) {
  stopRobot();
  if(selected==MAZE) {
    if(!feedbackReady()) { navigationState="ENCODERS_REQUIRED";return; }
    if(lineTapeLevel<0) { navigationState="LINE_NOT_CALIBRATED";return; }
    if(!explorer.geometry.valid()) { navigationState="GEOMETRY_REQUIRED";return; }
    if(!explorer.graph.nodeCount && lineMask()!=2) { navigationState="START_ON_CENTRE_TAPE";return; }
    if(!explorer.begin(odometry.pose)) { navigationState=explorer.reason;return; }
  } else if(selected!=MANUAL) explorer.invalidate();
  if((selected==LINE||selected==STOP_ON_TAPE||selected==STOP_AT_END)&&lineTapeLevel<0) {
    navigationState="LINE_NOT_CALIBRATED";return;
  }
  endArmed=lineMask()!=0;mode=selected;navigationState="WAITING_FOR_SENSOR";
}
// Obstacle observation constrains all autonomous commands. No competing controller
// can subtract this stop. A hold does not mark a graph edge explored or a goal found.
static bool frontClear(bool turning) {
  SensorReading front=sensorReading(0);
  if(turning && explorer.geometry.valid()) {
    SensorReading right=sensorReading(1);
    if(right.status!=RANGE_VALID && right.status!=RANGE_NO_RETURN) { hold("RIGHT_INVALID");return false; }
    if(right.status==RANGE_VALID && right.cm<=explorer.geometry.turnClearance/10.0f) { hold("RIGHT_BLOCKED");return false; }
  }
  if(front.status==RANGE_NO_RETURN) return true;
  if(front.status!=RANGE_VALID||front.cm<0) { hold("FRONT_INVALID");return false; }
  float required=navigationConfig.frontStopCm;
  if(turning && explorer.geometry.valid()) required=explorer.geometry.turnClearance/10.0f;
  if(front.cm<=required) { hold("FRONT_BLOCKED");return false; }
  return true;
}
static void basicLine(uint8_t mask) {
  if(mask==2) { drive({1,1});navigationState="LINE_TRACKING"; }
  else if(mask==6||mask==4) { drive({0,1});navigationState="LINE_LEFT"; }
  else if(mask==3||mask==1) { drive({1,0});navigationState="LINE_RIGHT"; }
  else hold(mask==0?"LINE_LOST_USE_MAZE":"JUNCTION_USE_MAZE");
}
static void followWall() {
  SensorReading right=sensorReading(1),front=sensorReading(0);
  if(!wallTurning && front.status==RANGE_VALID && front.cm<=navigationConfig.frontStopCm) {
    if(!feedbackReady()||!explorer.geometry.valid()) { hold("ENCODERS_REQUIRED_FOR_TURN");return; }
    // Rotate only until the front clears. A measured half-turn is a failure
    // bound, not an assumed corner angle or a successful movement target.
    wallMotion.turn(maze::PI_F,odometry.pose);wallTurning=true;
  }
  if(wallTurning) {
    if(!frontClear(true)) return;
    if(front.status==RANGE_VALID && front.cm>navigationConfig.frontStopCm+navigationConfig.wallBandCm) {
      wallMotion.cancel();wallTurning=false;hold("WALL_FRONT_CLEARED");return;
    }
    maze::Drive d=wallMotion.step(odometry.pose,explorer.geometry.distanceTolerance,explorer.geometry.angleTolerance);
    if(wallMotion.done) { stopRobot();navigationState="WALL_TURN_UNRESOLVED";return; }
    drive(d);navigationState="WALL_MEASURED_LEFT";return;
  }
  // Losing the side wall is not evidence that it is safe to turn right.
  if(right.status!=RANGE_VALID||right.cm<0) { hold("WALL_LOST");return; }
  if(!frontClear(false)) return;
  float error=right.cm-settings.wallCm;
  if(error>navigationConfig.wallBandCm) { drive({1,0});navigationState="WALL_RIGHT"; }
  else if(error<-navigationConfig.wallBandCm) { drive({0,1});navigationState="WALL_LEFT"; }
  else { drive({1,1});navigationState="WALL_TRACKING"; }
}
void serviceNavigation() {
  if(mode==MANUAL) return;
  if(mode==MAZE && !feedbackReady()) { hold("ENCODERS_REQUIRED");return; }
  if(feedbackReady() && !progress.check(odometry.pose,
      {(int8_t)(motorLeft>0?1:motorLeft<0?-1:0),(int8_t)(motorRight>0?1:motorRight<0?-1:0)},
      millis(),navigationConfig.stallMs)) {
    explorer.fail("ENCODER_STALL");stopRobot();navigationState="ENCODER_STALL";return;
  }
  if(mode==WALL) { followWall();return; }
  if(mode==FOLLOW) {
    float front=sensorDistanceCm(0);
    if(front<0) { hold("FRONT_INVALID");return; }
    if(front>FOLLOW_MAX_CM) { hold("TARGET_LOST");return; }
    float error=front-settings.followCm;
    if(fabsf(error)<=DEADBAND_CM) { hold("AT_DISTANCE");return; }
    int power=constrain((int)(fabsf(error)*FOLLOW_KP),settings.minimum,settings.power);
    if(error<0) power=-power;
    requestMotor(power,power);navigationState=error>0?"APPROACHING":"BACKING_AWAY";return;
  }
  if(mode!=MAZE && !frontClear(false)) return;
  uint8_t mask=lineMask();
  if(lineTapeLevel<0) { hold("LINE_NOT_CALIBRATED");return; }
  if(mode==STOP_ON_TAPE) {
    if(mask) { stopRobot();navigationState="TAPE_REACHED"; }
    else { drive({1,1});navigationState="SEEKING_TAPE"; }
  } else if(mode==STOP_AT_END) {
    if(!endArmed) { hold("START_ON_TAPE");return; }
    if(!mask) { stopRobot();navigationState="TAPE_ENDED"; }
    else basicLine(mask);
  } else if(mode==LINE) basicLine(mask);
  else if(mode==MAZE) {
    uint32_t now=millis();
    maze::Drive requested=explorer.step(now);
    maze::GuardResult guard=maze::guardMotion(requested,explorer.world,explorer.dynamics,now,
      navigationConfig.frontStopCm*10,explorer.geometry.turnClearance);
    navigationState=explorer.reason;
    if(guard.allowed) drive(requested);
    else { hold(guard.reason);explorer.interruptObservation(); }
    if(explorer.state==maze::Explorer::FAULT||explorer.state==maze::Explorer::COMPLETE||
       explorer.state==maze::Explorer::NEED_CONFIRM) {
      const char* reason=explorer.reason;stopRobot();navigationState=reason;
    }
  }
}

static bool observationTrace=false;
void setObservationTrace(bool enabled) { observationTrace=enabled; }
void serviceObservation() {
  static uint32_t last=0,sequence=0;
  uint32_t now=millis();if(sequence&&(uint32_t)(now-last)<20) return;last=now;
  SensorReading f=sensorReading(0),r=sensorReading(1);
  maze::Observation o;o.sequence=++sequence;o.stamp=now;o.pose=odometry.pose;o.tape=lineMask();
  o.front={f.cm*10,f.sampledMs,f.status==RANGE_VALID};o.right={r.cm*10,r.sampledMs,r.status==RANGE_VALID};
  if(odometry.configured&&lineTapeLevel>=0) explorer.observe(o);
  if(!observationTrace) return;
  Serial.print(F("OBS,"));Serial.print(now);Serial.print(',');Serial.print(sequence);Serial.print(',');Serial.print((int)o.tape);
  Serial.print(',');Serial.print(o.pose.x);Serial.print(',');Serial.print(o.pose.y);Serial.print(',');Serial.print(o.pose.heading);
  Serial.print(',');Serial.print(o.pose.left);Serial.print(',');Serial.print(o.pose.right);
  Serial.print(',');Serial.print(o.front.valid?o.front.mm:-1);Serial.print(',');Serial.print(o.right.valid?o.right.mm:-1);
  Serial.print(',');Serial.print((uint32_t)(now-f.sampledMs));Serial.print(',');Serial.print((uint32_t)(now-r.sampledMs));
  Serial.print(',');Serial.print(motorLeft);Serial.print(',');Serial.print(motorRight);Serial.print(',');Serial.println(odometry.configured?1:0);
}
