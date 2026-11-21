#include "Robot.h"
#include <math.h>
#include "LineWheelControl.h"
static bool batteryLinePending=false;
static uint32_t batteryLineArmedAt=0;
static line::Follower lineFollower;
static line::WheelControl lineWheels;
static line::Input lineInput(){
  line::Input i{millis(),odometry.pose,lineMask()};
  i.appliedLeft=motorLeft;i.appliedRight=motorRight;lineWheels.observe(i);return i;
}

void beginRaceLine(){
  lineWheels.reset();
  lineFollower.begin(lineInput());
}
void serviceRaceLine(wall::Input input,uint8_t tape){
  line::Input i{input.now,input.pose,tape};
  i.appliedLeft=motorLeft;i.appliedRight=motorRight;lineWheels.observe(i);
  auto target=lineFollower.step(i);
  for(auto& range:input.ranges){
    if(range.noReturn&&(uint32_t)(input.now-range.stamp)<300){range.valid=true;range.mm=4000;}
  }
  raceNavigator.reason=lineFollower.reason;
  auto limited=raceNavigator.guarded(wallNavigator.config,input,{target.left,target.right});
  auto pwm=lineWheels.step({limited.left,limited.right},i,settings.power);
  navigationState=raceNavigator.reason;requestMotor(pwm.left,pwm.right);
}

Mode mode=MANUAL;
NavigationConfig navigationConfig;
maze::Explorer explorer;
static object::Search objectSearch;
const char* navigationState="STOPPED";
static maze::ProgressGuard progress;
static bool wallTurning=false;
static maze::Motion wallMotion;
static void drive(maze::Drive d) { requestMotor(d.left*settings.power,d.right*settings.power); }
static void hold(const char* reason) { stopMotor();navigationState=reason; }
static bool feedbackReady() { return encodersEnabled()&&odometry.configured&&odometry.sampled; }
const char* modeName() {
  switch(mode) {
    case CALIBRATE:return "CALIBRATE";case OBJECT:return "OBJECT";case RACE:return "RACE";case WALL_MAZE:return "WALL_MAZE";case FOLLOW:return "FOLLOW";case WALL:return "WALL";case LINE:return "LINE";
    case MAZE:return "MAZE";case STOP_ON_TAPE:return "STOP_ON_TAPE";case STOP_AT_END:return "STOP_AT_END";
    default:return "MANUAL";
  }
}
void cancelNavigation() {
  batteryLinePending=false;lineFollower.stop();lineWheels.reset();cancelAutomaticSetup();raceNavigator.stop();objectSearch.stop();standaloneActive=false;explorer.pause();pauseWall();mode=MANUAL;progress.reset();wallMotion.cancel();wallTurning=false;
  navigationState="STOPPED";
}
void stopRobot() { cancelNavigation();stopMotor(); }
void startNavigation(Mode selected) {
  stopRobot();
  if(selected!=WALL_MAZE&&selected!=RACE&&selected!=MANUAL)wallNavigator.invalidate();
  if(selected==CALIBRATE){
    if(beginAutomaticSetup(CALIBRATE))mode=CALIBRATE;
    return;
  }
  if(selected==OBJECT){
    if(lineTapeLevel<0||!prepareWallCalibration(false)){
      navigationState="CALIBRATE_WHEELS_FIRST";
      return;
    }
    resetWallFeedback();objectSearch.begin(motionInput());
  }
  if(selected==LINE||selected==STOP_AT_END||selected==STOP_ON_TAPE){
    if(lineTapeLevel<0){navigationState="LINE_NOT_CALIBRATED";return;}
    if(!feedbackReady()){navigationState="ENCODERS_REQUIRED";return;}
    if(selected!=STOP_ON_TAPE&&!lineFollower.begin(lineInput())){
      navigationState=lineFollower.reason;return;
    }
  }
  if(selected==RACE){if(!startRace())return;}
  else if(selected==WALL_MAZE){if(!startWall())return;}
  else if(selected==MAZE) {
    if(wallNavigator.ownsGraph){navigationState="RESET_WALL_MAP_FIRST";return;}
    if(!feedbackReady()) { navigationState="ENCODERS_REQUIRED";return; }
    if(lineTapeLevel<0) { navigationState="LINE_NOT_CALIBRATED";return; }
    if(!explorer.geometry.valid()) { navigationState="GEOMETRY_REQUIRED";return; }
    if(!explorer.graph.nodeCount && lineMask()!=2) { navigationState="START_ON_CENTRE_TAPE";return; }
    if(!explorer.begin(odometry.pose)) { navigationState=explorer.reason;return; }
  } else if(selected!=MANUAL) explorer.invalidate();
  mode=selected;navigationState="WAITING_FOR_SENSOR";
  if(ROBOT_LINE_ONLY&&selected==LINE){
    standaloneActive=true;controlSession=false;controlExpired=false;
  }
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
  if(automaticSetupActive()){serviceAutomaticSetup();return;}
  if((mode==MAZE||mode==WALL_MAZE||mode==RACE||mode==LINE||mode==OBJECT||mode==STOP_AT_END||mode==STOP_ON_TAPE) && !feedbackReady()) { hold("ENCODERS_REQUIRED");return; }
  if(mode!=LINE && !(mode==RACE && !raceNavigator.wallStage()) && feedbackReady() && !progress.check(odometry.pose,
      {(int8_t)(motorLeft>0?1:motorLeft<0?-1:0),(int8_t)(motorRight>0?1:motorRight<0?-1:0)},
      millis(),navigationConfig.stallMs)) {
    if(mode==WALL_MAZE)wallNavigator.fail("ENCODER_STALL");else explorer.fail("ENCODER_STALL");stopRobot();navigationState="ENCODER_STALL";return;
  }
  if(mode==OBJECT){
    driveVelocity(objectSearch.step(wallNavigator.config,motionInput(),lineMask()));navigationState=objectSearch.reason;
    if(objectSearch.state==object::Search::FAULT||objectSearch.state==object::Search::DONE||objectSearch.state==object::Search::BOUNDARY){
      const char*why=objectSearch.reason;stopRobot();navigationState=why;
    }
    return;
  }
  if(mode==STOP_ON_TAPE){
    if(lineMask()){stopRobot();navigationState="TAPE_REACHED";}
    else {requestMotor(settings.power,settings.power);navigationState="SEEKING_TAPE";}
    return;
  }
  if(mode==LINE||mode==STOP_AT_END){
    if(mode==STOP_AT_END&&!lineMask()){stopRobot();navigationState="TAPE_ENDED";return;}
    auto input=lineInput();
    auto power=lineWheels.step(lineFollower.step(input),input,settings.power);
    navigationState=lineFollower.reason;requestMotor(power.left,power.right);
    if(lineFollower.state==line::Follower::STOPPED){const char*why=lineFollower.reason;stopRobot();navigationState=why;}
    return;
  }
  if(mode==RACE){serviceRace();return;}
  if(mode==WALL_MAZE){serviceWall();return;}
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
  if(mode==MAZE) {
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
  Serial.print(',');Serial.print(motorLeft);Serial.print(',');Serial.print(motorRight);Serial.print(',');Serial.print(odometry.configured?1:0);reportWallTrace();
}

bool configureLineDrive(int cruise,int minimum){
  if(cruise<=lineWheels.feedForwardOffset)return false;
  stopRobot();lineWheels.cruise=cruise;lineWheels.minimum=minimum;return true;
}
void reportLineControl(){
  auto i=lineInput();
  Serial.print(F("LINECTRL,"));Serial.print((int)lineFollower.state);
  Serial.print(',');Serial.print(lineWheels.cruise);Serial.print(',');Serial.print(lineWheels.minimum);
  Serial.print(',');Serial.print(lineWheels.saturated?1:0);
  Serial.print(F(",0,0")); // Reserved legacy heading/remaining fields; no geometric planner.
  Serial.print(',');Serial.print(i.leftSpeed);Serial.print(',');Serial.println(i.rightSpeed);
}

void armBatteryLine(){batteryLinePending=true;batteryLineArmedAt=millis();}
void serviceBatteryLine(){
  if(!batteryLinePending||(uint32_t)(millis()-batteryLineArmedAt)<1500)return;
  batteryLinePending=false;
  startNavigation(LINE);
  standaloneActive=mode==LINE;
}

void configureLineSpeeds(float cruise,float turn,float correction,float capture,bool keepRunning){
  if(!keepRunning)stopRobot();
  auto& c=lineFollower.config;
  c.cruiseSpeed=cruise;c.turnSpeed=turn;c.correctionSpeed=correction;c.captureSpeed=capture;
}
bool configureLineMotor(float reference,float offset,float kp,float ki,int start){
  if(offset>=lineWheels.cruise)return false;
  stopRobot();lineWheels.referenceSpeed=reference;lineWheels.feedForwardOffset=offset;
  lineWheels.kp=kp;lineWheels.ki=ki;lineWheels.startPower=start;return true;
}
void reportLineTuning(){
  auto& c=lineFollower.config;
  Serial.print(F("LINETUNE,"));Serial.print(c.cruiseSpeed);Serial.print(',');Serial.print(c.turnSpeed);
  Serial.print(',');Serial.print(c.correctionSpeed);Serial.print(',');Serial.print(c.captureSpeed);
  Serial.print(',');Serial.print(lineWheels.referenceSpeed);Serial.print(',');Serial.print(lineWheels.feedForwardOffset);
  Serial.print(',');Serial.print(lineWheels.kp);Serial.print(',');Serial.print(lineWheels.ki);
  Serial.print(',');Serial.println(lineWheels.startPower);
}

void lineWirelessStatus(char* out,size_t size){
 const auto& c=lineFollower.config;auto i=lineInput();
 snprintf(out,size,"\"phase\":%d,\"speed\":[%.1f,%.1f],\"tune\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%d,%d,%d]",
  (int)lineFollower.state,i.leftSpeed,i.rightSpeed,c.cruiseSpeed,c.turnSpeed,c.correctionSpeed,c.captureSpeed,
  lineWheels.referenceSpeed,lineWheels.feedForwardOffset,lineWheels.kp,lineWheels.ki,lineWheels.startPower,lineWheels.cruise,lineWheels.minimum);
}
