#include "Robot.h"
#include "RadarSensor.h"
#include "ArenaSearch.h"
#include <math.h>
#include "LineWheelControl.h"
static uint32_t lastLineUs=0,maxLineGapUs=0;
static void recordLineTick(){
  uint32_t now=micros();
  if(lastLineUs && now-lastLineUs>maxLineGapUs)maxLineGapUs=now-lastLineUs;
  lastLineUs=now;
}
static bool batteryLinePending=false;
static uint32_t batteryLineArmedAt=0;
static line::Follower lineFollower;
static line::WheelControl lineWheels;
static line::Input lineInput(uint8_t tape){
  line::Input input{millis(),odometry.pose,tape};
  input.appliedLeft=motorLeft;input.appliedRight=motorRight;
  readWheelSpeeds(input.leftSpeed,input.rightSpeed,input.edgeSequence);
  return input;
}
static line::Input lineInput(){return lineInput(lineMask());}
static void applyLineDrive(line::Input input,line::Command target){
  auto power=lineWheels.step(target,input,settings.power);
  requestLineMotor(power.left,power.right,target.brakeInside && !target.left,
                   target.brakeInside && !target.right);
}

void beginRaceLine(){
  lastLineUs=maxLineGapUs=0;
  lineWheels.reset();
  lineFollower.begin(lineInput());
}
void serviceRaceLine(wall::Input input,uint8_t tape){
  recordLineTick();
  auto i=lineInput(tape);
  auto target=lineFollower.step(i);
  for(auto& range:input.ranges){
    if(range.noReturn&&(uint32_t)(input.now-range.stamp)<300){range.valid=true;range.mm=4000;}
  }
  raceNavigator.reason=lineFollower.reason;
  auto limited=raceNavigator.guarded(wallNavigator.config,input,{target.left,target.right});
  target.left=limited.left;target.right=limited.right;
  applyLineDrive(i,target);
  navigationState=raceNavigator.reason;
}

Mode mode=MANUAL;
NavigationConfig navigationConfig;
maze::Explorer explorer;
static arena::Search arenaSearch;
static bool profileSet=false, exitArmed=false, sawFloor=false;
enum class EntryPhase { WALL, STRIPE, CLEARING };
static EntryPhase entryPhase=EntryPhase::WALL;
enum class WallApproach { TRACK, PASS_OPENING, PIVOT };
static WallApproach approachPhase=WallApproach::TRACK;
static wall::Turn approachTurn;
static float openingPath=0;
static uint32_t cornerAt=0;
static bool acquiredWall=false;
static uint8_t stripeMask=0;
static uint32_t entryAt=0;
static float entryPath=0, clearPath=0, entryHeading=0;
static int benchAngle=0, benchSegment=0, benchPrevious=-1, benchStep=3, benchDirection=1;
static uint16_t benchSweep=0;
static uint32_t benchAimAt=0;

struct RadarPlotPoint { int16_t angle,mm,segment; };
static RadarPlotPoint plotTail[8];
static unsigned plotCount=0,plotNext=0;
static uint16_t plotSweep=0,sourceSweep=0;
static uint32_t plotTime=0;
static int plotLock[3]={};
void radarWirelessStatus(char* out,size_t size){
  if(!size)return;
  size_t used=0;
  auto append=[&](const char* fmt,auto... args){
    if(used>=size)return;
    int n=snprintf(out+used,size-used,fmt,args...);
    if(n>=0)used+=(size_t)n;
  };
  append("\"radar\":{\"sweep\":%u,\"ms\":%lu,\"step\":%d,\"points\":[",(unsigned)plotSweep,(unsigned long)plotTime,benchStep);
  for(unsigned i=0;i<plotCount;++i){
    const auto& p=plotTail[(plotNext+8-plotCount+i)%8];
    append("%s[%d,%d,%d]",i?",":"",p.angle,p.mm,p.segment);
  }
  append("%s","],\"lock\":[");
  if(plotLock[1]>0)append("%d,%d,%d",plotLock[0],plotLock[1],plotLock[2]);
  append("%s","]}");
}
void radarPoint(uint16_t sweep,int angle,int mm,int segment){
  if(!plotCount||sweep!=sourceSweep){plotCount=0;plotNext=0;plotLock[1]=0;sourceSweep=sweep;++plotSweep;}
  plotTail[plotNext]={(int16_t)angle,(int16_t)mm,(int16_t)segment};
  plotNext=(plotNext+1)%8;if(plotCount<8)++plotCount;
  plotTime=millis();
  Serial.print(F("RADAR,"));Serial.print(plotSweep);Serial.print(',');Serial.print(angle);
  Serial.print(',');Serial.print(mm);Serial.print(',');Serial.println(segment);
}
void radarLock(int angle,int mm,int width){
  plotLock[0]=angle;plotLock[1]=mm;plotLock[2]=width;
  Serial.print(F("LOCK,"));Serial.print(angle);Serial.print(',');Serial.print(mm);
  Serial.print(',');Serial.println(width);
}
static bool enableScanner(){
  radarEnable();
  if(radarReady())return true;
  navigationState="RADAR_SERVO_UNAVAILABLE";return false;
}
bool radarProfile(int minimum,int maximum){
  stopRobot();
  if(!enableScanner())return false;
  arenaSearch.setProfile(minimum,maximum);profileSet=true;return true;
}
void radarClearance(int mm){stopRobot();arenaSearch.setEntryClearance(mm);}
void radarBenchStep(int step){stopRobot();benchStep=step;}
bool radarStartBench(){
  stopRobot();
  if(!enableScanner())return false;
  mode=RADAR_SCAN;navigationState="RADAR_BENCH_SCAN";
  ++benchSweep;benchAngle=-60;benchDirection=1;benchSegment=0;benchPrevious=-1;
  benchAimAt=millis();radarAim(benchAngle);return true;
}
static void serviceBench(){
  const SensorReading r=radarReading();
  const bool fresh=(int32_t)(r.sampledMs-benchAimAt)>0 && r.sequence &&
    (r.status==RANGE_VALID||r.status==RANGE_NO_RETURN);
  if(!fresh && millis()-benchAimAt<950)return;
  int mm=fresh && r.status==RANGE_VALID?(int)lroundf(r.cm*10):-1;
  if(mm>0 && (benchPrevious<0 || !arena::Search::adjacent(benchPrevious,mm,benchStep)))++benchSegment;
  radarPoint(benchSweep,benchAngle,mm,mm>0?benchSegment:0);benchPrevious=mm;
  if(benchDirection*benchAngle==60){
    ++benchSweep;benchDirection=-benchDirection;benchSegment=0;benchPrevious=-1;
  }else benchAngle+=benchDirection*benchStep;
  benchAimAt=millis();radarAim(benchAngle);
}
static void watchMazeExit(){
  approachPhase=WallApproach::TRACK;acquiredWall=false;
  radarAim(0);exitArmed=true;sawFloor=false;entryPhase=EntryPhase::WALL;
}
bool radarArmExit(){
  if(!profileSet){navigationState="RADAR_PROFILE_REQUIRED";return false;}
  if(!radarReady()){navigationState="RADAR_SERVO_UNAVAILABLE";return false;}
  if(mode!=MANUAL && mode!=WALL && mode!=WALL_MAZE){navigationState="STOP_OR_SELECT_WALL_FIRST";return false;}
  if(lineTapeLevel<0 || !prepareWallCalibration(false)){navigationState="CALIBRATE_WHEELS_FIRST";return false;}
  if(mode==MANUAL){startNavigation(WALL);if(mode!=WALL)return false;}
  if(!exitArmed)watchMazeExit();
  navigationState="ARENA_EXIT_ARMED";return true;
}
static void arenaStop(const char* why){stopRobot();navigationState=why;}
// Two-sonar right-wall approach for an armed arena run. Use calibrated wheel
// speeds and measured quarter turns. The existing three-sonar graph solver
// retains control when the operator arms it mid-maze.
static void serviceArmedWall(){
  const auto& i=motionInput();
  const auto& c=wallNavigator.config;
  float front=wall::corrected(c,i,0);
  float right=wall::corrected(c,i,1);
  if(i.ranges[0].noReturn && i.now-i.ranges[0].stamp<300)front=4000;
  if(i.ranges[1].openBeyond && i.now-i.ranges[1].stamp<300)right=wall::SIDE_RANGE_MM;
  if(front<0 || right<0){driveVelocity({});navigationState="ARENA_WAITING_FOR_WALL_RANGE";return;}
  const float speed=fminf(80,c.cruise);
  const float stopping=speed*c.reaction+speed*speed/(2*c.deceleration);
  const float turnRadius=c.radius()+c.margin;
  if(approachPhase==WallApproach::PIVOT){
    if(i.now-cornerAt>9000 || front+c.frontSensor<=turnRadius || right+c.sideY<=turnRadius){
      arenaStop("ARENA_MAZE_PIVOT_BLOCKED");return;
    }
    auto d=approachTurn.step(c,i);
    if(approachTurn.failed){arenaStop("ARENA_MAZE_PIVOT_DRIFT");return;}
    if(approachTurn.done){approachPhase=WallApproach::TRACK;acquiredWall=false;}
    driveVelocity(d);navigationState="ARENA_MAZE_PIVOT";return;
  }
  const float desired=fmaxf(settings.wallCm*10,turnRadius-c.sideY+25);
  bool opening=right>desired+110;
  if(!opening){acquiredWall=true;approachPhase=WallApproach::TRACK;}
  else if(acquiredWall && approachPhase==WallApproach::TRACK){
    openingPath=i.pose.path;approachPhase=WallApproach::PASS_OPENING;
  }
  bool turnRight=approachPhase==WallApproach::PASS_OPENING && i.pose.path-openingPath>=c.sideX+turnRadius;
  bool turnLeft=front+c.frontSensor<turnRadius+stopping+45;
  if(turnRight||turnLeft){
    if(front+c.frontSensor<=turnRadius || right+c.sideY<=turnRadius){
      arenaStop("ARENA_MAZE_CORNER_TOO_TIGHT");return;
    }
    cornerAt=i.now;approachPhase=WallApproach::PIVOT;
    approachTurn.begin((turnRight?-.5f:.5f)*maze::PI_F,i.pose);driveVelocity({});return;
  }
  if(!acquiredWall && opening){driveVelocity({});navigationState="ARENA_NEEDS_RIGHT_WALL";return;}
  float steer=opening?0:wall::limit(.48f*(right-desired),-speed*.45f,speed*.45f);
  driveVelocity({speed+steer,speed-steer});navigationState="ARENA_TRACKING_RIGHT_WALL";
}
static bool serviceEntry(){
  const auto& i=motionInput();
  const uint8_t tape=lineMask();
  if(entryPhase==EntryPhase::WALL){
    if(!tape)sawFloor=true;
    if(!sawFloor && tape){arenaStop("ARENA_ARM_ON_BARE_FLOOR");return true;}
    if(!tape)return false;
    if((mode==WALL && approachPhase==WallApproach::PIVOT) ||
       ((mode==WALL_MAZE || mode==RACE) && wallNavigator.state!=wall::Navigator::CORRIDOR)){
      arenaStop("ARENA_TAPE_DURING_TURN");return true;
    }
    entryPhase=EntryPhase::STRIPE;entryPath=i.pose.path;entryAt=i.now;
    entryHeading=i.pose.heading;stripeMask=0;resetWallFeedback();
  }
  auto &c=wallNavigator.config;
  if(i.now-entryAt>15000 || fabsf(i.pose.heading-entryHeading)>.22f){
    arenaStop("ARENA_ENTRY_DRIFT_OR_TIMEOUT");return true;
  }
  if(entryPhase==EntryPhase::STRIPE){
    stripeMask|=tape;
    if(i.pose.path-entryPath>100){arenaStop("ARENA_EXIT_STRIPE_TOO_LONG");return true;}
    if(!tape){
      if(stripeMask!=7){arenaStop("ARENA_EXIT_MARKER_INCOMPLETE");return true;}
      clearPath=i.pose.path;entryPhase=EntryPhase::CLEARING;
    }
  }else if(tape){arenaStop("OBJECT_ARENA_BOUNDARY");return true;}
  // On release the floor sensors are already inside but the axle is still
  // behind the stripe. Clear the pivot envelope before allowing a turn.
  if(entryPhase==EntryPhase::CLEARING && i.pose.path-clearPath>=lineSensorOffset()+c.radius()+c.margin){
    bool standalone=standaloneActive;
    stopRobot();wallNavigator.invalidate();explorer.invalidate();resetWallFeedback();
    arenaSearch.begin(motionInput());mode=OBJECT;standaloneActive=standalone;
    navigationState=arenaSearch.reason;return true;
  }
  float front=wall::corrected(c,i,0);
  float speed=fminf(65,c.cruise), v=fmaxf(speed,fmaxf(fabsf(i.leftSpeed),fabsf(i.rightSpeed)));
  float needed=c.front-c.frontSensor+c.margin+v*c.reaction+v*v/(2*c.deceleration);
  if(front<needed){arenaStop("ARENA_ENTRY_FRONT_BLOCKED");return true;}
  navigationState=entryPhase==EntryPhase::STRIPE?"ARENA_CROSSING_STRIPE":"ARENA_CLEARING_STRIPE";
  float correction=wall::limit((entryHeading-i.pose.heading)*c.track,-speed*.35f,speed*.35f);
  driveVelocity({speed-correction,speed+correction});return true;
}
bool raceUsesArenaExit(){
  if(!profileSet)return false;
  if(!exitArmed)watchMazeExit();
  return true;
}
const char* navigationState="STOPPED";
static maze::ProgressGuard progress;
static bool wallTurning=false;
static maze::Motion wallMotion;
static void drive(maze::Drive d) { requestMotor(d.left*settings.power,d.right*settings.power); }
static void hold(const char* reason) { stopMotor();navigationState=reason; }
static bool feedbackReady() { return encodersEnabled()&&odometry.configured&&odometry.sampled; }
const char* modeName() {
  switch(mode) {
    case RADAR_SCAN:return "RADAR_SCAN";case CALIBRATE:return "CALIBRATE";case OBJECT:return "OBJECT";case RACE:return "RACE";case WALL_MAZE:return "WALL_MAZE";case FOLLOW:return "FOLLOW";case WALL:return "WALL";case LINE:return "LINE";
    case MAZE:return "MAZE";case STOP_ON_TAPE:return "STOP_ON_TAPE";case STOP_AT_END:return "STOP_AT_END";
    default:return "MANUAL";
  }
}
void cancelNavigation() {
  batteryLinePending=false;lineFollower.stop();lineWheels.reset();cancelAutomaticSetup();raceNavigator.stop();arenaSearch.stop();plotCount=0;plotLock[1]=0;exitArmed=false;standaloneActive=false;explorer.pause();pauseWall();mode=MANUAL;progress.reset();wallMotion.cancel();wallTurning=false;
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
  if(selected==OBJECT){navigationState="MAZE_EXIT_TAPE_REQUIRED";return;}
  if(selected==LINE||selected==STOP_AT_END||selected==STOP_ON_TAPE){
    lastLineUs=maxLineGapUs=0;
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
  if(profileSet && (selected==WALL||selected==WALL_MAZE)){
    if(!radarReady()||lineTapeLevel<0||!prepareWallCalibration(false)){
      arenaStop("ARENA_CALIBRATION_REQUIRED");return;
    }
    watchMazeExit();
  }
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
  if(mode==RADAR_SCAN){serviceBench();return;}
  if(automaticSetupActive()){serviceAutomaticSetup();return;}
  if((exitArmed||mode==MAZE||mode==WALL_MAZE||mode==RACE||mode==LINE||mode==OBJECT||mode==STOP_AT_END||mode==STOP_ON_TAPE) && !feedbackReady()) { hold("ENCODERS_REQUIRED");return; }
  if(mode!=LINE && !(mode==RACE && !raceNavigator.wallStage()) && feedbackReady() && !progress.check(odometry.pose,
      {(int8_t)(motorLeft>0?1:motorLeft<0?-1:0),(int8_t)(motorRight>0?1:motorRight<0?-1:0)},
      millis(),navigationConfig.stallMs)) {
    if(mode==WALL_MAZE)wallNavigator.fail("ENCODER_STALL");else explorer.fail("ENCODER_STALL");stopRobot();navigationState="ENCODER_STALL";return;
  }
  if(exitArmed && serviceEntry())return;
  if(mode==OBJECT){
    driveVelocity(arenaSearch.step(wallNavigator.config,motionInput(),lineMask(),radarReading()));navigationState=arenaSearch.reason;
    if(arenaSearch.state==arena::Search::FAULT||arenaSearch.state==arena::Search::DONE){
      const char*why=arenaSearch.reason;stopRobot();navigationState=why;
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
    recordLineTick();
    auto input=lineInput();
    auto target=lineFollower.step(input);
    applyLineDrive(input,target);
    navigationState=lineFollower.reason;
    return;
  }
  if(mode==RACE){serviceRace();return;}
  if(mode==WALL_MAZE){serviceWall();return;}
  if(mode==WALL) { if(exitArmed)serviceArmedWall();else followWall();return; }
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
  if(mode==LINE && !observationTrace)return;
  static uint32_t last=0,sequence=0;
  uint32_t now=millis();if(sequence&&(uint32_t)(now-last)<20) return;last=now;
  SensorReading f=sensorReading(0),r=sensorReading(1);
  maze::Observation o;o.sequence=++sequence;o.stamp=now;o.pose=odometry.pose;o.tape=lineMask();
  o.front={f.cm*10,f.sampledMs,f.status==RANGE_VALID};o.right={r.cm*10,r.sampledMs,r.status==RANGE_VALID};
  if(mode!=LINE&&odometry.configured&&lineTapeLevel>=0) explorer.observe(o);
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
 snprintf(out,size,"\"maxLineGapUs\":%lu,\"phase\":%d,\"speed\":[%.1f,%.1f],\"tune\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%d,%d,%d]",
  (unsigned long)maxLineGapUs,(int)lineFollower.state,i.leftSpeed,i.rightSpeed,c.cruiseSpeed,c.turnSpeed,c.correctionSpeed,c.captureSpeed,
  lineWheels.referenceSpeed,lineWheels.feedForwardOffset,lineWheels.kp,lineWheels.ki,lineWheels.startPower,lineWheels.cruise,lineWheels.minimum);
}
