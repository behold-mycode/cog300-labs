#include "Robot.h"
wall::Navigator wallNavigator(explorer.graph);
wall::Velocity wheelVelocity;
static wall::Input wallInput;
static uint32_t lastVelocity=0,lastControlStep=0;
static float lastWheelL=0,lastWheelR=0;
static bool velocitySampled=false;
static wall::Speeds targets;
void resetWallFeedback(){velocitySampled=false;wheelVelocity.reset();targets={};}
void resetWallCalibration(){wallNavigator.config=wall::Config();wheelVelocity.config=wall::VelocityConfig();velocitySampled=false;wheelVelocity.reset();}
void pauseWall(){wallNavigator.pause();wheelVelocity.reset();targets={};}
static void readWallInput(){
 uint32_t now=millis();wallInput.now=now;wallInput.pose=odometry.pose;
 if(!velocitySampled){lastVelocity=now;lastWheelL=odometry.pose.left;lastWheelR=odometry.pose.right;velocitySampled=true;wallInput.leftSpeed=wallInput.rightSpeed=0;}
 if((uint32_t)(now-lastVelocity)>=100){
  float dt=(uint32_t)(now-lastVelocity)*.001f;
  wallInput.leftSpeed=(odometry.pose.left-lastWheelL)/dt;wallInput.rightSpeed=(odometry.pose.right-lastWheelR)/dt;
  lastVelocity=now;lastWheelL=odometry.pose.left;lastWheelR=odometry.pose.right;
 }
 for(int n=0;n<3;++n){auto r=sensorReading(n);wallInput.ranges[n]={r.cm*10,r.sampledMs,r.sequence,r.status==RANGE_VALID,sensorPose(n),r.status==RANGE_NO_RETURN};
  if(n>0&&(r.status==RANGE_VALID||r.status==RANGE_NO_RETURN)){
   wallInput.ranges[n].mm=r.status==RANGE_NO_RETURN?wall::SIDE_RANGE_MM:fminf(r.cm*10,wall::SIDE_RANGE_MM);
   wallInput.ranges[n].valid=true;wallInput.ranges[n].noReturn=false;
   wallInput.ranges[n].openBeyond=r.status==RANGE_NO_RETURN||r.cm*10>=wall::SIDE_RANGE_MM;
  }}
}
bool prepareWallCalibration(bool requireLeft){
 if(requireLeft&&!leftSonarEnabled()){navigationState="LEFT_SONAR_REQUIRED";return false;}
 if(!encodersEnabled()||!odometry.configured||!odometry.sampled){navigationState="ENCODERS_REQUIRED";return false;}
 if(!wheelVelocity.config.valid()){navigationState="SPEED_CALIBRATION_REQUIRED";return false;}
 auto &c=wallNavigator.config;c.track=odometry.track;
 c.distanceTolerance=fmaxf(odometry.leftScale,odometry.rightScale)*1.5f;
 c.angleTolerance=fmaxf(.025f,2*c.distanceTolerance/c.track);
 return c.valid();
}
wall::Input motionInput(){readWallInput();return wallInput;}
void driveVelocity(wall::Speeds target){
 if(target.left==0&&target.right==0){stopMotor();wheelVelocity.reset();targets={};return;}
 readWallInput();uint32_t now=millis();if((uint32_t)(now-lastControlStep)<20)return;
 float dt=(uint32_t)(now-lastControlStep)*.001f;lastControlStep=now;targets=target;
 auto p=wheelVelocity.step(target.left,target.right,wallInput.leftSpeed,wallInput.rightSpeed,dt,settings.power);requestMotor(p.left,p.right);
}
bool startWall(){
 if(!prepareWallCalibration())return false;
 readWallInput();wheelVelocity.reset();lastControlStep=millis();
 // On a fresh autonomous wall map, measure the corridor rather than require
 // its width as an app setting. Three rays are still required for navigation.
 if(hardwareFingerprint()&&!explorer.graph.nodeCount){
  float r=wall::corrected(wallNavigator.config,wallInput,1),l=wall::corrected(wallNavigator.config,wallInput,2);
  float width=r+l+2*wallNavigator.config.sideY;
  if(r>=0&&l>=0&&width>2*(wallNavigator.config.halfWidth+wallNavigator.config.margin))wallNavigator.config.corridorWidth=width;
 }
 if(!wallNavigator.begin(wallInput)){navigationState=wallNavigator.reason;return false;}
 return true;
}
void serviceWall(){
 readWallInput();uint32_t now=millis();
 if((uint32_t)(now-lastControlStep)<20)return;
 float dt=(uint32_t)(now-lastControlStep)*.001f;lastControlStep=now;
 targets=wallNavigator.step(wallInput);navigationState=wallNavigator.reason;
 wall::Powers p=wheelVelocity.step(targets.left,targets.right,wallInput.leftSpeed,wallInput.rightSpeed,dt,settings.power);
 requestMotor(p.left,p.right);
 if(wallNavigator.state==wall::Navigator::FAULT||wallNavigator.state==wall::Navigator::COMPLETE||wallNavigator.state==wall::Navigator::CONFIRM){
  const char*why=wallNavigator.reason;stopRobot();navigationState=why;
 }
}
void reportWall(){
 Serial.print(F("WALLNAV,"));Serial.print(leftSonarEnabled()?1:0);Serial.print(',');
 Serial.print(wallNavigator.config.valid()&&wheelVelocity.config.valid()?1:0);Serial.print(',');
 Serial.print(wallNavigator.localized?1:0);Serial.print(',');Serial.print(wallNavigator.ownsGraph?1:0);Serial.print(',');
 Serial.print(wallNavigator.current);Serial.print(',');Serial.print(wallNavigator.candidate);Serial.print(',');
 Serial.print(sensorDistanceCm(2));
 for(auto &b:wallNavigator.beliefs){Serial.print(',');Serial.print(b.wallProbability);}
 Serial.print(',');Serial.print(targets.left);Serial.print(',');Serial.print(targets.right);
 Serial.print(',');Serial.print(wallInput.leftSpeed);Serial.print(',');Serial.println(wallInput.rightSpeed);
}

void reportWallTrace(){
 auto left=sensorReading(2);
 Serial.print(',');Serial.print(left.cm*10);Serial.print(',');Serial.print((uint32_t)(millis()-left.sampledMs));
 for(auto &b:wallNavigator.beliefs){Serial.print(',');Serial.print(b.wallProbability);}
 Serial.print(',');Serial.print(targets.left);Serial.print(',');Serial.print(targets.right);
 Serial.print(',');Serial.print(wallInput.leftSpeed);Serial.print(',');Serial.print(wallInput.rightSpeed);
 Serial.print(',');Serial.println((int)wallNavigator.state);
}
