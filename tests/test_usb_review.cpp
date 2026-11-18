#include "firmware_fixture.h"
int main(int argc,char**argv){
 assert(argc==2);setup();configureHardware(syntheticHardware());serviceOdometry();
 command("BODY 90 70 60 80 30 50");command("WALLCFG 60 40 15 200 0.15 60 400");
 command("SPEEDCFG 200 200 0.6 0.3 150");command("CONFIG 255 255 1 0 20 10 1");command("RACECFG 50 100 1.8");
 if(!strcmp(argv[1],"lease")){
  tape(2);command("MODE RACE");assert(mode==MANUAL);return 0;
 }
 if(!strcmp(argv[1],"two_sonar")){
  command("SONAR 0 70");command("BODY 90 70 60 80 30 50");command("WALLCFG 60 40 15 200 0.15 60 400");
  command("SPEEDCFG 200 200 0.6 0.3 150");
  command("HELLO");tape(2);command("MODE LINE");
  for(int n=0;n<20;++n){command("HEARTBEAT");tick(50,60,15);}
  assert(mode==LINE&&motorLeft==240);
  tape(0);command("HEARTBEAT");tick(50,60,15);
  assert(mode==LINE&&lineFollower.state==line::Follower::FOLLOW&&motorLeft>0&&motorRight>0);return 0;
 }
 if(!strcmp(argv[1],"opening")){
  wallNavigator.config.corridorWidth=600;
  acceptReading(0,100,5800);acceptReading(1,25,1450);acceptReading(2,25,1450);
  auto i=motionInput();assert(wallNavigator.begin(i));
  acceptReading(1,100,5800);i=motionInput();wallNavigator.step(i);
  assert(wallNavigator.state==wall::Navigator::APPROACH);
  wall::Belief b;for(int k=0;k<10;++k){i.now+=100;i.ranges[1].stamp=i.now;i.ranges[1].sequence++;b.update(i.ranges[1],i.now,353.5f,505);}
  assert(b.open());return 0;
 }
 if(!strcmp(argv[1],"range")){
  acceptReading(0,200,11600);acceptReading(1,100,5800);acceptReading(2,20,1160);
  auto i=motionInput();assert(i.ranges[0].mm==2000&&i.ranges[1].mm==400&&i.ranges[2].mm==200);
  samples[1].valid=false;samples[1].status=RANGE_NO_RETURN;
  i=motionInput();assert(i.ranges[1].valid&&i.ranges[1].mm==400);
  samples[1].status=RANGE_INVALID;i=motionInput();assert(!i.ranges[1].valid);return 0;
 }
 if(!strcmp(argv[1],"boundary_setup")){
  lineTapeLevel=-1;command("HELLO");tape(0);command("MODE OBJECT");assert(!automaticSetupActive()&&mode==MANUAL);
  command("MODE STOP_ON_TAPE");assert(!automaticSetupActive()&&mode==MANUAL);return 0;
 }
 command("HELLO");tape(2);command("MODE RACE");
 if(!strcmp(argv[1],"corner")){
  for(int n=0;n<20;++n){tape(0);acceptReading(2,15,870);command("HEARTBEAT");tick(50,60,15);}
  assert(mode==RACE&&!raceWallStarted);assert(raceNavigator.state!=race::Navigator::TRACK);return 0;
 }
 if(!strcmp(argv[1],"wide")){
  for(int n=0;n<20;++n){tape(7);acceptReading(2,60,3480);command("HEARTBEAT");tick(50,60,60);}
  assert(mode==RACE&&!raceWallStarted);assert(motorLeft>0&&motorRight>0);return 0;
 }
 if(!strcmp(argv[1],"encoder")){
  stopRobot();requestMotor(-100,-100);nowMs+=300;serviceMotor();
  leftEncoderChanged();rightEncoderChanged();serviceOdometry();
  assert(odometry.pose.left==-1&&odometry.pose.right==-1);return 0;
 }
 if(!strcmp(argv[1],"stop")){
  stopRobot();requestMotor(100,100);nowMs+=300;serviceMotor();
  lastControlStep=nowMs;driveVelocity({});serviceMotor();
  assert(motorLeft==0&&motorRight==0);return 0;
 }
 assert(false);
}
