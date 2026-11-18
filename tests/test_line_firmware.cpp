#include "firmware_fixture.h"
#include <initializer_list>
int main() {
 setupUnconfigured();command("LINECFG 245 232");assert(lastCommandOk);
 command("LINECFG 240 0");assert(lastCommandOk);
 command("LINESPEED 200 90 180 130");assert(lastCommandOk);
 command("LINEMOTOR 200 160 0.12 0.8 220");assert(lastCommandOk);
 command("LINEMOTOR 200 250 0.12 0.8 220");assert(!lastCommandOk);
 command("LINECFG 100 0");assert(!lastCommandOk);
 command("LINECFG 220 230");assert(!lastCommandOk);command("HELLO");command("MODE LINE");assert(mode==MANUAL&&motorLeft==0);
 assert(configureHardware(syntheticHardware()));serviceOdometry();
 command("BODY 90 70 60 80 30 50");command("WALLCFG 60 40 15 200 0.15 60 400");
 command("SPEEDCFG 200 200 0.6 0.3 150");command("RACECFG 50 90 1.8");
 command("CONFIG 255 255 1 0 20 10 1");tape(2);command("MODE LINE");
 for(int n=0;n<10;++n){command("HEARTBEAT");tick(50);}
 assert(motorLeft>0&&motorRight>0&&motorLeft<=settings.power);
 tape(4);command("HEARTBEAT");tick(10);assert(mode==LINE);
 tape(0);tick(20);assert(motorLeft==0&&motorRight>0&&lineFollower.state==line::Follower::FOLLOW);
 tick(501);assert(mode==MANUAL&&motorLeft==0&&motorRight==0);
 command("HELLO");tape(0);command("MODE STOP_ON_TAPE");tick(20);assert(mode==STOP_ON_TAPE);
 tape(1);tick(20);assert(mode==MANUAL&&motorLeft==0&&!strcmp(navigationState,"TAPE_REACHED"));
 tape(2);command("MODE STOP_AT_END");tick(20);assert(mode==STOP_AT_END);
 tape(0);tick(20);assert(mode==MANUAL&&motorLeft==0&&!strcmp(navigationState,"TAPE_ENDED"));
 command("CONFIG 255 255 1 0 20 10 0");tape(5);command("MODE LINE");tick(250);assert(mode==LINE);
 tick(20,2);assert(motorLeft>0&&motorRight>0);
 tick(20,-1);assert(motorLeft>0);
 // STOP cancels both active and pending reversing commands in every line phase.
 for(auto phase:{line::Follower::FOLLOW,line::Follower::CORNER,line::Follower::SEARCH}){
  tape(5);command("MODE LINE");lineFollower.state=phase;requestMotor(-240,240);serviceMotor();
  command("STOP");nowMs+=500;serviceMotor();
  assert(mode==MANUAL&&!motorDemandActive()&&!motorLeft&&!motorRight);
 }
 command("CONFIG 255 255 1 0 20 10 2");assert(lineTapeLevel==0&&mode==MANUAL);
}
