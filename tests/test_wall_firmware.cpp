#include "firmware_fixture.h"
int main(){
 setup();command("MODE WALL_MAZE");assert(mode==MANUAL&&motorLeft==0);
 command("SONAR 1 25");assert(leftSonarEnabled());
 command("ENC 1 0 0");command("ODOM 1 1 120");tick(10);
 command("BODY 90 70 60 80 30 50");
 command("WALLCFG 80 45 15 250 0.15 60 400");
 command("SPEEDCFG 200 210 0.6 0.3 150");
 acceptReading(2,15,870);command("MODE WALL_MAZE");assert(mode==WALL_MAZE);
 for(int k=0;k<8;++k){acceptReading(2,15,870);tick(20,60,15);}
 assert(motorLeft>0&&motorRight>0&&motorLeft<255&&motorRight<255);
 command("MAP DUMP");assert(mode==MANUAL&&motorLeft==0);
 for(int k=0;k<5;++k)tick(20);assert(!mapExportActive());
 assert(Serial.output.str().find("MAP_NODE,0,")!=std::string::npos);
 command("HELLO");command("DRIVE 100 100");command("STOP");command("MODE WALL_MAZE");
 assert(mode==MANUAL&&!wallNavigator.localized);
 command("MAP RESET");assert(explorer.graph.nodeCount==0&&wallNavigator.localized);
  command("TRACE 1");tick(20);assert(Serial.output.str().find("OBS,")!=std::string::npos);
 command("CONFIG 255 255 0 0 20 10 -1");assert(!wheelVelocity.config.valid());
  command("SONAR 0 70");command("MODE WALL_MAZE");assert(mode==MANUAL);
}
