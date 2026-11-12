#include "firmware_fixture.h"
int main() {
  setup();lineTapeLevel=1;tape(2);
  startNavigation(MAZE);assert(mode==MANUAL&&!strcmp(navigationState,"ENCODERS_REQUIRED"));
  command("ENC 2 0 0");command("ODOM 1 1 100");tick(1);
  startNavigation(MAZE);assert(mode==MANUAL&&!strcmp(navigationState,"GEOMETRY_REQUIRED"));
  command("GEOMETRY 40 10 20 80");startNavigation(MAZE);assert(mode==MANUAL&&!strcmp(navigationState,"DYNAMICS_REQUIRED"));command("DYNAMICS 200 0.1 10");startNavigation(MAZE);tick(10);
  assert(mode==MAZE&&explorer.graph.nodeCount==1&&motorLeft==255&&motorRight==255);
  tick(10);tick(1501);assert(mode==MANUAL&&!strcmp(navigationState,"ENCODER_STALL"));
  assert(explorer.graph.nodeCount==1);command("MAP RESET");assert(explorer.graph.nodeCount==0);
  startNavigation(MAZE);tick(1);command("STOP");assert(explorer.graph.nodeCount==1&&explorer.localized);
  command("HELLO");assert(explorer.graph.nodeCount==1);command("DRIVE 255 255");assert(!explorer.localized);
  command("MODE MAZE");assert(mode==MANUAL&&!strcmp(navigationState,"RELOCALIZE_REQUIRED"));
  command("MAP DUMP");command("MAP RESET");assert(explorer.graph.nodeCount==1); // Export snapshot lock.
  for(int i=0;i<4;++i) tick(10);
  assert(!mapExportActive());assert(Serial.output.str().find("MAP_NODE,0,")!=std::string::npos);
  command("MAP RESET");assert(explorer.graph.nodeCount==0);
  command("ENC 0 0 0");startNavigation(WALL);tick(10,60,10);assert(motorLeft==255&&motorRight==255);
  tick(10,60,30);assert(motorLeft==255&&motorRight==0);
  tick(10,60,-1);assert(motorLeft==0&&motorRight==0&&!strcmp(navigationState,"WALL_LOST"));
  tick(10,19,10);assert(motorLeft==0&&!strcmp(navigationState,"ENCODERS_REQUIRED_FOR_TURN"));
  command("ENC 2 0 0");command("ODOM 1 1 100");command("GEOMETRY 40 10 20 80");tick(1);startNavigation(WALL);tick(10,19,10);
  assert(!strcmp(navigationState,"WALL_MEASURED_LEFT"));tick(250,19,10);assert(motorLeft==-255&&motorRight==255);
  tick(10,25,10);assert(motorLeft==0&&!strcmp(navigationState,"WALL_FRONT_CLEARED"));
  // No assumed 90-degree completion; front observation ends this turn.
  command("ODOM 2 2 120");assert(!explorer.geometry.valid());
  command("GEOMETRY 40 10 20 80");assert(explorer.geometry.valid());
  command("ENC 2 0 0");assert(!odometry.configured&&!explorer.geometry.valid());
  command("TRACE 1");tick(20);assert(Serial.output.str().find("OBS,")!=std::string::npos);command("TRACE 0");
  leftTicks=123;rightTicks=456;reportEncoders();
  assert(Serial.output.str().find("COUNTS,123,456,2")!=std::string::npos);

}
