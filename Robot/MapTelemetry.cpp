#include "Robot.h"
static bool exporting=false;
static uint16_t record=0;
static uint32_t lastRecordMs=0;
bool mapExportActive() { return exporting; }
void beginMapExport() { stopRobot();exporting=true;record=0; }
void reportNavigation() {
  Serial.print(F("NAV,"));Serial.print(encodersEnabled()?1:0);Serial.print(',');
  Serial.print(odometry.configured?1:0);Serial.print(',');Serial.print(explorer.geometry.valid()?1:0);Serial.print(',');
  Serial.print(explorer.localized?1:0);Serial.print(',');Serial.print(explorer.graph.nodeCount);Serial.print(',');
  Serial.print(explorer.graph.edgeCount);Serial.print(',');Serial.print(explorer.current);Serial.print(',');
  Serial.print(explorer.candidate);Serial.print(',');Serial.print(odometry.pose.x);Serial.print(',');
  Serial.print(odometry.pose.y);Serial.print(',');Serial.println(odometry.pose.heading);
  Serial.print(F("BELIEF,"));Serial.print(explorer.dynamics.valid()?1:0);
  for(float probability:explorer.world.belief) { Serial.print(',');Serial.print(probability); }
  Serial.print(',');Serial.print(explorer.world.leftSpeed);Serial.print(',');Serial.print(explorer.world.rightSpeed);
  Serial.print(',');Serial.print(explorer.world.headingUncertainty);
  Serial.print(',');Serial.println(explorer.world.updates);
}
void serviceMapExport() {
  if(!exporting || (uint32_t)(millis()-lastRecordMs)<10) return;
  lastRecordMs=millis();
  const maze::Graph &g=explorer.graph;
  if(record==0) {
    Serial.print(F("MAP_BEGIN,"));Serial.print(g.nodeCount);Serial.print(',');Serial.println(g.edgeCount);
  } else if(record<=g.nodeCount) {
    int n=record-1;const maze::Node &node=g.nodes[n];
    Serial.print(F("MAP_NODE,"));Serial.print(n);Serial.print(',');Serial.print(node.x);Serial.print(',');Serial.print(node.y);
    for(int p=0;p<node.count;++p) {
      Serial.print(',');Serial.print(node.ports[p].bearing);Serial.print(':');
      Serial.print(node.ports[p].edge);Serial.print(':');Serial.print(node.ports[p].blocked?1:0);
    }
    Serial.println();
  } else if(record<=g.nodeCount+g.edgeCount) {
    int id=record-g.nodeCount-1;const maze::Edge &e=g.edges[id];
    Serial.print(F("MAP_EDGE,"));Serial.print(id);Serial.print(',');Serial.print((int)e.a);Serial.print(',');
    Serial.print((int)e.ap);Serial.print(',');Serial.print((int)e.b);Serial.print(',');Serial.print((int)e.bp);
    Serial.print(',');Serial.print(e.length);Serial.print(',');Serial.println(e.traversals);
  } else { Serial.println(F("MAP_END"));exporting=false; }
  ++record;
}
