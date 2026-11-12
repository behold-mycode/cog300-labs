#include "firmware_fixture.h"
void advance(int mask,int ms=50){tape(mask);acceptReading(2,15,870);tick(ms,60,15);}
int main(){
 setup();assert(configureHardware(syntheticHardware()));command("SONAR 1 70");command("ENC 1 0 0");command("ODOM 1 1 120");tick(10);
 command("BODY 90 70 60 80 30 50");command("WALLCFG 60 40 15 200 0.15 60 400");command("SPEEDCFG 200 200 0.6 0.3 150");command("CONFIG 255 255 1 0 20 10 1");command("RACECFG 50 100 1.8");
 advance(0);assert(startStandalone());assert(raceWallStarted&&mode==RACE);
 int nodes=explorer.graph.nodeCount;assert(nodes==1);
 advance(2);assert(raceWallStarted); // A transient reading cannot switch guidance.
 advance(0);assert(raceWallStarted);
 for(int n=0;n<10;++n)advance(2);
 assert(!raceWallStarted&&resumeWallGraph&&mode==RACE);assert(explorer.graph.nodeCount==nodes);
 advance(0);assert(!raceWallStarted);
 for(int n=0;n<10;++n)advance(0);
 assert(raceWallStarted&&mode==RACE);assert(explorer.graph.nodeCount==nodes);
 stopRobot();advance(2);assert(startStandalone());assert(!raceWallStarted);
 for(int n=0;n<10;++n)advance(0);
 assert(raceWallStarted&&mode==RACE);
 stopRobot();advance(7);assert(!startStandalone()&&mode==MANUAL);
}
