#include "firmware_fixture.h"
static bool wifi(const char*s,uint32_t token=0){char b[96];strcpy(b,s);return processWirelessCommand(b,token);}
int main(){
 setup();lineTapeLevel=0;tick(10);tape(5); // installed LOW tape -> centre only
 assert(wifi("CLAIM"));auto first=wirelessControlToken();assert(first);
 assert(!wifi("CLAIM"));assert(!wifi("MODE LINE",first+1));
 assert(wifi("MODE LINE",first));serviceNavigation();serviceMotor();assert(mode==LINE);
 command("HELLO");assert(!lastCommandOk&&mode==LINE&&!controlSession);
 command("LINESPEED 100 80 100 100");assert(!lastCommandOk&&mode==LINE);
 assert(wifi("HEARTBEAT",first));nowMs+=1499;serviceControlTimeout();assert(mode==LINE);
 nowMs++;serviceControlTimeout();assert(mode==MANUAL&&!wirelessControlToken()&&!motorDemandActive());
 assert(!wifi("MODE LINE",first));assert(wifi("CLAIM"));auto second=wirelessControlToken();assert(second!=first);
 assert(wifi("MODE LINE",second));command("STOP");assert(mode==MANUAL&&!wirelessControlToken());
 command("HELLO");assert(controlSession);assert(!wifi("CLAIM"));command("MODE LINE");assert(mode==LINE);
 assert(wifi("STOP"));assert(mode==MANUAL);assert(!controlSession);command("DRIVE 255 255");assert(!motorDemandActive());
 nowMs+=500;serviceControlTimeout();assert(!controlSession);assert(wifi("CLAIM"));
 auto third=wirelessControlToken();assert(wifi("LINESPEED 190 90 180 130",third));assert(lineFollower.config.cruiseSpeed==190);
 assert(!wifi("DRIVE 255 255",third));assert(!wifi("RUN START",third));
 // Rollover-safe lease.
 assert(wifi("RELEASE",third));nowMs=UINT32_MAX-700;assert(wifi("CLAIM"));assert(wifi("HEARTBEAT",wirelessControlToken()));nowMs+=1500;serviceControlTimeout();assert(!wirelessControlToken());
}
