#define ROBOT_LINE_ONLY 1
#include "firmware_fixture.h"
int main(){setup();assert(batteryLinePending);char claim[]="CLAIM";assert(processWirelessCommand(claim,0));assert(!batteryLinePending);nowMs+=1600;serviceBatteryLine();assert(mode==MANUAL&&!motorDemandActive());}
