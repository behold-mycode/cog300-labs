#define ROBOT_ENABLE_STANDALONE 1
#include "firmware_fixture.h"
int main(){
 setup();assert(mode==MANUAL);assert(!startStandalone());
 assert(configureHardware(syntheticHardware()));
 command("SONAR 1 70");command("ENC 1 0 0");command("ODOM 1 1 120");tick(10);
 command("BODY 90 70 60 80 30 50");command("WALLCFG 60 40 15 200 0.15 60 400");command("SPEEDCFG 200 200 0.6 0.3 150");
 command("CONFIG 255 255 1 0 20 10 1");command("RACECFG 50 100 1.8");
 command("SAVE");assert(calibrationSaved());
 settings.power=100;lineTapeLevel=-1;assert(loadCalibration());assert(settings.power==255&&lineTapeLevel==1&&mode==MANUAL);
 tape(2);acceptReading(2,15,870);tick(10,60,15);
 command("HELLO");assert(startStandalone());assert(standaloneActive&&mode==RACE&&!controlSession);
 tick(600,60,15);assert(mode==RACE); // Explicit standalone ignores the old USB lease.
 stopRobot();assert(!standaloneActive&&mode==MANUAL);
 command("HELLO");command("MODE RACE");tick(501);assert(mode==MANUAL); // Tethered RACE retains watchdog.
 command("RUN START");assert(mode==MANUAL&&!lastCommandOk); // Delayed USB start cannot revive an expired lease.
 command("HELLO");beginMapExport();assert(mapExportActive());assert(!startStandalone());
 command("RUN START");assert(mode==MANUAL&&!lastCommandOk&&mapExportActive());
}
