#include "firmware_fixture.h"
int main(){
 setup();configureHardware(syntheticHardware());serviceOdometry();
 command("BODY 90 70 60 80 30 50");command("WALLCFG 60 40 15 200 0.15 60 400");
 command("SPEEDCFG 200 200 0.6 0.3 150");command("RACECFG 50 90 1.8");
 command("CONFIG 255 255 1 0 20 10 1");command("HELLO");tape(2);command("MODE LINE");
 tick(250,-1,-1);assert(motorLeft==255&&motorRight==255);
 tape(4);tick(10,-1,-1);assert(motorLeft==0&&motorRight==0);
 command("HEARTBEAT");tick(250,-1,-1);assert(motorLeft<0&&motorRight>0);
 tick(1,-1,-1);tick(35,-1,-1);assert(motorLeft==0&&motorRight==0);
 tick(120,-1,-1);assert(motorLeft<0&&motorRight>0);
 tape(2);tick(1,-1,-1);assert(motorLeft==0&&motorRight==0); // Centre acquisition stops pivot immediately.
 command("STOP");tick(250,-1,-1);assert(mode==MANUAL&&motorLeft==0&&motorRight==0);
}
