#define ARDUINO_UNOR4_WIFI
#include "firmware_fixture.h"
static bool wifi(const char*s,uint32_t token=0){char b[96];strcpy(b,s);return processWirelessCommand(b,token);}
int main(){
 setup();lineTapeLevel=0;tick(10);tape(5);
 assert(wifi("CLAIM"));auto token=wirelessControlToken();
 assert(wifi("MODE LINE",token));serviceNavigation();serviceMotor();
 assert(mode==LINE&&motorDemandActive());
 assert(wifi("LINESPEED 300 250 250 125",token));
 assert(mode==LINE&&motorDemandActive()&&wirelessControlToken()==token);
 assert(lineFollower.config.cruiseSpeed==300);
 assert(!wifi("LINESPEED 400 250 250 125",token+1));
 assert(lineFollower.config.cruiseSpeed==300&&mode==LINE);
 assert(wifi("LINESPEED 300 250 450 0",token));
 assert(mode==LINE && lineFollower.config.captureSpeed==0);
 // Status exposes the estimated pose while moving, without resetting motion.
 odometry.pose.x=123.5f;odometry.pose.y=-45.5f;odometry.pose.heading=.5f;
 status(true,token);
 assert(std::string(statusBody).find("\"pose\":[123.5,-45.5,0.5000]")!=std::string::npos);
 assert(std::string(statusBody).find("\"poseReady\":true")!=std::string::npos);
 // Motor calibration still stops; a speed change while stopped never starts.
 assert(wifi("LINECFG 240 0",token));assert(mode==MANUAL&&!motorDemandActive());
 assert(wifi("LINESPEED 250 250 250 125",token));assert(mode==MANUAL);
 assert(wifi("MODE LINE",token));nowMs+=1500;serviceControlTimeout();
 assert(mode==MANUAL&&!motorDemandActive());
 assert(!wifi("LINESPEED 300 250 250 125",token));
 // Changing odometry scale resets its coordinate origin, so a viewer must
 // receive a new epoch rather than draw a line back to zero.
 command("HELLO");auto epoch=odometryEpoch;
 command("ODOM 5 5 130");assert(lastCommandOk);
 assert(odometryEpoch!=epoch);serviceOdometry();
 assert(odometry.pose.x==0&&odometry.pose.y==0);
 epoch=odometryEpoch;resetOdometry();assert(odometryEpoch!=epoch);
}
