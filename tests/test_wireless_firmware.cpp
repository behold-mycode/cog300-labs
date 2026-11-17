#define ROBOT_ENABLE_STANDALONE 1
#define ARDUINO_UNOR4_WIFI
#include "firmware_fixture.h"
static void request(const char*command){
 std::string b(command),r="POST /command HTTP/1.1\r\nX-Robot-Control: 1\r\nContent-Length: "+std::to_string(b.size())+"\r\n\r\n"+b;
 for(char c:r)webBytes.push_back(c);pendingWeb=true;
}
int main(){
 setup();assert(apReady);
 request("RUN START");serviceWireless();assert(mode==MANUAL&&!standaloneActive);assert(!strcmp(navigationState,"HARDWARE_DIMENSIONS_REQUIRED"));
 assert(configureHardware(syntheticHardware()));
 command("SONAR 1 70");command("ENC 1 0 0");command("ODOM 1 1 120");tick(10);
 command("BODY 90 70 60 80 30 50");command("WALLCFG 60 40 15 200 0.15 60 400");command("SPEEDCFG 200 200 0.6 0.3 150");
 command("CONFIG 255 255 1 0 20 10 1");command("RACECFG 50 100 1.8");tape(2);
 request("RUN START");serviceWireless();assert(standaloneActive&&mode==RACE&&!serverRunning&&!clientActive);
 int calls=modemCalls;for(int j=0;j<20;++j)serviceWireless();assert(modemCalls==calls);
 // Requests buffered during motion cannot restart a stopped robot.
 stopRobot();serviceWireless();assert(mode==MANUAL&&serverRunning);
 requestMotor(100,100);serviceWireless();request("RUN START");stopRobot();serviceWireless();assert(mode==MANUAL&&webBytes.empty());
 // Pending PWM demand matters even before output is applied.
 requestMotor(100,100);calls=modemCalls;serviceWireless();assert(modemCalls==calls);stopRobot();serviceWireless();
 command("HELLO");command("TEST 100 100 1000");calls=modemCalls;serviceWireless();assert(modemCalls==calls);stopRobot();
}
