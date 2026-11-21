#define ARDUINO_UNOR4_WIFI
#include "firmware_fixture.h"
static void post(const char*command,uint32_t token=0){
 std::string b(command);webRequests.push_back("POST /command HTTP/1.1\r\nX-Robot-Control: 1\r\nX-Robot-Token: "+std::to_string(token)+"\r\nContent-Length: "+std::to_string(b.size())+"\r\n\r\n"+b);
}
static void run(int n){for(int j=0;j<n;++j){nowMs++;serviceControlTimeout();serviceOdometry();serviceNavigation();serviceMotor();serviceWireless();}}
int main(){
 setup();lineTapeLevel=0;tape(5);serviceOdometry();assert(apReady);int initialCalls=modemCalls;
 post("CLAIM");run(200);auto token=wirelessControlToken();assert(token);assert(webOutput.find("\"ok\":true")!=std::string::npos);
 post("MODE LINE",token);run(200);assert(mode==LINE&&motorDemandActive());assert(modemCalls==initialCalls);
 post("HEARTBEAT",token);run(200);assert(mode==LINE);
 webOutput.clear();webRequests.push_back("GET /status HTTP/1.1\r\n\r\n");run(200);
 assert(webOutput.find("\"owner\":\"Wi-Fi\"")!=std::string::npos);assert(webOutput.find("\"token\":0")!=std::string::npos);
 post("STOP");run(200);assert(mode==MANUAL&&!wirelessControlToken());
 post("MODE LINE",token);run(200);assert(mode==MANUAL);
 webOutput.clear();webRequests.push_back("GET / HTTP/1.1\r\n\r\n");run(1500);assert(webOutput.find("Content-Encoding: gzip")!=std::string::npos);
 auto body=webOutput.substr(webOutput.find("\r\n\r\n")+4);
 assert(body==std::string(reinterpret_cast<const char*>(wirelessPageGzip),sizeof(wirelessPageGzip)));
 // The complete page is transferred incrementally; no blocking WiFiS3 calls.
 assert(modemCalls==initialCalls&&Serial2.maxWrite<=16);
 post("CLAIM");run(200);token=wirelessControlToken();post("MODE LINE",token);run(200);assert(mode==LINE);
 radioSilent=true;run(2500);assert(mode==LINE&&motorDemandActive()&&!wirelessControlToken());
 radioSilent=false;run(2600);post("CLAIM");run(200);
 assert(mode==LINE&&wirelessControlToken()&&modemCalls==initialCalls);
 post("STOP");run(200);assert(mode==MANUAL&&!motorDemandActive());
}
