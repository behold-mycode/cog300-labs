#include "Robot.h"
#include "WirelessHttp.h"
#if defined(ARDUINO_UNOR4_WIFI) && ROBOT_ENABLE_STANDALONE
#include <WiFiS3.h>
#include "WirelessPage.h"
#if __has_include("WirelessSecrets.h")
#include "WirelessSecrets.h"
#else
// A fresh clone remains buildable, but cannot accidentally create an open AP.
#define ROBOT_WIFI_SSID "COGS-Robot"
#define ROBOT_WIFI_PASSWORD ""
#endif
static WiFiServer webServer(80);
static WiFiClient webClient;
static wireless::Request webRequest;
static bool apReady=false,serverRunning=false,clientActive=false,dirtySocket=false;
static uint32_t clientSince=0;
void suspendWireless(){
 if(clientActive){webClient.stop();clientActive=false;}
 if(serverRunning){webServer.end();serverRunning=false;}
 webRequest.reset();dirtySocket=false;
}
void setupWireless(){
 if(strlen(ROBOT_WIFI_PASSWORD)<8||strlen(ROBOT_WIFI_PASSWORD)>63){Serial.println(F("WIFI_DISABLED configure WirelessSecrets.h"));return;}
 WiFi.config(IPAddress(192,168,4,1));
 apReady=WiFi.beginAP(ROBOT_WIFI_SSID,ROBOT_WIFI_PASSWORD)==WL_AP_LISTENING;
 if(apReady){webServer.begin();serverRunning=true;Serial.println(F("WIFI_READY http://192.168.4.1"));}
 else Serial.println(F("WIFI_FAILED physical start remains available"));
}
static void header(bool ok,const char*type){
 webClient.print(ok?"HTTP/1.1 200 OK\r\n":"HTTP/1.1 400 Bad Request\r\n");
 webClient.print("Connection: close\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\nContent-Type: ");webClient.print(type);webClient.print("\r\n\r\n");
}
static void configuration(){
 webClient.print("CONFIG ");webClient.print(settings.power);webClient.print(' ');webClient.print(settings.minimum);webClient.print(' ');webClient.print(settings.invertA);webClient.print(' ');webClient.print(settings.invertB);webClient.print(' ');webClient.print(settings.followCm);webClient.print(' ');webClient.print(settings.wallCm);webClient.print(' ');webClient.println(lineTapeLevel);
 webClient.print("SONAR ");webClient.print(leftSonarEnabled());webClient.print(' ');webClient.println(sonarInterval());
 webClient.print("ENC ");webClient.print(encoderKind());webClient.print(' ');webClient.print(encoderInvertLeft());webClient.print(' ');webClient.println(encoderInvertRight());
 webClient.print("ODOM ");webClient.print(odometry.leftScale,5);webClient.print(' ');webClient.print(odometry.rightScale,5);webClient.print(' ');webClient.println(odometry.track);
 const auto&c=wallNavigator.config;
 webClient.print("BODY ");float body[]={c.front,c.rear,c.halfWidth,c.frontSensor,c.sideX,c.sideY};for(float x:body){webClient.print(x);webClient.print(' ');}webClient.println();
 webClient.print("WALLCFG ");float wall[]={c.cruise,c.turnSpeed,c.margin,c.deceleration,c.reaction,c.matchRadius,c.corridorWidth};for(float x:wall){webClient.print(x);webClient.print(' ');}webClient.println();
 const auto&v=wheelVelocity.config;webClient.print("SPEEDCFG ");float speed[]={v.fullLeft,v.fullRight,v.kp,v.ki,v.acceleration};for(float x:speed){webClient.print(x);webClient.print(' ');}webClient.println();
 webClient.print("RACECFG ");webClient.print(raceConfig.speed);webClient.print(' ');webClient.print(raceConfig.sensorOffset);webClient.print(' ');webClient.println(raceConfig.searchAngle);
}
void serviceWireless(){
 // Every radio/client operation can block on the ESP modem. Never call one
 // while driving, even if it looks like an inexpensive connected()/available().
 if(mode!=MANUAL||motorLeft||motorRight||motorDemandActive()){dirtySocket=true;return;}
 if(!apReady)return;
 if(dirtySocket)suspendWireless();
 if(!serverRunning){webServer.begin();serverRunning=true;}
 if(!clientActive){webClient=webServer.available();if(!webClient)return;clientActive=true;clientSince=millis();webRequest.reset();}
 for(int budget=0;budget<128&&webClient.available()&&!webRequest.complete&&!webRequest.bad;++budget)webRequest.feed((char)webClient.read());
 if(!webRequest.complete&&!webRequest.bad&&(uint32_t)(millis()-clientSince)<1500)return;
 bool start=false;
 if(webRequest.get("/")){header(true,"text/html; charset=utf-8");webClient.print(wirelessPage);}
 else if(webRequest.get("/status")){
  header(true,"text/plain");webClient.print("State: ");webClient.println(navigationState);
  webClient.print("Saved calibration available: ");webClient.println(calibrationSaved()?"yes":"no");
  webClient.print("Tape mask: ");webClient.println(lineMask());
  webClient.print("Ranges front/right/left cm: ");for(int n=0;n<3;++n){webClient.print(sensorDistanceCm(n));webClient.print(' ');}webClient.println();
 }else if(webRequest.get("/config")){header(true,"text/plain");configuration();}
 else{
  char cmd[96];bool ok=webRequest.command(cmd);
  if(ok){
   start=!strcmp(cmd,"RUN START");
   if(!start){controlSession=false;controlExpired=false;processCommand(cmd);ok=lastCommandOk;}
  }
  header(ok,"text/plain");webClient.println(ok?(start?"Start requested. Wi-Fi pauses during motion; use the physical button to stop.":"Command accepted."):"Command rejected. Check syntax, calibration, and whether the map must be cleared.");
 }
 webClient.stop();clientActive=false;webRequest.reset();
 if(start)startStandalone(); // Socket closes before motors can be enabled.
}
#else
void setupWireless(){}
void serviceWireless(){}
void suspendWireless(){}
#endif
