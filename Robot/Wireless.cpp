#include "Robot.h"
#include <limits.h>
#include "RadarSensor.h"
#include "WirelessHttp.h"
#if defined(ARDUINO_UNOR4_WIFI) && ROBOT_ENABLE_WIFI
#include <WiFiS3.h>
#include "Modem.h"
#include "AsyncRadio.h"
#include "FirmwareUpdate.h"
#include "WirelessPageGzip.h"
#if __has_include("WirelessSecrets.h")
#include "WirelessSecrets.h"
#else
#define ROBOT_WIFI_SSID "COGS-Robot"
#define ROBOT_WIFI_PASSWORD ""
#endif

#if ROBOT_OTA_CONFIGURED
static_assert(sizeof(ROBOT_WIFI_PASSWORD) >= 9 && sizeof(ROBOT_WIFI_PASSWORD) <= 64,
              "OTA builds require an 8-63 character access point password");
#endif

// WiFiS3 is used during stopped startup and signed firmware installation. During operation each call sends
// at most 16 UART bytes (~1.4ms at 115200), reads at most 128 buffered bytes,
// and never waits for a network/modem response. No core/library patches needed.
namespace {
enum Operation { ACCEPT, AVAILABLE, RECEIVE, SEND, CLOSE, HEALTH, RESET_RADIO, AP_MODE, AP_ADDRESS, AP_START, SERVER_START } operation=ACCEPT;
wireless::Reply reply;
wireless::Request request;
bool apReady=false,pending=false,resync=false;
int serverSocket=-1,clientSocket=-1;
uint32_t started=0,clientSince=0,nextPoll=0,wifiMaxUs=0;
char transmit[320]{};size_t txSize=0,txOffset=0;
char responseHeader[240]{},statusBody[1128]{};
const char* responseBody=nullptr;
size_t responseOffset=0,headerSize=0,bodySize=0,sendingSize=0;

bool retryEnabled=false;
unsigned failures=0;
uint32_t radioRestartAttempts=0;
uint32_t nextHealth=0, nextRestart=0;

void scheduleRestart(){
 apReady=false;pending=false;resync=false;serverSocket=clientSocket=-1;
 releaseWirelessControl();request.reset();nextRestart=millis()+3000;
}

void radioFailure(){
 if(failures<3)++failures;
 if(!apReady || failures>=3){scheduleRestart();return;}
 pending=false;resync=true; // Ownership still expires normally if heartbeats stop.
 // A timed-out reply can still arrive. Drain it before issuing another AT command.
 nextPoll=millis()+250;
 if(operation==CLOSE)clientSocket=-1;
 request.reset();
 Serial.println(F("WIFI_RETRY"));
}
void beginOperation(Operation op,int amount=0){
 operation=op;
 const char* name="";
 int n=0;
 switch(op){
 case RESET_RADIO:++radioRestartAttempts;name="SOFTRESETWIFI";n=snprintf(transmit,sizeof(transmit),"AT+%s\r\n",name);break;
 case AP_MODE:name="WIFIMODE";n=snprintf(transmit,sizeof(transmit),"AT+%s=2\r\n",name);break;
 case AP_ADDRESS:name="SOFTAPCONFIG";n=snprintf(transmit,sizeof(transmit),"AT+%s=192.168.4.1,192.168.4.1,255.255.255.0\r\n",name);break;
 case AP_START:name="BEGINSOFTAP";n=snprintf(transmit,sizeof(transmit),"AT+%s=%s,%s,1\r\n",name,ROBOT_WIFI_SSID,ROBOT_WIFI_PASSWORD);break;
 case SERVER_START:name="BEGINSERVER";n=snprintf(transmit,sizeof(transmit),"AT+%s=80\r\n",name);break;
 case HEALTH:name="GETSTATUS";n=snprintf(transmit,sizeof(transmit),"AT+%s?\r\n",name);break;
 case ACCEPT:name="SERVERAVAILABLE";break;
 case AVAILABLE:name="AVAILABLE";break;
 case RECEIVE:name="CLIENTRECEIVE";break;
 case SEND:name="CLIENTSEND";break;
 case CLOSE:name="CLIENTCLOSE";break;
 }
 if(!n){
  const int socket=op==ACCEPT?serverSocket:clientSocket;
  n=(op==RECEIVE||op==SEND)?snprintf(transmit,sizeof(transmit),"AT+%s=%d,%d\r\n",name,socket,amount):
   snprintf(transmit,sizeof(transmit),"AT+%s=%d\r\n",name,socket);
 }
 if(n<0 || (size_t)n>=sizeof(transmit)){scheduleRestart();return;}
 char expected[40];snprintf(expected,sizeof(expected),"+%s:",name);reply.begin(op==SEND?"":expected);
 txSize=(size_t)n;txOffset=0;
 if(op==SEND){
  for(int j=0;j<amount;++j){size_t index=responseOffset+j;transmit[txSize++]=index<headerSize?responseHeader[index]:responseBody[index-headerSize];}
  sendingSize=amount;
 }
 started=millis();pending=true;
}
void status(bool ok,uint32_t returnedToken){
 char line[420];lineWirelessStatus(line,sizeof(line));
 const auto& pose=odometry.pose;
 snprintf(statusBody,sizeof(statusBody),"{\"ok\":%s,\"token\":%lu,\"owner\":\"%s\",\"state\":\"%s\",\"mode\":\"%s\",\"ir\":%u,\"pwm\":[%d,%d],\"bootCause\":\"%s\",\"resetFlags\":[%u,%u],\"radioRestarts\":%lu,\"radioMaxUs\":%lu,\"ms\":%lu,\"poseEpoch\":%lu,\"poseReady\":%s,\"pose\":[%.1f,%.1f,%.4f],%s}",
  ok?"true":"false",(unsigned long)returnedToken,controlSession?"USB":wirelessControlToken()?"Wi-Fi":"none",
  navigationState,modeName(),lineMask(),motorLeft,motorRight,bootResetCause,(unsigned)bootFlags0,(unsigned)bootFlags1,(unsigned long)radioRestartAttempts,(unsigned long)wifiMaxUs,
  (unsigned long)millis(),(unsigned long)odometryEpoch,
  encodersEnabled()&&odometry.configured&&odometry.sampled?"true":"false",
  pose.x,pose.y,pose.heading,line);
 if (radarReady()) {
  radarWirelessStatus(line, sizeof(line));
  size_t used = strlen(statusBody);
  const size_t extra = strlen(line);
  if (used && statusBody[used-1] == '}' && extra && used + extra + 2 <= sizeof(statusBody)) {
   statusBody[used-1] = ',';
   memcpy(statusBody + used, line, extra);
   statusBody[used+extra] = '}';
   statusBody[used+extra+1] = 0;
  }
 }
}
void prepareResponse(){
 bool ok=true;const bool page=request.get("/");const char*type="application/json";
 if(page){responseBody=reinterpret_cast<const char*>(wirelessPageGzip);type="text/html; charset=utf-8";}
#if ROBOT_OTA_CONFIGURED
 else if(request.get("/firmware")){
  firmwareInfo(statusBody,sizeof(statusBody));responseBody=statusBody;
 }
 else if(const char* manifest=request.firmwareManifest()){
  if(firmwareAuthorized(manifest)){
   receiveFirmware(clientSocket,manifest);
   beginOperation(CLOSE);
   return;
  }
  ok=false;responseBody="{\"error\":\"update refused: stop robot, release control and check signing key\"}";
 }
#endif
 else if(request.get("/status")){status(true,0);responseBody=statusBody;}
 else{
  char command[96];ok=request.command(command);uint32_t token=0;
  if(ok){bool claim=!strcmp(command,"CLAIM");auto supplied=request.token();
   ok=processWirelessCommand(command,supplied);
   if((ok&&claim)||(supplied&&supplied==wirelessControlToken()))token=wirelessControlToken();
  }
  status(ok,token);responseBody=statusBody;
 }
 bodySize=page?sizeof(wirelessPageGzip):strlen(responseBody);responseOffset=0;
 headerSize=snprintf(responseHeader,sizeof(responseHeader),
  "HTTP/1.1 %s\r\nConnection: close\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\nContent-Type: %s\r\nContent-Length: %u\r\n%s\r\n",
  ok?"200 OK":"409 Conflict",type,(unsigned)bodySize,page?"Content-Encoding: gzip\r\n":"");
 beginOperation(SEND,(int)((headerSize+bodySize)<256?headerSize+bodySize:256));
}
void advance(){
 if(!reply.ok){
  if(reply.error&&clientSocket>=0&&operation!=CLOSE)beginOperation(CLOSE);
  else radioFailure();
  return;
 }
 switch(operation){
 case RESET_RADIO:beginOperation(AP_MODE);break;
 case AP_MODE:beginOperation(AP_ADDRESS);break;
 case AP_ADDRESS:beginOperation(AP_START);break;
 case AP_START:
  if(atoi(reply.data)!=1){scheduleRestart();return;}
  beginOperation(SERVER_START);break;
 case SERVER_START:{
  char* end=nullptr;
  long socket=strtol(reply.data,&end,10);
  if(end==reply.data || *end || socket<0 || socket>INT_MAX){scheduleRestart();return;}
  serverSocket=(int)socket;apReady=true;failures=0;nextPoll=millis();nextHealth=millis()+5000;
  Serial.println(F("WIFI_READY http://192.168.4.1"));break;}
 case HEALTH:{
  const int state=atoi(reply.data);
  if(state!=WL_AP_LISTENING && state!=WL_AP_CONNECTED){
   scheduleRestart();return;
  }
  nextPoll=millis();break;}
 case ACCEPT:
  failures=0;
  clientSocket=atoi(reply.data);
  if(clientSocket<0){nextPoll=millis()+20;return;}
  clientSince=millis();request.reset();beginOperation(AVAILABLE);break;
 case AVAILABLE:{
  int count=atoi(reply.data);
  if(count>0)beginOperation(RECEIVE,count>256?256:count);
  else if((uint32_t)(millis()-clientSince)>=1000)beginOperation(CLOSE);
  else nextPoll=millis()+10;
  break;}
 case RECEIVE:
  for(size_t j=0;j<reply.used;++j)request.feed(reply.data[j]);
  if(request.complete||request.bad)prepareResponse();
  else if((uint32_t)(millis()-clientSince)>=1000)beginOperation(CLOSE);
  else beginOperation(AVAILABLE);
  break;
 case SEND:
  responseOffset+=sendingSize;
  if(responseOffset<headerSize+bodySize){size_t remaining=headerSize+bodySize-responseOffset;beginOperation(SEND,remaining>256?256:remaining);}
  else beginOperation(CLOSE);
  break;
 case CLOSE:clientSocket=-1;request.reset();nextPoll=millis();break;
 }
}
void step(){
 if(!apReady && !pending){
  if(!retryEnabled)return;
  for(int n=0;n<128&&Serial2.available();++n){Serial2.read();}
  if((int32_t)(millis()-nextRestart)>=0)beginOperation(RESET_RADIO);
  return;
 }
 if(resync){
  for(int n=0;n<128&&Serial2.available();++n){Serial2.read();nextPoll=millis()+250;}
  if((int32_t)(millis()-nextPoll)<0)return;
  resync=false;beginOperation(clientSocket>=0?CLOSE:ACCEPT);return;
 }
 if(!pending){
  if((int32_t)(millis()-nextPoll)<0)return;
  if(clientSocket<0 && (int32_t)(millis()-nextHealth)>=0){
   nextHealth=millis()+5000;beginOperation(HEALTH);
  }else beginOperation(clientSocket<0?ACCEPT:AVAILABLE);
  return;
 }
 if(txOffset<txSize){
  size_t n=txSize-txOffset;if(n>16)n=16;
  Serial2.write((uint8_t*)transmit+txOffset,n);txOffset+=n;
 }
 for(int n=0;n<128&&Serial2.available()&&!reply.done;++n)reply.feed((char)Serial2.read());
 if(reply.done){pending=false;advance();}
 else if((uint32_t)(millis()-started)>=2000)radioFailure();
}
}
void setupWireless(){
 if(strlen(ROBOT_WIFI_PASSWORD)<8||strlen(ROBOT_WIFI_PASSWORD)>63){Serial.println(F("WIFI_DISABLED configure WirelessSecrets.h"));return;}
 retryEnabled=true;
 WiFi.config(IPAddress(192,168,4,1));
 if(WiFi.beginAP(ROBOT_WIFI_SSID,ROBOT_WIFI_PASSWORD)!=WL_AP_LISTENING){Serial.println(F("WIFI_AP_RETRY"));scheduleRestart();return;}
 std::string result;
 if(!modem.write("+BEGINSERVER:",result,"AT+BEGINSERVER=80\r\n")){Serial.println(F("WIFI_SERVER_FAILED"));scheduleRestart();return;}
 serverSocket=atoi(result.c_str());apReady=serverSocket>=0;
 if(!apReady){scheduleRestart();return;}
 failures=0;nextHealth=millis()+5000;nextPoll=millis();
 Serial.println(F("WIFI_READY http://192.168.4.1"));
}
void serviceWireless(){uint32_t start=micros();step();uint32_t elapsed=micros()-start;if(elapsed>wifiMaxUs)wifiMaxUs=elapsed;}
void reportWireless(){
 Serial.print(apReady?F("WIFI_READY http://192.168.4.1 build=sticky-line-v1 max_service_us="):F("WIFI_DISABLED max_service_us="));Serial.print(wifiMaxUs);
 Serial.print(F(" boot="));Serial.print(bootResetCause);
 Serial.print(F(" flags="));Serial.print((unsigned)bootFlags0);Serial.print(',');Serial.print((unsigned)bootFlags1);
 Serial.print(F(" radio_restarts="));Serial.println(radioRestartAttempts);
}
void suspendWireless(){} // Runtime transport remains incremental in every mode.
#else
void reportWireless(){Serial.println(F("WIFI_DISABLED"));}
void setupWireless(){}
void serviceWireless(){}
void suspendWireless(){}
#endif
