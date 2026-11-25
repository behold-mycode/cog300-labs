#pragma once
#include <deque>
#include <string>
#include <sstream>
#include <cassert>
#define WL_AP_LISTENING 7
#define WL_AP_CONNECTED 8
static int radioState=WL_AP_LISTENING;
static int modemCalls=0;
static std::deque<std::string> webRequests;
static std::string webOutput;
static bool radioSilent=false;
struct IPAddress{IPAddress(int,int,int,int){}};
struct FakeWiFi {void end(){++modemCalls;}void config(IPAddress){++modemCalls;}int beginAP(const char*,const char*){++modemCalls;return radioState;}} WiFi;
struct FakeRadio {
 std::deque<char> rx;std::string tx,body;int payload=0,remaining=0;size_t maxWrite=0;
 void respond(const std::string&s){if(!radioSilent)for(char c:s)rx.push_back(c);}
 void command(){
  if(tx=="AT+GETSTATUS?\r\n"){tx.clear();respond("+GETSTATUS: "+std::to_string(radioState)+"\r\nOK\r\n");return;}
  if(tx=="AT+SOFTRESETWIFI\r\n"){tx.clear();respond("+SOFTRESETWIFI:\r\nOK\r\n");return;}
  auto equal=tx.find('=');std::string name=tx.substr(2,equal-2),argument=tx.substr(equal+1);tx.clear();
  int amount=0;auto comma=argument.find(',');if(comma!=std::string::npos)amount=atoi(argument.c_str()+comma+1);
  if(name=="+WIFIMODE" || name=="+SOFTAPCONFIG"){respond(name+":\r\nOK\r\n");return;}
  if(name=="+BEGINSOFTAP"){respond(name+": "+(radioState==WL_AP_LISTENING?"1":"0")+"\r\nOK\r\n");return;}
  if(name=="+BEGINSERVER"){respond(name+": 0\r\nOK\r\n");return;}
  if(name=="+SERVERAVAILABLE"){
   if(body.empty()&&!webRequests.empty()){body=webRequests.front();webRequests.pop_front();}
   respond(name+": "+(body.empty()?"-1":"1")+"\r\nOK\r\n");
  }else if(name=="+AVAILABLE")respond(name+": "+std::to_string(body.size())+"\r\nOK\r\n");
  else if(name=="+CLIENTRECEIVE"){
   auto part=body.substr(0,amount);body.erase(0,part.size());respond(name+": "+std::to_string(part.size())+"|"+part+"OK\r\n");
  }else if(name=="+CLIENTSEND"){payload=remaining=amount;}
  else if(name=="+CLIENTCLOSE"){body.clear();respond(name+":\r\nOK\r\n");}
  else assert(false);
 }
 size_t write(uint8_t*bytes,size_t n){
  if(n>maxWrite)maxWrite=n;assert(n<=16);
  for(size_t j=0;j<n;++j){char c=bytes[j];if(remaining){webOutput+=c;if(!--remaining)respond("OK\r\n");}
   else {tx+=c;if(c=='\n')command();}}
  return n;
 }
 int available(){return rx.size();}int read(){char c=rx.front();rx.pop_front();return c;}
} Serial2;
