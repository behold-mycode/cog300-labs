#pragma once
#include <deque>
#include <string>
#include <sstream>
#define WL_AP_LISTENING 7
static int modemCalls=0;
static bool pendingWeb=false;
static std::deque<char> webBytes;
static std::ostringstream webOutput;
struct IPAddress{IPAddress(int,int,int,int){}};
struct FakeWiFi {void config(IPAddress){++modemCalls;}int beginAP(const char*,const char*){++modemCalls;return WL_AP_LISTENING;}} WiFi;
struct WiFiClient{
 bool active=false;
 operator bool(){++modemCalls;return active;}
 int available(){++modemCalls;return webBytes.size();}
 int read(){++modemCalls;char c=webBytes.front();webBytes.pop_front();return c;}
 void stop(){++modemCalls;active=false;webBytes.clear();}
 template<class T>void print(T x){++modemCalls;webOutput<<x;}
 void print(float x,int){print(x);}
 template<class T>void println(T x){print(x);print('\n');}
 void println(){print('\n');}
};
struct WiFiServer{
 WiFiServer(int){}void begin(){++modemCalls;}
 void end(){++modemCalls;pendingWeb=false;webBytes.clear();}
 WiFiClient available(){++modemCalls;WiFiClient c;c.active=pendingWeb;pendingWeb=false;return c;}
};
