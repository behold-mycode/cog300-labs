#include "../Robot/WirelessHttp.h"
#include <cassert>
#include <string>
static wireless::Request parse(const std::string&s){wireless::Request r;for(char c:s)r.feed(c);return r;}
int main(){
 char out[96];auto r=parse("GET / HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n");assert(r.get("/"));
 auto post=[](std::string b){return parse("POST /command HTTP/1.1\r\nX-Robot-Control: 1\r\nContent-Length: "+std::to_string(b.size())+"\r\n\r\n"+b);};
 r=post("MODE LINE");assert(r.command(out)&&!strcmp(out,"MODE LINE"));
 r=post("DRIVE 255 255");assert(!r.command(out));r=post("CONFIG 255 255 1 0 20 10 1");assert(!r.command(out));
 r=parse("POST /command HTTP/1.1\r\nContent-Length: 9\r\n\r\nMODE LINE");assert(!r.command(out));
 r=post("STOP\nMODE LINE");assert(!r.command(out));
 r=parse(std::string(2000,'a'));assert(r.bad);
 r=parse("POST /command HTTP/1.1\r\nContent-Length: 4\r\nContent-Length: 4\r\n\r\nSAVE");assert(r.bad);
 r=parse("POST /command HTTP/1.1\r\nx-robot-control: 1\r\nx-robot-token: 123\r\ncontent-length: 4\r\n\r\nSTOP");assert(r.command(out)&&r.token()==123);
 r=parse("POST /command HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n");assert(r.bad);
 r=parse("POST /command HTTP/1.1\r\nContent-Length: 95\r\n\r\nSAVE");assert(!r.complete&&!r.command(out));
 r=parse("POST /command HTTP/1.1\r\nX-Robot-Control: 1\r\nX-Robot-Token: 12345\r\nContent-Length: 9\r\n\r\nHEARTBEAT");assert(r.command(out)&&r.token()==12345);
 r=post("LINESPEED 200 90 180 130");assert(r.command(out));
}
