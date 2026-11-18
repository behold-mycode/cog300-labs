#include "../Robot/AsyncRadio.h"
#include <cassert>
#include <string>
static void feed(wireless::Reply&r,const std::string&s){for(char c:s)r.feed(c);}
int main(){
 wireless::Reply r;r.begin("");feed(r,"OK\r\n");assert(r.done&&r.ok);
 r.begin("");feed(r,"ERROR\r\n");assert(r.done&&!r.ok);
 r.begin("+SERVERAVAILABLE:");feed(r,"+SERVERAVAILABLE: 3\r\nOK\r\n");assert(r.done&&r.ok&&std::string(r.data)==" 3");
 r.begin("+CLIENTRECEIVE:");std::string body="GET / HTTP/1.1\r\n\r\n";
 feed(r,"+CLIENTRECEIVE: "+std::to_string(body.size())+"|"+body+"OK\r\n");assert(r.done&&r.ok&&std::string(r.data,r.used)==body);
 r.begin("+CLIENTRECEIVE:");feed(r,"+CLIENTRECEIVE: 0|OK\r\n");assert(r.done&&r.ok&&r.used==0);
 r.begin("+CLIENTRECEIVE:");feed(r,"+CLIENTRECEIVE: 999|bad");assert(r.done&&!r.ok);
 r.begin("+AVAILABLE:");feed(r,"+AVAILABLE: ERROR\r\n");assert(r.done&&!r.ok);
 r.begin("+AVAILABLE:");feed(r,"+WRONG: 1\r\nOK\r\n");assert(r.done&&!r.ok);
 r.begin("+CLIENTRECEIVE:");feed(r,"+CLIENTRECEIVE: 7|OK\r\nXYZOK\r\n");assert(r.done&&r.ok&&r.used==7);
}
