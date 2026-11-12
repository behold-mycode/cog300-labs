#pragma once
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
namespace wireless {
// One bounded HTTP/1 request. No chunked bodies, pipelining, or URL-decoding.
class Request {
 char bytes[1024]{};size_t used=0,header=0,length=0;
 public:
 bool complete=false,bad=false;
 void reset(){used=header=length=0;complete=bad=false;bytes[0]=0;}
 void feed(char c){
  if(complete||bad)return;
  if(!c||used>=sizeof(bytes)-1){bad=true;return;}
  bytes[used++]=c;bytes[used]=0;
  if(!header&&used>=4&&!memcmp(bytes+used-4,"\r\n\r\n",4)){
   header=used;char *p=strstr(bytes,"\r\n")+2;bool haveLength=false;
   while(*p!='\r'){
    char *end=strstr(p,"\r\n");if(!end){bad=true;return;}
    if(!strncmp(p,"Content-Length:",15)){
     if(haveLength){bad=true;return;}haveLength=true;char *q=p+15;while(*q==' ')++q;
     if(*q<'0'||*q>'9'){bad=true;return;}char *tail;length=strtoul(q,&tail,10);
     if(tail!=end||length>95){bad=true;return;}
    }
    // Reject alternate casing of framing headers rather than misparse them.
    else {char name[32]{};size_t n=0;while(p+n<end&&p[n]!=':'&&n<31){char x=p[n];name[n]=(x>='A'&&x<='Z')?x+32:x;++n;}
     if(!strcmp(name,"content-length")||!strcmp(name,"transfer-encoding")){bad=true;return;}}
    p=end+2;
   }
  }
  if(header&&used==header+length)complete=true;
 }
 bool get(const char*path)const{
  if(!complete||bad||length)return false;
  char line[80];if(strlen(path)>50)return false;
  strcpy(line,"GET ");strcat(line,path);strcat(line," HTTP/1.1\r\n");return !strncmp(bytes,line,strlen(line));
 }
 bool command(char*out)const{
  if(!complete||bad||!length||strncmp(bytes,"POST /command HTTP/1.1\r\n",24))return false;
  const char*h=strstr(bytes,"\r\nX-Robot-Control: 1\r\n");if(!h||h>=bytes+header)return false;
  const char*b=bytes+header;
  for(size_t j=0;j<length;++j)if(b[j]<32||b[j]>126)return false;
  // Explicit allowlist prevents network joystick/test commands and session changes.
  const char*allowed[]={"SONAR ","ENC ","ODOM ","BODY ","WALLCFG ","SPEEDCFG ","CONFIG ","RACECFG "};
  bool ok=!strcmp(b,"SAVE")||!strcmp(b,"LOAD")||!strcmp(b,"RUN START")||!strcmp(b,"MAP RESET");
  for(auto a:allowed)if(!strncmp(b,a,strlen(a)))ok=true;
  if(!ok)return false;
  memcpy(out,b,length+1);return true;
 }
};
}
