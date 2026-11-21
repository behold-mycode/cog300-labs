#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
namespace wireless {
// One bounded HTTP/1 request. No chunked bodies, pipelining, or URL-decoding.
class Request {
 char bytes[1024]{};size_t used=0,header=0,length=0;
 bool control=false;uint32_t sessionToken=0;
 static bool nameIs(const char* begin,const char* end,const char* name){
  if((size_t)(end-begin)!=strlen(name))return false;
  while(begin<end){char c=*begin++;if(c>='A'&&c<='Z')c+=32;if(c!=*name++)return false;}return true;
 }
 bool decimal(const char*begin,const char*end,uint32_t&value){
  if(begin==end)return false;value=0;
  for(auto p=begin;p<end;++p){if(*p<'0'||*p>'9'||value>(UINT32_MAX-(*p-'0'))/10)return false;value=value*10+(*p-'0');}
  return true;
 }
 public:
 bool complete=false,bad=false;
 void reset(){used=header=length=0;complete=bad=false;bytes[0]=0;control=false;sessionToken=0;}
 void feed(char c){
  if(complete||bad)return;
  if(!c||used>=sizeof(bytes)-1){bad=true;return;}
  bytes[used++]=c;bytes[used]=0;
  if(!header&&used>=4&&!memcmp(bytes+used-4,"\r\n\r\n",4)){
   header=used;char *p=strstr(bytes,"\r\n")+2;bool haveLength=false,haveControl=false,haveToken=false;
   while(*p!='\r'){
    char *end=strstr(p,"\r\n"),*colon=strchr(p,':');
    if(!end||!colon||colon>=end){bad=true;return;}
    char*value=colon+1;while(value<end&&(*value==' '||*value=='\t'))++value;
    char*tail=end;while(tail>value&&(tail[-1]==' '||tail[-1]=='\t'))--tail;
    uint32_t number=0;
    if(nameIs(p,colon,"content-length")){
     if(haveLength||!decimal(value,tail,number)||number>95){bad=true;return;}
     haveLength=true;length=number;
    }else if(nameIs(p,colon,"transfer-encoding")){bad=true;return;}
    else if(nameIs(p,colon,"x-robot-control")){
     if(haveControl){bad=true;return;}haveControl=true;control=tail-value==1&&*value=='1';
    }else if(nameIs(p,colon,"x-robot-token")){
     if(haveToken||!decimal(value,tail,sessionToken)){bad=true;return;}haveToken=true;
    }
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
 uint32_t token()const{return sessionToken;}
 bool command(char*out)const{
  if(!complete||bad||!length||strncmp(bytes,"POST /command HTTP/1.1\r\n",24))return false;
  if(!control)return false;
  const char*b=bytes+header;
  for(size_t j=0;j<length;++j)if(b[j]<32||b[j]>126)return false;
  // Explicit allowlist prevents network joystick/test commands and session changes.
  const char*allowed[]={"LINECFG ","LINESPEED ","LINEMOTOR ", "RADAR STEP ","RADAR PROFILE ","RADAR SERVO ","RADAR CLEARANCE "};
  bool ok=!strcmp(b,"STOP")||!strcmp(b,"CLAIM")||!strcmp(b,"RELEASE")||!strcmp(b,"HEARTBEAT")||!strcmp(b,"MODE LINE")
      ||!strcmp(b,"MODE CALIBRATE")||!strcmp(b,"RADAR ARM")||!strcmp(b,"RADAR SCAN");
  for(auto a:allowed)if(!strncmp(b,a,strlen(a)))ok=true;
  if(!ok)return false;
  memcpy(out,b,length+1);return true;
 }
};
}
