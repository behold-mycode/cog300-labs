#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

namespace wireless {
// Incremental parser for the installed UNO R4 ESP bridge's AT response format.
// Binary receive bodies are length framed: embedded OK/CRLF are ordinary bytes.
class Reply {
 enum Phase { PREFIX, VALUE, PAYLOAD, RESULT } phase=PREFIX;
 char prefix[40]{},line[48]{};size_t pos=0,left=0;
 void fail(){done=true;ok=false;}
 public:
 char data[257]{};size_t used=0;bool done=false,ok=false,error=false;
 void begin(const char*expected){
  phase=*expected?PREFIX:RESULT;pos=left=used=0;done=ok=error=false;data[0]=line[0]=0;
  strncpy(prefix,expected,sizeof(prefix)-1);
 }
 void feed(char c){
  if(done)return;
  if(phase==PREFIX){
   if(!pos&&(c=='\r'||c=='\n'))return;
   if(!pos&&c=='E'){phase=RESULT;line[0]='E';line[1]=0;pos=1;return;}
   if(c!=prefix[pos]){fail();return;}
   if(!prefix[++pos]){phase=VALUE;pos=0;}return;
  }
  if(phase==PAYLOAD){
   data[used++]=c;data[used]=0;
   if(!--left){phase=RESULT;pos=0;}return;
  }
  if(phase==VALUE){
   if(c=='|' ){
    char*end;long n=strtol(data,&end,10);
    if(end==data||*end||n<0||n>256){fail();return;}
    used=0;data[0]=0;left=(size_t)n;phase=left?PAYLOAD:RESULT;pos=0;return;
   }
   if(c=='\r')return;
   if(c=='\n'){
    if(strstr(data,"ERROR")){fail();return;}
    phase=RESULT;pos=0;return;
   }
   if(used==256){fail();return;}data[used++]=c;data[used]=0;return;
  }
  if(c=='\r')return;
  if(c=='\n'){
   if(!pos)return;done=true;ok=!strcmp(line,"OK");error=!strcmp(line,"ERROR");return;
  }
  if(pos>=sizeof(line)-1){fail();return;}line[pos++]=c;line[pos]=0;
 }
};
}
