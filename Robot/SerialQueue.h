#pragma once
#include <stddef.h>
namespace serialio {
// Queue complete protocol lines. When a reader stalls, drop whole records;
// never wait for it or splice fragments into the next record.
class Queue {
 char lines[1024]{},line[384]{};size_t head=0,count=0,length=0;bool overflow=false;
 public:
 void clear(){head=count=length=0;overflow=false;}
 void write(char c){
  if(length<sizeof(line))line[length++]=c;else overflow=true;
  if(c!='\n')return;
  if(!overflow&&length<=sizeof(lines)-count){
   for(size_t i=0;i<length;++i)lines[(head+count+i)%sizeof(lines)]=line[i];count+=length;
  }
  length=0;overflow=false;
 }
 size_t available()const{return count;}
 char front()const{return lines[head];}
 void pop(){head=(head+1)%sizeof(lines);--count;}
};
}
