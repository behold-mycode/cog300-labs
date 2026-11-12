#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace persistent {
constexpr uint32_t MAGIC=0x434F4738;
constexpr int SLOT=4096;
inline uint32_t crc(const void* data,size_t length){
 uint32_t c=0xffffffff;const auto*b=static_cast<const uint8_t*>(data);
 while(length--){c^=*b++;for(int k=0;k<8;++k)c=(c>>1)^((c&1)?0xedb88320u:0);}
 return ~c;
}
template<class T>struct Record {uint32_t magic=0,version=1,size=sizeof(T),sequence=0;T data{};uint32_t checksum=0;};
template<class T>bool valid(const Record<T>&r){return r.magic==MAGIC&&r.version==1&&r.size==sizeof(T)&&r.checksum==crc(&r,offsetof(Record<T>,checksum));}
template<class T>int newest(const Record<T>&a,const Record<T>&b){
 bool va=valid(a),vb=valid(b);if(!va&&!vb)return -1;
 return vb&&(!va||(int32_t)(b.sequence-a.sequence)>0)?1:0;
}
template<class Store,class T>bool load(Store&s,T&out){
 Record<T>a,b;s.get(0,a);s.get(SLOT,b);int n=newest(a,b);if(n<0)return false;out=n?b.data:a.data;return true;
}
template<class Store,class T>bool save(Store&s,const T&value){
 static_assert(sizeof(Record<T>)<SLOT,"Calibration exceeds storage slot");
 Record<T>a,b;s.get(0,a);s.get(SLOT,b);int n=newest(a,b),target=n==0?SLOT:0;
 Record<T>r{};r.magic=MAGIC;r.sequence=n<0?1:(n?b.sequence:a.sequence)+1;r.data=value;
 r.checksum=crc(&r,offsetof(Record<T>,checksum));
 // Slots are in separate flash regions. A torn write fails CRC; old slot survives.
 s.put(target,r);Record<T>check;s.get(target,check);return valid(check)&&check.sequence==r.sequence;
}
}
