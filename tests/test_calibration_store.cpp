#include "../Robot/CalibrationStore.h"
#include <cassert>
#include <cstring>
struct Memory {
 unsigned char bytes[8192];Memory(){memset(bytes,255,sizeof(bytes));}
 template<class T>void get(int n,T&v){memcpy(&v,bytes+n,sizeof(v));}
 template<class T>void put(int n,const T&v){memcpy(bytes+n,&v,sizeof(v));}
};
struct Data{float wheelScale;int power;};
struct TornMemory:Memory{
 size_t limit=8192;
 template<class T>void put(int n,const T&v){memcpy(bytes+n,&v,limit<sizeof(v)?limit:sizeof(v));}
};
int main(){
 Memory m;Data d{5.1f,255},out{};assert(!persistent::load(m,out));
 assert(persistent::save(m,d));assert(persistent::load(m,out));assert(out.power==255&&out.wheelScale==5.1f);
 d.power=200;assert(persistent::save(m,d));assert(persistent::load(m,out)&&out.power==200);
 // Corrupt newest slot: retain previous complete calibration.
 m.bytes[4096+20]^=0x40;assert(persistent::load(m,out)&&out.power==255);
 memset(m.bytes,0,4096);assert(!persistent::load(m,out));
 // Every partial write preserves the previous complete slot.
 for(size_t cut=0;cut<sizeof(persistent::Record<Data>);++cut){
  TornMemory t;Data old{2,111};assert(persistent::save(t,old));t.limit=cut;
  persistent::save(t,d);assert(persistent::load(t,out)&&out.power==111);
 }

}
