#pragma once
#include <cstring>
struct EEPROMClass {
 unsigned char bytes[8192];EEPROMClass(){memset(bytes,255,sizeof(bytes));}
 template<class T>void get(int n,T&v){memcpy(&v,bytes+n,sizeof(v));}
 template<class T>void put(int n,const T&v){memcpy(bytes+n,&v,sizeof(v));}
 int length(){return 8192;}
} EEPROM;
