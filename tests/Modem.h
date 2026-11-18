#pragma once
#include <string>
struct FakeModem {
 bool write(const std::string&,std::string&result,const char*,...){++modemCalls;result="0";return true;}
} modem;
