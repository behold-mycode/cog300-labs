#include <stdint.h>
#include <stddef.h>
#include <cassert>
#include <string>
#define ARDUINO_ARCH_RENESAS_UNO
#define NO_USB
class Print{public:virtual size_t write(uint8_t)=0;};
struct BridgePort{
 std::string output;void begin(unsigned long){}operator bool(){return true;}
 int available(){return 0;}int read(){return -1;}
 int availableForWrite(){return 0;} // Actual core UART inherits this default.
 size_t write(uint8_t c){output+=(char)c;return 1;}
} Serial;
#define Serial Serial
#include "../Robot/SerialOutput.h"
RobotSerialPort robotSerial;
int main(){
 for(char c:std::string("DATA,0,MANUAL,STOPPED,0,0\n"))robotSerial.write(c);
 assert(serialOutputReady());
 serviceSerialOutput();
 #undef Serial
 assert(Serial.output.size()==16);
 serviceSerialOutput();assert(Serial.output=="DATA,0,MANUAL,STOPPED,0,0\n");
 for(int n=0;n<100;++n)for(char c:std::string("DATA,0,0\n"))robotSerial.write(c);
 assert(!serialOutputReady());for(int n=0;n<100;++n)serviceSerialOutput();assert(serialOutputReady());
}
