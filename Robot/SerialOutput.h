#pragma once
#if defined(ARDUINO_ARCH_RENESAS_UNO)
#include "SerialQueue.h"
// Keep protocol printers, but bound each loop's serial work. UNO R4 WiFi USB
// travels through the ESP bridge UART (NO_USB), not the core SerialUSB object.
class RobotSerialPort:public Print {
 serialio::Queue queue;
 public:
 void begin(unsigned long baud){Serial.begin(baud);}
 operator bool(){return (bool)Serial;}
 int available(){return Serial.available();}
 int read(){return Serial.read();}
 using Print::write;
 size_t write(uint8_t c)override{
  if(Serial)queue.write((char)c);else queue.clear();return 1;
 }
 bool ready(){return queue.available()<=256;}
 void service(){
  if(!Serial){queue.clear();return;}
  int space=16; // Bridge UART: ~1.4ms, no network/host acknowledgement.
#ifndef NO_USB
  // Native USB builds must not enter SerialUSB's wait-for-room path.
  int available=Serial.availableForWrite();if(space>available)space=available;
#endif
  while(space-->0&&queue.available()){
   if(!Serial.write((uint8_t)queue.front()))break;queue.pop();
  }
 }
};
extern RobotSerialPort robotSerial;
inline void serviceSerialOutput(){robotSerial.service();}
inline bool serialOutputReady(){return robotSerial.ready();}
#undef Serial
#define Serial robotSerial
#else
inline void serviceSerialOutput(){}
inline bool serialOutputReady(){return true;}
#endif
