#include "Robot.h"
int lineTapeLevel=-1;
void setupLine() { pinMode(LINE_LEFT_PIN,INPUT);pinMode(LINE_CENTER_PIN,INPUT);pinMode(LINE_RIGHT_PIN,INPUT); }
uint8_t lineMask() {
  if(lineTapeLevel<0) return 0;
#if defined(ARDUINO_UNOR4_WIFI)
  static_assert(LINE_LEFT_PIN==4 && LINE_CENTER_PIN==2 && LINE_RIGHT_PIN==3,"Update the UNO port mapping");
  // D4=P106, D2=P104, D3=P105. One snapshot cannot straddle a transition.
  uint16_t pins=R_PORT1->PIDR;
  if(lineTapeLevel==LOW)pins=~pins;
  return ((pins>>4)&4)|((pins>>3)&2)|((pins>>5)&1);
#else
  return (digitalRead(LINE_LEFT_PIN)==lineTapeLevel?4:0)|
         (digitalRead(LINE_CENTER_PIN)==lineTapeLevel?2:0)|
         (digitalRead(LINE_RIGHT_PIN)==lineTapeLevel?1:0);
#endif
}
void reportLine() {
  Serial.print(F("IR,"));Serial.print(digitalRead(LINE_LEFT_PIN));Serial.print(',');
  Serial.print(digitalRead(LINE_CENTER_PIN));Serial.print(',');
  Serial.print(digitalRead(LINE_RIGHT_PIN));Serial.print(',');Serial.println(lineTapeLevel);
}
