#include "Robot.h"
int lineTapeLevel=-1;
void setupLine() { pinMode(LINE_LEFT_PIN,INPUT);pinMode(LINE_CENTER_PIN,INPUT);pinMode(LINE_RIGHT_PIN,INPUT); }
uint8_t lineMask() {
  if(lineTapeLevel<0) return 0;
  return (digitalRead(LINE_LEFT_PIN)==lineTapeLevel?4:0)|
         (digitalRead(LINE_CENTER_PIN)==lineTapeLevel?2:0)|
         (digitalRead(LINE_RIGHT_PIN)==lineTapeLevel?1:0);
}
void reportLine() {
  Serial.print(F("IR,"));Serial.print(digitalRead(LINE_LEFT_PIN));Serial.print(',');
  Serial.print(digitalRead(LINE_CENTER_PIN));Serial.print(',');
  Serial.print(digitalRead(LINE_RIGHT_PIN));Serial.print(',');Serial.println(lineTapeLevel);
}
