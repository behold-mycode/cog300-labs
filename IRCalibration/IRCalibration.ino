// UNO R4 WiFi: left/centre/right floor sensor OUT -> D3/D2/D4.
// Motors remain disabled. Serial Monitor: 115200 baud, any line ending.
const uint8_t IR_PINS[] = {3, 2, 4};
const uint8_t MOTOR_PINS[] = {5, 6, 7, 8, 9, 10};
int tapeLevels[] = {-1, -1, -1};
uint32_t lastLog = 0;
void setup() {
  for (uint8_t pin : MOTOR_PINS) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
  for (uint8_t pin : IR_PINS) pinMode(pin, INPUT);
  Serial.begin(115200);
  Serial.println("All three sensors: put ALL over white tape, send T. Then ALL over floor, send G.");
}
void loop() {
  while (Serial.available()) {
    char command = Serial.read();
    if (command == 't' || command == 'T') {
      for (int n=0;n<3;++n) tapeLevels[n]=digitalRead(IR_PINS[n]);
      if(tapeLevels[0]!=tapeLevels[1]||tapeLevels[1]!=tapeLevels[2])
        Serial.println("Tape levels differ: adjust/check modules. Robot expects matching tape polarity.");
      else Serial.println("Tape recorded. Move ALL sensors over floor; send G.");
    } else if (command == 'g' || command == 'G') {
      for(int n=0;n<3;++n){
        Serial.print("D");Serial.print(IR_PINS[n]);
        Serial.println(tapeLevels[n]<0?": record tape with T first":digitalRead(IR_PINS[n])!=tapeLevels[n]?": tape/floor differ OK":": NO CONTRAST - adjust height/threshold/wiring");
      }
    }
  }
  if ((uint32_t)(millis()-lastLog)>=200) {
    lastLog=millis();Serial.print("Left D3 / Centre D2 / Right D4: ");
    for(int n=0;n<3;++n){Serial.print(digitalRead(IR_PINS[n]));Serial.print(' ');}
    Serial.println();
  }
}
