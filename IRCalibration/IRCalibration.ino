// UNO R4 WiFi: middle IR module OUT -> D2, VCC -> regulated 5V, GND -> GND.
// Serial Monitor: 115200 baud. Send T while over tape to label the readings.
const uint8_t IR_PIN = 2;
const uint8_t MOTOR_PINS[] = {5, 6, 7, 8, 9, 10};
int tapeLevel = -1;
uint32_t lastLog = 0;

void setup() {
  // Disable both L298N channels and hold all direction inputs low.
  for (uint8_t pin : MOTOR_PINS) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  pinMode(IR_PIN, INPUT);
  Serial.begin(115200);
  Serial.println("IR calibration: motors OFF. Place sensor over tape and send T.");
}

void loop() {
  while (Serial.available()) {
    char command = Serial.read();
    if (command == 't' || command == 'T') {
      tapeLevel = digitalRead(IR_PIN);
      Serial.print("Tape recorded as ");
      Serial.println(tapeLevel == HIGH ? "HIGH (1)" : "LOW (0)");
      Serial.println("Move to ground: raw reading must change. Adjust sensor screw if needed.");
    } else if (command == 'r' || command == 'R') {
      tapeLevel = -1;
      Serial.println("Labels reset. Place over tape and send T.");
    }
  }
  if ((uint32_t)(millis() - lastLog) >= 200) {
    lastLog = millis();
    int raw = digitalRead(IR_PIN);
    Serial.print("IR middle D2 | raw=");
    Serial.print(raw);
    Serial.print(raw == HIGH ? " HIGH | " : " LOW  | ");
    if (tapeLevel < 0) Serial.println("UNLABELLED: place over tape, send T");
    else Serial.println(raw == tapeLevel ? "TAPE" : "GROUND");
  }
}
