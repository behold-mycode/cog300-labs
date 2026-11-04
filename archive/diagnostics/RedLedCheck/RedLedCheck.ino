// Standalone bench diagnostic for the documented Uno R4 WiFi wiring.
// Holds red D12 HIGH and green D11 LOW, independent of the distance sensor.
// Motors remain disabled. Upload the V2 robot sketch again to resume driving.
#include <Arduino.h>
void setup() {
  digitalWrite(9, LOW); digitalWrite(10, LOW);
  pinMode(9, OUTPUT); pinMode(10, OUTPUT);
  for (uint8_t pin = 5; pin <= 8; ++pin) {
    digitalWrite(pin, LOW); pinMode(pin, OUTPUT);
  }
  digitalWrite(11, LOW); pinMode(11, OUTPUT);
  digitalWrite(12, HIGH); pinMode(12, OUTPUT);
  Serial.begin(9600);
  Serial.println("LED CHECK: D12 HIGH (red on), D11 LOW (green off), motors disabled.");
}
void loop() {}
