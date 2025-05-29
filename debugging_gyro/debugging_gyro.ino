#include <Wire.h>

void setup() {
  Wire1.begin();
  Serial.begin(9600);
  while (!Serial);  // Wait for Serial Monitor

  Serial.println("Scanning I2C on Wire1...");
  byte count = 0;
  for (byte addr = 1; addr < 127; ++addr) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found on Wire1.");
  }
}

void loop() {}

