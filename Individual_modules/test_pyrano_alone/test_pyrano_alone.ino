#include <Arduino.h>

HardwareSerial& pyranoSerial = Serial3; // Use Serial3 on Teensy

// Baud rates to test
const int baudRates[] = {4800, 9600, 19200, 38400};
const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);

// Function codes to try
const byte functionCodes[] = {0x03, 0x04}; // Read Holding / Input Registers
const int numFuncs = sizeof(functionCodes) / sizeof(functionCodes[0]);

// Address range to scan
const byte addrMin = 0x01;
const byte addrMax = 0x10;

// Register to read
const uint16_t registerAddress = 0x0000;
const uint16_t registerCount = 0x0001;

// === CRC-16 (Modbus) ===
uint16_t modbusCRC(const byte *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void sendTrame(byte addr, byte func, int baud) {
  byte request[8];
  request[0] = addr;
  request[1] = func;
  request[2] = (registerAddress >> 8) & 0xFF;
  request[3] = registerAddress & 0xFF;
  request[4] = (registerCount >> 8) & 0xFF;
  request[5] = registerCount & 0xFF;

  uint16_t crc = modbusCRC(request, 6);
  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;

  // Send request
  pyranoSerial.write(request, 8);
  pyranoSerial.flush();

  delay(20);

  // Read response
  unsigned long timeout = millis() + 200;
  bool gotResponse = false;
  Serial.print(" ← Response: ");
  while (millis() < timeout) {
    while (pyranoSerial.available()) {
      byte b = pyranoSerial.read();
      Serial.print("0x");
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
      gotResponse = true;
    }
  }

  if (!gotResponse) Serial.print("No response.");
  Serial.println();
}

void setup() {
  Serial.begin(74880);
  delay(2000);
  Serial.println("=== Modbus Pyranometer Scanner ===");
}

void loop() {
  for (int i = 0; i < numBaudRates; i++) {
    int baud = baudRates[i];
    pyranoSerial.begin(baud);
    delay(100);
    Serial.print("\n🔁 Testing baud rate: ");
    Serial.println(baud);

    for (byte addr = addrMin; addr <= addrMax; addr++) {
      for (int f = 0; f < numFuncs; f++) {
        byte func = functionCodes[f];

        Serial.print("→ Addr: 0x");
        if (addr < 0x10) Serial.print("0");
        Serial.print(addr, HEX);
        Serial.print(", Func: 0x");
        if (func < 0x10) Serial.print("0");
        Serial.print(func, HEX);

        sendTrame(addr, func, baud);
        delay(300);
      }
    }

    pyranoSerial.end();
    delay(1000);
  }

  Serial.println("\n✅ Scan complete. Restarting in 15s...\n");
  delay(15000);
}
