// === Teensy 4.0 + SPS30 over UART (Serial6) ===
// RX5 (Pin 33) ← Sensor TX
// TX5 (Pin 34) → Sensor RX

// SPS30 start measurement command (UART, continuous measurement, no arguments)
const byte START_MEAS_CMD[] = { 0x7E, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01, 0x03, 0xF9, 0x7E };

// SPS30 read measurement command (UART)
const byte READ_DATA_CMD[] = { 0x7E, 0x00, 0x03, 0x00, 0x00, 0xFC, 0x07, 0x81, 0x7E };

void setup() {
  Serial.begin(115200);      // USB Serial for debugging
  Serial6.begin(115200);     // Serial6 (UART) for SPS30
  while (!Serial);           // Wait for Serial Monitor

  delay(1000);               // Let everything power up

  Serial.println("SPS30 UART (Serial6) communication started");

  // Send start measurement command
  sendCommand(START_MEAS_CMD, sizeof(START_MEAS_CMD));
  Serial.println("Start measurement command sent");

  delay(1000); // Wait for sensor to begin measurements
}

void loop() {
  sendCommand(READ_DATA_CMD, sizeof(READ_DATA_CMD));
  delay(1000); // Give sensor time to respond

  if (Serial6.available()) {
    byte buffer[100];
    int len = Serial6.readBytes(buffer, sizeof(buffer));

    if (len >= 60) {
      float pm1   = parseFloat(&buffer[10]);
      float pm2_5 = parseFloat(&buffer[16]);
      float pm4   = parseFloat(&buffer[22]);
      float pm10  = parseFloat(&buffer[28]);

      Serial.print("PM1.0 : "); Serial.print(pm1); Serial.println(" µg/m³");
      Serial.print("PM2.5 : "); Serial.print(pm2_5); Serial.println(" µg/m³");
      Serial.print("PM4.0 : "); Serial.print(pm4); Serial.println(" µg/m³");
      Serial.print("PM10  : "); Serial.print(pm10); Serial.println(" µg/m³");
      Serial.println("-------------------------");
    } else {
      Serial.print("Expected 60 bytes, got: ");
      Serial.println(len);
    }
  } else {
    Serial.println("No response from SPS30");
  }

  delay(2000); // Wait before next reading
}

void sendCommand(const byte *cmd, int len) {
  Serial6.write(cmd, len);
}

float parseFloat(const uint8_t *data) {
  union {
    uint8_t b[4];
    float f;
  } u;

  u.b[0] = data[0];
  u.b[1] = data[1];
  u.b[2] = data[2];
  u.b[3] = data[3];

  return u.f;
}
