void setup() {
  Serial.begin(115200);    // For communication with your PC
  Serial2.begin(115200);   // For communication with Wemos (TX/RX)
  Serial.println("Teensy is ready. Transmitting data to Wemos...");
}

void loop() {
  // Example: send data from Teensy to Wemos (over UART)
  Serial2.println("Hello from Teensy!");

  delay(1000);  // Send data every 1 second (adjust as needed)
}
