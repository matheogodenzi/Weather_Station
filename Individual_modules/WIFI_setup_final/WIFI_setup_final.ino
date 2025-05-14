#include <ESP8266WiFi.h>

WiFiServer server(23);  // Port 23 for TCP communication (or any other port)
WiFiClient client;

void setup() {
  Serial.begin(115200);    // Communication with Teensy via UART
  WiFi.softAP("TeensyBridge", "12345678"); // Set up the Wi-Fi Access Point
  server.begin();
  server.setNoDelay(true);
  Serial.println("WiFi AP started. Waiting for connection...");
}

void loop() {
  // Check if the Wi-Fi client is connected
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      Serial.println("Client connected");
    }
  }

  // From Wi-Fi (PC) to Teensy:
  if (client && client.connected()) {
    while (client.available()) {
      // Read data from Wi-Fi and send to Teensy over UART (TX/RX)
      char c = client.read();
      Serial.write(c);
    }

    // From Teensy to Wi-Fi (PC):
    while (Serial.available()) {
      // Read data from Teensy (RX/TX) and send to Wi-Fi (PC)
      char c = Serial.read();
      client.write(c);
    }
  }
}
