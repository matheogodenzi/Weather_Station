#include <ESP8266WiFi.h>
WiFiServer server(23);  // Port 23 for raw TCP

WiFiClient client;

void setup() {
  Serial.begin(74880); // UART to Teensy
  WiFi.softAP("TeensyBridge", "12345678"); // SSID and Password
  server.begin();
  server.setNoDelay(true);
  Serial.println("WiFi AP started. Waiting for connection...");
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
    return;
  }

  // From Wi-Fi to Teensy
  while (client.available()) {
    Serial.write(client.read());
  }

  // From Teensy to Wi-Fi
  while (Serial.available()) {
    client.write(Serial.read());
  }
}
