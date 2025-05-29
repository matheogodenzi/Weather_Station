#include <SensirionUARTSPS30.h>

SensirionUARTSPS30 sps30;
HardwareSerial& spsSerial = Serial5;  // Use Serial5 for pins 20/21

void setup() {
  Serial.begin(115200);
  spsSerial.begin(115200);
  sps30.begin(spsSerial);

  if (sps30.probe() != 0) {
    Serial.println("SPS30 not found");
    while (1);
  }

  sps30.startMeasurement();
}

void loop() {
  float pm2_5;
  if (sps30.readMeasuredValues(pm2_5) == 0) {
    Serial.print("PM2.5: ");
    Serial.println(pm2_5);
  }
  delay(1000);
}
