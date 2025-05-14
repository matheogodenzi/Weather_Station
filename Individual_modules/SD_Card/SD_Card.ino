#include <SPI.h>
#include <SD.h>

const int chipSelect = 23;  // Adjust to your CS pin

void setup() {
  Serial.begin(9600);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");
  
  File dataFile = SD.open("test.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Hello from Teensy!");
    dataFile.close();
    Serial.println("Wrote to test.txt");
  } else {
    Serial.println("Error opening test.txt");
  }
}

void loop() {
  // Do nothing here
}
