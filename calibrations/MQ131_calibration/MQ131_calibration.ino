#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <math.h>


const int mq131Pin = A9;       // A9 = pin 23 on Teensy 4.0
const float Vcc = 5.0;         // Sensor is powered with 5V
const float ADCref = 3.3;      // ADC reference voltage on Teensy
const float RL = 10000.0;      // Assumed 10kΩ load resistor

void setup() {
  Serial.begin(9600);
  delay(5000); // Let readings stabilize

  float R0 = calibrateR0();
  Serial.print("R0 (clean air): ");
  Serial.print(R0);
  Serial.println(" ohms");
}

void loop() {
  // Nothing here for now
}

float calibrateR0() {
  const int samples = 100;
  float rsSum = 0;

  for (int i = 0; i < samples; i++) {
    int raw = analogRead(mq131Pin);
    float Vout = raw * (ADCref / 1023.0);  // Voltage read by Teensy
    float Rs = RL * ((Vcc - Vout) / Vout); // Back-calculate Rs
    rsSum += Rs;
    delay(100); // Give time between samples
  }

  return rsSum / samples;
}
