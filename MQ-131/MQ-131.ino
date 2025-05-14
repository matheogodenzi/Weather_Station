// MQ-131 with Teensy - Basic Analog Read Example
// For more accurate readings, you need sensor calibration and temperature/humidity compensation.

const int mq131Pin = A10; // Analog pin where MQ-131 is connected
int rawValue = 0;        // Raw analog reading

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open
  Serial.println("MQ-131 Sensor Test on Teensy");
}

void loop() {
  rawValue = analogRead(mq131Pin); // Read analog value
  float voltage = rawValue * (3.3 / 1023.0); // Convert to voltage (for Teensy 3.3V ADC)
  
  // Optional: Map voltage to estimated O3 ppm (only approximate!)
  // You'll need actual calibration for real ppm conversion
  float ppm = map(rawValue, 100, 900, 0, 100); // Dummy mapping for example

  Serial.print("Raw Value: ");
  Serial.print(rawValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V | Approx PPM: ");
  Serial.println(ppm);

  delay(1000);
}
