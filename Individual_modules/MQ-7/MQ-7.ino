// MQ-7 with Teensy - Basic Analog Read Example on Pin A8 (Digital 22)

const int mq7Pin = A8; // Analog pin A8 = Digital pin 22 on Teensy
int rawValue = 0;


void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor
  Serial.println("MQ-7 Sensor Test on Teensy (Pin 22 / A8)");
}

void loop() {
  rawValue = analogRead(mq7Pin); // Read analog value
  float voltage = rawValue * (3.3 / 1023.0); // Convert to voltage (Teensy 3.3V reference)

  // Dummy ppm mapping (for demonstration only)
  float ppm = map(rawValue, 100, 900, 0, 1000); // MQ-7 is sensitive to 20–1000ppm CO


  // Calibraion tests 
  float RL = 10000.0; // Ohms (check your module)
  float adcValue = analogRead(mq7Pin);
  float Vout = adcValue * (3.3 / 1023.0); // Teensy 3.3V ADC
  float Rs = ((3.3 - Vout) / Vout) * RL;

  Serial.print("Rs: ");
  Serial.println(Rs);

  Serial.print("Raw Value: ");
  Serial.print(rawValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V | Approx CO PPM: ");
  Serial.println(ppm);

  delay(1000);
}
