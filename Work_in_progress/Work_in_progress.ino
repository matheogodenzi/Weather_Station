#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <math.h>

// === Sensors ===

// Accelerometer and gyroscope 
Adafruit_MPU6050 mpu;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// Temperature, Pressure, and Humidity
Adafruit_BME280 bme;

// Pyranometer (MODBUS frame on Serial3)
byte message[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
float light = 0;
float correctedLight = 0;

// MQ-7
const int mq7Pin = A8; // Pin 22 on Teensy 4.0
const float mq7_RL = 10000.0; // Ohms to be confirmed
const float mq7_Vcc = 5.0; // MQ-7 powered by 5V
float mq7_R0 = 10000.0; // Ohms, To be calibrated

// MQ-131
const int mq131Pin = A9; // Pin 23 on Teensy 4.0
const float mq131_RL = 10000.0; // Supposed to be correct
const float mq131_Vcc = 5.0; // MQ-131 powered by 5V
float R0 = 13000.0; // To be calibrated

// MPU6050 vars
float rollEst = 0, pitchEst = 0;
float rollAcc, pitchAcc;
float alpha = 0.80; // 80% given to the accelerometer, 20% to the gyro

// Sensor flags
bool mpuOK = false;
bool bmeOK = false;
bool pyranoOK = true;

void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep device still");
  const int samples = 500;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(2);
  }

  gyroBiasX = sumX / samples;
  gyroBiasY = sumY / samples;
  gyroBiasZ = sumZ / samples;

  Serial.println("Gyro bias calibrated.");
}

void setup() {
  Serial.begin(74880);
  Serial3.begin(9600); // Pyranometer MODBUS baud
  delay(2000);

  Serial.println("=== Sensor initialization ===");

  // I2C
  Wire.begin();      // BME280
  Wire1.begin();     // MPU6050

  // MPU6050
  Serial.println("→ MPU6050...");
  if (mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire1)) {
    Serial.println("  ✅ MPU6050 detected.");
    mpuOK = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else {
    Serial.println("  ❌ MPU6050 not detected!");
  }

  calibrateGyro(); // Calibrate before loop starts

  // BME280
  Serial.println("→ BME280...");
  if (bme.begin(0x76)) {
    Serial.println("  ✅ BME280 detected.");
    bmeOK = true;
  } else {
    Serial.println("  ❌ BME280 not detected!");
  }

  Serial.println("→ Pyranometer ready on Serial3 (MODBUS 9600).\n");
}

void readPyranometer() {
  Serial3.write(message, sizeof(message));
  delay(1000);
  if (Serial3.available() >= 7) {
    Serial3.read(); Serial3.read(); Serial3.read(); // Header
    light = (256 * Serial3.read()) + Serial3.read(); // Data
    light *= 0.207; // Convert to W/m²
    Serial3.read(); Serial3.read(); // CRC
  } else {
    Serial.println("  ⚠️  Invalid or missing pyranometer response.");
    pyranoOK = false;
  }
}

void readMQ7() {
  int rawValue = analogRead(mq7Pin);
  float voltage = rawValue * (3.3 / 1023.0);  // Teensy reads in 3.3V, so scale to 3.3V

  // Calculate sensor resistance Rs
  float Rs = mq7_RL * ((mq7_Vcc - voltage) / voltage);  // Powered by 5V

  // Calculate Rs/R0 ratio
  float ratio = Rs / mq7_R0;

  // Estimate CO ppm using log formula
  float a = -1.5; // TO BE UPDATED based on your sensor's datasheet
  float b = 1.0;  // TO BE UPDATED based on your sensor's datasheet
  float mq7_ppm = pow(10, (a * log10(ratio) + b));

  Serial.print("MQ-7 Raw ADC: "); Serial.print(rawValue);
  Serial.print(" | Voltage: "); Serial.print(voltage, 2); Serial.print(" V");
  Serial.print(" | Rs: "); Serial.print(Rs, 2);
  Serial.print(" | Ratio (Rs/R0): "); Serial.print(ratio, 2);
  Serial.print(" | Estimated CO: "); Serial.print(mq7_ppm, 2); Serial.println(" ppm");
}

void readMQ131() {
  int rawValue = analogRead(mq131Pin);
  float voltage = rawValue * (3.3 / 1023.0);  // Teensy reads in 3.3V, so scale to 3.3V
  
  // Sensor resistance
  float Rs = mq131_RL * ((mq131_Vcc - voltage) / voltage);  // Powered by 5V

  // Ratio Rs/R0
  float ratio = Rs / R0;

  // Using an approximate logarithmic formula (from datasheet curve)
  float a = 0.45; // Approximated from datasheet
  float b = 0.9; // Approximated from datasheet
  float mq131_ppm = pow(10, ((1/a) - (b/a) * log10(ratio)));

  Serial.print("MQ-131 Raw ADC: "); Serial.print(rawValue);
  Serial.print(" | Voltage: "); Serial.print(voltage, 2); Serial.print(" V");
  Serial.print(" | Rs: "); Serial.print(Rs, 2);
  Serial.print(" | Ratio (Rs/R0): "); Serial.print(ratio, 2);
  Serial.print(" | Estimated O3: "); Serial.print(mq131_ppm, 2); Serial.println(" ppm");
}

void loop() {
  static unsigned long prevTime = millis();
  unsigned long currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  Serial.println("\n=========== SENSOR DATA ===========");

  // === Step 1: Read pyranometer ===
  readPyranometer();  // updates `light`

  // === Step 2: Get MPU6050 data and compute tilt ===
  if (mpuOK) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float gx = (g.gyro.x - gyroBiasX) * 180.0 / PI;
    float gy = (g.gyro.y - gyroBiasY) * 180.0 / PI;

    // Accelerometer-based pitch and roll
    pitchAcc = atan2(-az, sqrt(ay * ay + az * az)) * 180.0 / PI;
    rollAcc = atan2(ax, ay) * 180.0 / PI; // True formula is atan2(ay/az) but the axes are defined differently with z going into the battery, x going towards the teensy, and y going towards the battery source and supposing that the battery faces the front of the boat and the motherboard the rear.


    pitchEst = alpha * pitchAcc + (1 - alpha) * (pitchEst + gx * deltaTime);
    rollEst = alpha * rollAcc + (1 - alpha) * (rollEst + gy * deltaTime);

    float theta = sqrt(pitchEst * pitchEst + rollEst * rollEst);
    float correctedIrradiance = light / cos(theta * PI / 180.0);

    Serial.print("Inclination | Pitch: "); Serial.print(pitchEst);
    Serial.print("°, Roll: "); Serial.print(rollEst);
    Serial.print("°, Theta: "); Serial.println(theta);

    //Serial.print("Light (raw): "); Serial.print(light); Serial.println(" W/m²");
    Serial.print("Corrected Light: "); Serial.print(correctedIrradiance); Serial.println(" W/m²");
  } else {
    Serial.println("❌ MPU6050 data not available.");
  }

  // === Step 3: Read BME280 ===
  if (bmeOK) {
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float pres = bme.readPressure() / 100.0F;

    Serial.print("Temperature: "); Serial.print(temp); Serial.println(" °C");
    Serial.print("Humidity: "); Serial.print(hum); Serial.println(" %");
    Serial.print("Pressure: "); Serial.print(pres); Serial.println(" hPa");
  } else {
    Serial.println("❌ BME280 data not available.");
  }

  // === Step 4: Read Gas sensors ===
  readMQ131();  // updates O3
  readMQ7(); // updates CO

  Serial.println("========================================");
  delay(5000);
}
