#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <math.h>

// === Capteurs ===
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

// Pyranomètre
byte message[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
float light = 0;

// MQ-7
const int mq7Pin = A8; // Analog pin A8 = Digital pin 22 on Teensy
float RL = 10000.0; // Ohms

// MQ-131
const int mq131Pin = A10; // Analog pin for MQ-131

// === Variables MPU6050 ===
float rollEst = 0, pitchEst = 0;
float rollAcc, pitchAcc;
float alpha = 0.05;

// === Statut des capteurs ===
bool mpuOK = false;
bool bmeOK = false;
bool pyranoOK = true;  // Pas de test simple, on tente la lecture

void setup() {
  Serial2.begin(74880);
  Serial3.begin(4800); // Pour le pyranomètre
  delay(2000);

  Serial2.println("=== Initialisation des capteurs ===");

  // I2C init
  Wire.begin();     // BME280 sur SDA0/SCL0 (18/19)
  Wire1.begin();    // MPU6050 sur SDA1/SCL1 (17/16)

  // MPU6050
  Serial2.println("→ MPU6050...");
  if (mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire1)) {
    Serial2.println("  ✅ MPU6050 détecté.");
    mpuOK = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else {
    Serial2.println("  ❌ MPU6050 non détecté !");
  }

  // BME280
  Serial2.println("→ BME280...");
  if (bme.begin(0x76)) {
    Serial2.println("  ✅ BME280 détecté.");
    bmeOK = true;
  } else {
    Serial2.println("  ❌ BME280 non détecté !");
  }

  Serial2.println("→ Pyranomètre prêt (pas de test à l'init).\n");
}

void readPyranometer() {
  Serial3.write(message, sizeof(message));
  delay(10);
  if (Serial3.available() >= 7) {
    Serial3.read(); Serial3.read(); Serial3.read(); // Ignore header
    light = (256 * Serial3.read()) + Serial3.read();
    light *= 0.207;
    Serial3.read(); Serial3.read(); // Ignore checksum
  } else {
    Serial2.println("  ⚠️  Réponse pyranomètre invalide ou absente.");
    pyranoOK = false;
  }
}

void readMQ7() {
  int rawValue = analogRead(mq7Pin);
  float voltage = rawValue * (3.3 / 1023.0);
  float ppm = map(rawValue, 100, 900, 0, 1000);
  float Vout = voltage;
  float Rs = ((3.3 - Vout) / Vout) * RL;

  Serial2.print("MQ-7 Rs: "); Serial2.println(Rs);
  Serial2.print("MQ-7 Raw: "); Serial2.print(rawValue);
  Serial2.print(" | Voltage: "); Serial2.print(voltage, 2);
  Serial2.print(" V | Approx CO PPM: "); Serial2.println(ppm);
}

void readMQ131() {
  int rawValue = analogRead(mq131Pin);
  float voltage = rawValue * (3.3 / 1023.0);
  float ppm = map(rawValue, 100, 900, 0, 100);

  Serial2.print("MQ-131 Raw: "); Serial2.print(rawValue);
  Serial2.print(" | Voltage: "); Serial2.print(voltage, 2);
  Serial2.print(" V | Approx O3 PPM: "); Serial2.println(ppm);
}

void loop() {
  static unsigned long prevTime = millis();
  unsigned long currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  Serial2.println("\n=========== DONNÉES CAPTEURS ===========");

  // MPU6050
  if (mpuOK) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float gx = g.gyro.x * 180.0 / PI;
    float gy = g.gyro.y * 180.0 / PI;

    pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    rollAcc = atan2(ay, az) * 180.0 / PI;

    pitchEst = alpha * pitchAcc + (1 - alpha) * (pitchEst + gx * deltaTime);
    rollEst = alpha * rollAcc + (1 - alpha) * (rollEst + gy * deltaTime);
    float theta = sqrt(pitchEst * pitchEst + rollEst * rollEst);

    Serial2.print("Inclinaison | Pitch: "); Serial2.print(pitchEst);
    Serial2.print("°, Roll: "); Serial2.print(rollEst);
    Serial2.print("°, Theta: "); Serial2.println(theta);

    Serial2.print("Accélération (g) | X: "); Serial2.print(ax);
    Serial2.print(" | Y: "); Serial2.print(ay);
    Serial2.print(" | Z: "); Serial2.println(az);
  } else {
    Serial2.println("❌ Données MPU6050 non disponibles.");
  }

// Pyranomètre
  readPyranometer();
  if (pyranoOK) {
    Serial2.print("Lumière (pyrano): "); Serial2.print(light); Serial2.println(" W/m²");
  } else {
    Serial2.println("❌ Données pyranomètre non disponibles.");
  }

  // BME280
  if (bmeOK) {
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float pres = bme.readPressure() / 100.0F;

    Serial2.print("Température: "); Serial2.print(temp); Serial2.println(" °C");
    Serial2.print("Humidité: "); Serial2.print(hum); Serial2.println(" %");
    Serial2.print("Pression: "); Serial2.print(pres); Serial2.println(" hPa");
  } else {
    Serial2.println("❌ Données BME280 non disponibles.");
  }

  // MQ-7
  readMQ7();

  // MQ-131
  readMQ131();

  Serial2.println("========================================");

  delay(5000);
}