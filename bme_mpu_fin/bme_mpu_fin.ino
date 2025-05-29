#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Create MPU6050 and BME280 objects
MPU6050 mpu(Wire1);            // I2C address: 0x68
Adafruit_BME280 bme;    // I2C address: 0x76 or 0x77 (depending on your setup)

byte message[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A}; // Trame MODBUS pour interroger le pyrano

float rollEst = 0, pitchEst = 0; // Estimated roll and pitch using complementary filter
float rollAcc, pitchAcc; // Accelerometer-based roll and pitch
float alpha = 0.05; // Complementary filter factor
float light;

void setup() {
  Serial1.begin(4800);
  Serial.begin(9600);
  Serial.println("Test_Pyranomètre_OK 21.11.2024");
  
  delay(2000);  // Allow some time for the serial monitor to initialize

  // Initialize I2C communication
  Wire1.begin();

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    while (1);  // Halt program if sensor not found
  }
  Serial.println("MPU6050 initialized successfully!");

  // Initialize BME280
  Serial.println("Initializing BME280...");
  if (!bme.begin(0x76)) {  // Use 0x76 or 0x77 depending on your wiring
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);  // Halt program if sensor not found
  }
  Serial.println("BME280 initialized successfully!");
}

void myTimerEvent(){
Serial1.write(message, sizeof(message)); //interrogation
    delay(10);
        if(Serial1.available()){ 
           (Serial1.read(), HEX); //lecture des 3 premiers octets de la réponse, quick and dirty, sans intérêts
           (Serial1.read(), HEX);
           (Serial1.read(), HEX);
            light = (256 * (Serial1.read())) + (Serial1.read()); //voilà les deux octets qui m'intéressent
            light = light * 0.207; // conversion µmol/sm2 -> w/m2
              Serial.println(light);
           (Serial1.read(), HEX); //2 octets de checksum, ranapéter
           (Serial1.read(), HEX);
        }
}

void loop() {

  myTimerEvent();
  static unsigned long prevTime = millis();
  unsigned long currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  // Read MPU6050 data (acceleration and gyroscope)
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Convert MPU6050 data to meaningful units
  float ax_g = ax / 16384.0;  // Accelerometer values in g (assuming full-scale range = ±2g)
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float gx_dps = gx / 131.0;  // Gyroscope values in degrees per second (assuming full-scale range = ±250°/s)
  float gy_dps = gy / 131.0;
  float gz_dps = gz / 131.0;

  // Calculate accelerometer pitch and roll
  pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  rollAcc = atan2(ay, az) * 180.0 / PI;
  pitchEst = pitchAcc;
  rollEst = rollAcc;
  // Calculate pitch and roll using complementary filter
  float gyroPitchRate = gx / 131.0; // Convert gyro rate to deg/s (adjust based on sensitivity)
  float gyroRollRate = gy / 131.0;  // Same as above

  pitchEst = alpha * pitchAcc + (1 - alpha) * (pitchEst + gyroPitchRate * deltaTime);
  rollEst = alpha * rollAcc + (1 - alpha) * (rollEst + gyroRollRate * deltaTime);

  // Calculate effective tilt angle (theta)
  float theta = sqrt(pitchEst * pitchEst + rollEst * rollEst);

  // Correct irradiance using tilt angle
  float correctedIrradiance = light * cos(theta * PI / 180.0); // Convert theta to radians

  // Read BME280 data (temperature, humidity, and pressure)
  float temperature = bme.readTemperature();  // Temperature in Celsius
  float humidity = bme.readHumidity();        // Humidity as a percentage
  float pressure = bme.readPressure() / 100.0F;  // Pressure in hPa

  Serial.print("corrected irradiance: ");
  Serial.print(correctedIrradiance);
  // Output data to the serial monitor
  Serial.println("==================================");

  // Print MPU6050 data
  Serial.print("Accelerometer (g): X = ");
  Serial.print(ax_g);
  Serial.print(" | Y = ");
  Serial.print(ay_g);
  Serial.print(" | Z = ");
  Serial.println(az_g);

  Serial.print("Gyroscope (°/s): X = ");
  Serial.print(gx_dps);
  Serial.print(" | Y = ");
  Serial.print(gy_dps);
  Serial.print(" | Z = ");
  Serial.println(gz_dps);

  // Print BME280 data
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.println("==================================");
  
  delay(5000); // Delay for readability (5 second)
}
