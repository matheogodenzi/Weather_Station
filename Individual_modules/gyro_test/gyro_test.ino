#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

float rollEst = 0, pitchEst = 0;
float rollAcc, pitchAcc;
float alpha = 0.05;

void setup() {
  Serial.begin(9600);
  delay(1000);  // Allow time for serial monitor to open

  Wire1.begin();  // Use Wire1 on Teensy 4.0 (SDA1=17, SCL1=16)

  Serial.println("MPU6050 Test with Adafruit Library");

  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire1)) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("MPU6050 initialized.");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}


void loop() {
  static unsigned long prevTime = millis();
  unsigned long currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax_g = a.acceleration.x;
  float ay_g = a.acceleration.y;
  float az_g = a.acceleration.z;
  float gx_dps = g.gyro.x * 180.0 / PI;  // convert rad/s to deg/s
  float gy_dps = g.gyro.y * 180.0 / PI;
  float gz_dps = g.gyro.z * 180.0 / PI;

  // Compute accelerometer-based pitch and roll
  pitchAcc = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
  rollAcc = atan2(ay_g, az_g) * 180.0 / PI;

  // Gyro-based integration
  float gyroPitchRate = gx_dps;
  float gyroRollRate = gy_dps;

  // Complementary filter
  pitchEst = alpha * pitchAcc + (1 - alpha) * (pitchEst + gyroPitchRate * deltaTime);
  rollEst = alpha * rollAcc + (1 - alpha) * (rollEst + gyroRollRate * deltaTime);

  float theta = sqrt(pitchEst * pitchEst + rollEst * rollEst);

  // Output
  Serial.println("=== MPU6050 Data ===");

  Serial.print("Accel (g): X = ");
  Serial.print(ax_g);
  Serial.print(" | Y = ");
  Serial.print(ay_g);
  Serial.print(" | Z = ");
  Serial.println(az_g);

  Serial.print("Gyro (°/s): X = ");
  Serial.print(gx_dps);
  Serial.print(" | Y = ");
  Serial.print(gy_dps);
  Serial.print(" | Z = ");
  Serial.println(gz_dps);

  Serial.print("Estimated Pitch: ");
  Serial.print(pitchEst);
  Serial.print(" | Estimated Roll: ");
  Serial.println(rollEst);

  Serial.print("Tilt angle (theta): ");
  Serial.println(theta);

  Serial.println("====================");

  delay(5000);
}
