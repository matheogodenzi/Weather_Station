#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

float rollEst = 0, pitchEst = 0;
float rollAcc, pitchAcc;
float alpha = 0.98;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

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
  Serial.begin(9600);
  delay(1000);

  Wire1.begin();  // Teensy 4.0: SDA1 = 17, SCL1 = 16

  Serial.println("MPU6050 Test with Adafruit Library");

  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire1)) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("MPU6050 initialized.");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  calibrateGyro(); // Calibrate before loop starts
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

  float gx_dps = (g.gyro.x - gyroBiasX) * 180.0 / PI;
  float gy_dps = (g.gyro.y - gyroBiasY) * 180.0 / PI;
  float gz_dps = (g.gyro.z - gyroBiasZ) * 180.0 / PI;

  // Accelerometer-based pitch and roll
  pitchAcc = atan2(-az_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
  rollAcc = atan2(ax_g, ay_g) * 180.0 / PI; // True formula is atan2(ay/az) but the axes are defined differently with z going into the battery, x going towards the teensy, and y going towards the battery source

  // Complementary filter
  pitchEst = alpha * pitchAcc + (1 - alpha) * (pitchEst + gx_dps * deltaTime);
  rollEst = alpha * rollAcc + (1 - alpha) * (rollEst + gy_dps * deltaTime);

  float theta = sqrt(pitchEst * pitchEst + rollEst * rollEst);

  // Serial Plotter output
  Serial.print("AccelX:"); Serial.print(ax_g); Serial.print(" ");
  Serial.print("AccelY:"); Serial.print(ay_g); Serial.print(" ");
  Serial.print("AccelZ:"); Serial.print(az_g); Serial.print(" ");

  Serial.print("GyroX:"); Serial.print(gx_dps); Serial.print(" ");
  Serial.print("GyroY:"); Serial.print(gy_dps); Serial.print(" ");
  Serial.print("GyroZ:"); Serial.print(gz_dps); Serial.print(" ");

  Serial.print("Pitch:"); Serial.print(pitchEst); Serial.print(" ");
  Serial.print("Roll:"); Serial.print(rollEst); Serial.print(" ");
  Serial.print("Theta:"); Serial.println(theta);

  delay(500);
}
