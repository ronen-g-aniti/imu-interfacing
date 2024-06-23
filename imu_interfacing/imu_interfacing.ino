#include <Wire.h>

// MPU9250 I2C address
#define MPU9250_ADDR 0x68

// MPU9250 register addresses
#define PWR_MGMT_1  0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

// Buffer size for moving average
const int bufferSize = 10;

// Buffers to store recent readings
float accelXBuffer[bufferSize] = {0};
float accelYBuffer[bufferSize] = {0};
float accelZBuffer[bufferSize] = {0};
float gyroXBuffer[bufferSize] = {0};
float gyroYBuffer[bufferSize] = {0};
float gyroZBuffer[bufferSize] = {0};

float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

int bufferIndex = 0;
int loopCount = 0;
const int calibrationSamples = 1000;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz

  // Initialize MPU9250
  initializeMPU9250();

  // Calibrate sensors
  calibrateSensors();
}

void loop() {
  // Read accelerometer and gyroscope data
  float accelX = readSensor(MPU9250_ADDR, ACCEL_XOUT_H, 16384.0, 9.81) - accelXOffset;
  float accelY = readSensor(MPU9250_ADDR, ACCEL_YOUT_H, 16384.0, 9.81) - accelYOffset;
  float accelZ = readSensor(MPU9250_ADDR, ACCEL_ZOUT_H, 16384.0, 9.81) - accelZOffset;
  float gyroX = readSensor(MPU9250_ADDR, GYRO_XOUT_H, 131.0, 1.0) - gyroXOffset;
  float gyroY = readSensor(MPU9250_ADDR, GYRO_YOUT_H, 131.0, 1.0) - gyroYOffset;
  float gyroZ = readSensor(MPU9250_ADDR, GYRO_ZOUT_H, 131.0, 1.0) - gyroZOffset;

  // Update buffers with new readings
  updateBuffer(accelXBuffer, accelX);
  updateBuffer(accelYBuffer, accelY);
  updateBuffer(accelZBuffer, accelZ);
  updateBuffer(gyroXBuffer, gyroX);
  updateBuffer(gyroYBuffer, gyroY);
  updateBuffer(gyroZBuffer, gyroZ);

  // Increment buffer index once per cycle
  bufferIndex = (bufferIndex + 1) % bufferSize;
  loopCount++;

  // Calculate moving averages
  float filteredAccelX = calculateAverage(accelXBuffer, loopCount);
  float filteredAccelY = calculateAverage(accelYBuffer, loopCount);
  float filteredAccelZ = calculateAverage(accelZBuffer, loopCount);
  float filteredGyroX = calculateAverage(gyroXBuffer, loopCount);
  float filteredGyroY = calculateAverage(gyroYBuffer, loopCount);
  float filteredGyroZ = calculateAverage(gyroZBuffer, loopCount);

  // Print data in a format compatible with Arduino Serial Plotter
  Serial.print(accelX); Serial.print("\t");
  Serial.print(filteredAccelX); Serial.print("\t");
  Serial.print(accelY); Serial.print("\t");
  Serial.print(filteredAccelY); Serial.print("\t");
  Serial.print(accelZ); Serial.print("\t");
  Serial.print(filteredAccelZ); Serial.print("\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(filteredGyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(filteredGyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");
  Serial.print(filteredGyroZ); Serial.println();

  delay(10); // Adjust the delay to be larger for readability
}

void initializeMPU9250() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00); // Wake up the MPU9250
  Wire.endTransmission(true);

  delay(100); // Add delay to ensure MPU9250 is ready

  // Set accelerometer configuration to ±2g
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00); // Set AFS_SEL=0 (±2g)
  Wire.endTransmission(true);

  // Set gyroscope configuration to ±250°/s
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00); // Set FS_SEL=0 (±250°/s)
  Wire.endTransmission(true);
}

float readSensor(uint8_t deviceAddress, uint8_t highAddress, float scale, float conversionFactor) {
  int16_t rawValue = readRegisterPair(deviceAddress, highAddress);
  return (rawValue / scale) * conversionFactor;
}

int16_t readRegisterPair(uint8_t deviceAddress, uint8_t highAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(highAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, (uint8_t)2, (uint8_t)true); // Request 2 bytes from the register address
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();
  return (int16_t)((highByte << 8) | lowByte);
}

void updateBuffer(float* buffer, float newValue) {
  buffer[bufferIndex] = newValue;
}

float calculateAverage(float* buffer, int count) {
  float sum = 0;
  int actualCount = (count < bufferSize) ? count : bufferSize;
  for (int i = 0; i < actualCount; i++) {
    sum += buffer[i];
  }
  return sum / actualCount;
}

void calibrateSensors() {
  float accelXSum = 0;
  float accelYSum = 0;
  float accelZSum = 0;
  float gyroXSum = 0;
  float gyroYSum = 0;
  float gyroZSum = 0;

  for (int i = 0; i < calibrationSamples; i++) {
    accelXSum += readSensor(MPU9250_ADDR, ACCEL_XOUT_H, 16384.0, 9.81);
    accelYSum += readSensor(MPU9250_ADDR, ACCEL_YOUT_H, 16384.0, 9.81);
    accelZSum += readSensor(MPU9250_ADDR, ACCEL_ZOUT_H, 16384.0, 9.81);
    gyroXSum += readSensor(MPU9250_ADDR, GYRO_XOUT_H, 131.0, 1.0);
    gyroYSum += readSensor(MPU9250_ADDR, GYRO_YOUT_H, 131.0, 1.0);
    gyroZSum += readSensor(MPU9250_ADDR, GYRO_ZOUT_H, 131.0, 1.0);
    delay(10); // Short delay between readings
  }

  accelXOffset = accelXSum / calibrationSamples;
  accelYOffset = accelYOffset / calibrationSamples;
  accelZOffset = (accelZSum / calibrationSamples) - 9.81; // Adjust for gravity
  gyroXOffset = gyroXSum / calibrationSamples;
  gyroYOffset = gyroYOffset / calibrationSamples;
  gyroZOffset = gyroZSum / calibrationSamples;
}
