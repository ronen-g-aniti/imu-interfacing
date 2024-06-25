# MPU9250 Sensor Calibration and Data Processing

Welcome to the MPU9250 Sensor Calibration and Data Processing project. This repository contains the code to calibrate and process data from the MPU9250 sensor, ensuring accurate readings for accelerometer and gyroscope data. This project showcases my skills and learning in sensor integration, calibration, and data processing, essential for applications in defense technology.

## Project Overview

This project performs the following tasks:
- **Sensor Initialization:** Configures the MPU9250 sensor for accurate data readings.
- **Sensor Calibration:** Calculates and applies offsets to ensure accurate accelerometer and gyroscope readings when the device is stationary.
- **Data Smoothing:** Applies a moving average filter to smooth out noise in the sensor data.
- **Data Output:** Outputs the raw and filtered data in a format compatible with the Arduino Serial Plotter.

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/ronen-g-aniti/imu-interfacing.git
   cd imu_interfacing
   ```

2. Open `imu_interfacing.ino` in your Arduino IDE.

3. Upload the code to your Arduino board.

4. Place the device in a stable position for calibration.

## Usage

- **Calibrate the Sensor:** The sensor will automatically calibrate during the first 10 seconds after the code is uploaded. Ensure the device remains stationary during this period.
- **Visualize Data:** Open the Arduino Serial Plotter to visualize raw and filtered sensor data.

## Technical Insights

### I2C Communication

The MPU9250 sensor communicates with the Arduino over the I2C bus, which consists of two lines: Serial Data Line (SDA) and Serial Clock Line (SCL). This project demonstrates my ability to work with I2C communication and interface with sensor registers.

### Register Interfacing

Registers in the MPU9250 sensor are used to configure the sensor, read data, and control its operation. Here’s how I interface with the MPU9250 registers:

1. **Initialization:**
   The sensor is initialized by waking it up and configuring its accelerometer and gyroscope ranges.

   ```cpp
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
   ```

2. **Reading Data:**
   Data is read from the sensor by accessing its registers. Each axis of the accelerometer and gyroscope has two associated registers (high and low bytes).

   ```cpp
   int16_t readRegisterPair(uint8_t deviceAddress, uint8_t highAddress) {
     Wire.beginTransmission(deviceAddress);
     Wire.write(highAddress);
     Wire.endTransmission(false);
     Wire.requestFrom(deviceAddress, (uint8_t)2, (uint8_t)true); // Request 2 bytes from the register address
     uint8_t highByte = Wire.read();
     uint8_t lowByte = Wire.read();
     return (int16_t)((highByte << 8) | lowByte);
   }

   float readSensor(uint8_t deviceAddress, uint8_t highAddress, float scale, float conversionFactor) {
     int16_t rawValue = readRegisterPair(deviceAddress, highAddress);
     return (rawValue / scale) * conversionFactor;
   }
   ```

### Example Usage

Here’s how the sensor data is read and processed in the \`loop\` function:

```cpp
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

  delay(1000); // Adjust the delay to be larger for readability
}
```

## About Me

𝐌𝐲 𝐦𝐢𝐬𝐬𝐢𝐨𝐧 𝐢𝐬 𝐭𝐨 𝐛𝐨𝐥𝐬𝐭𝐞𝐫 𝐧𝐚𝐭𝐢𝐨𝐧𝐚𝐥 𝐬𝐞𝐜𝐮𝐫𝐢𝐭𝐲 𝐛𝐲 𝐚𝐝𝐝𝐫𝐞𝐬𝐬𝐢𝐧𝐠 𝐜𝐫𝐢𝐭𝐢𝐜𝐚𝐥 𝐝𝐞𝐟𝐞𝐧𝐬𝐞 𝐜𝐡𝐚𝐥𝐥𝐞𝐧𝐠𝐞𝐬 𝐭𝐡𝐫𝐨𝐮𝐠𝐡 𝐢𝐧𝐧𝐨𝐯𝐚𝐭𝐢𝐯𝐞 𝐞𝐧𝐠𝐢𝐧𝐞𝐞𝐫𝐢𝐧𝐠 𝐚𝐧𝐝 𝐬𝐭𝐫𝐨𝐧𝐠 𝐜𝐨𝐥𝐥𝐚𝐛𝐨𝐫𝐚𝐭𝐢𝐨𝐧.

This project demonstrates my commitment to continuous learning and my ability to tackle complex engineering challenges. By applying advanced mathematical concepts and programming skills, I aim to contribute to technological advancements in the defense sector.

### Key Competencies

- **Systems Integration:** Integrating sensors, microcontrollers, and algorithms.
- **Mathematical Problem-Solving and Programming:** Proficiency in C++, Python, and MATLAB.
- **Effective Communication and Collaboration:** Communicating complex concepts and collaborating within diverse teams.

I am eager to leverage my skills to contribute to innovative projects and support technological advancements. Let’s connect to discuss how my problem-solving abilities and technical expertise can contribute to our shared goals in engineering and technology innovation.
