// Calibrate the IMU gyroscope by averaging drift over a fixed time interval
// Parameters:
//   delayMillis        - delay before starting calibration (e.g., 500 ms) to allow stabilization
//   calibrationMillis  - duration of the calibration phase in milliseconds

void calibrateIMU(int delayMillis, int calibrationMillis) { //500 e 500

  int calibrationCount = 0;

  delay(delayMillis); // Prevent vibrations immediately after reset from affecting calibration

  float sumX, sumY, sumZ;
  int startTime = millis();

  // Collect gyro readings for the specified duration
  while (millis() < startTime + calibrationMillis) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(accX, accY, accZ);
      IMU.readGyroscope(gyroX, gyroY, gyroZ);

      // In theory, the gyroscope should output 0°/s when stationary; deviation is considered drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }

  // Handle case where no data was collected
  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  // Compute average drift for each axis
  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
}
