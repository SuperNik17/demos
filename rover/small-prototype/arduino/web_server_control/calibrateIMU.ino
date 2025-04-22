// Calibrate the gyroscope by measuring drift over time
// delayMillis: pause before starting calibration (to let the system stabilize)
// calibrationMillis: duration of calibration sampling window

void calibrateIMU(int delayMillis, int calibrationMillis) {
  int calibrationCount = 0;

  delay(delayMillis);  // Avoid vibrations right after reset

  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;  // Initialize sums for drift calculation
  int startTime = millis();

  while (millis() < startTime + calibrationMillis) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(accX, accY, accZ);
      IMU.readGyroscope(gyroX, gyroY, gyroZ);

      // In an ideal case, gyro values should be ~0 when stationary — any value is drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  // Average out the accumulated drift
  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
}
