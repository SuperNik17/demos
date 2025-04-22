// Read IMU data, apply filtering, and print results to Serial monitor
// This function performs one full cycle of data acquisition, filtering, and debug output

void readNwriteSD() {

  // Check if both accelerometer and gyroscope data are available
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);   // Read acceleration [g]
    IMU.readGyroscope(gyroX, gyroY, gyroZ);   // Read gyroscope data [°/s]
    timeX = micros() - time0;                 // Compute relative timestamp

    dt = (double)(micros() - timer) / 1000000;  // Time step [s]
    timer = micros();                           // Update timer for next iteration

    updatePitchRoll();                          // Compute roll and pitch angles from accelerometer

    double conv = 1;                            // Conversion factor (1 = raw deg/s)
    double gyroXrate = (gyroX - gyroDriftX) / conv;  // Drift-compensated gyroscope X-axis
    double gyroYrate = (gyroY - gyroDriftY) / conv;
    double gyroZrate = (gyroZ - gyroDriftZ) / conv;

#ifdef RESTRICT_PITCH
    // Handle angle wrapping issue between -180° and 180°
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);        // Reinitialize Kalman filter
      kalAngleX = roll;
      gyroXangle = roll;
    } else {
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Update Kalman estimate
    }

    // Invert Y rate if pitch crosses ±90° (to correct rotation direction)
    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate;

    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

#else
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else {
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    }

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate;

    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
#endif

    /* Yaw estimation */
    updateYaw(); // Compute yaw using gyro integration

    // Handle yaw wrapping (discontinuity correction)
    if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
      kalmanZ.setAngle(yaw);
      compAngleZ = yaw;
      kalAngleZ = yaw;
      gyroZangle = yaw;
    } else {
      kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
    }

    // Integrate raw gyroscope values (without filters)
    gyroXangle += gyroXrate * dt;
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;

    // Reset gyro-only angle if it drifts outside of [-180, 180]
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
    if (gyroZangle < -180 || gyroZangle > 180)
      gyroZangle = kalAngleZ;

    // Temperature conversion formula from IMU (if enabled)
    double temperature = (double)tempRaw / 340.0 + 36.53;

    // Debug output on Serial
    Serial.println(millis());
    Serial.println(accX);
    Serial.println(accY);
    Serial.println(accZ);
    Serial.println(gyroXrate);
    Serial.println(gyroYrate);
    Serial.println(gyroZrate);
    Serial.println(kalAngleX);
    Serial.println(kalAngleY);
    Serial.println(kalAngleZ);
  }
}
