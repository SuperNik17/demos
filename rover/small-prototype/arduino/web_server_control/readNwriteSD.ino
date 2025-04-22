// Read sensor data, process orientation using Kalman filter, and write to SD card
void readNwriteSD(float dimBuff) {
  int i = 0;

  // Fill the buffer with dimBuff sets of 10 values each
  while (9 + 10 * i < 10 * dimBuff) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(accX, accY, accZ);
      IMU.readGyroscope(gyroX, gyroY, gyroZ);
      timeX = micros() - time0;

      dt = (double)(micros() - timer) / 1000000.0;  // Delta time [s]
      timer = micros();

      updatePitchRoll(); // Compute roll and pitch from accelerometer

      double conv = 1;  // Conversion factor (can be tuned if needed)
      double gyroXrate = (gyroX - gyroDriftX) / conv;
      double gyroYrate = (gyroY - gyroDriftY) / conv;
      double gyroZrate = (gyroZ - gyroDriftZ) / conv;

#ifdef RESTRICT_PITCH
      // Prevent angle wrapping issues in Kalman filter
      if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        kalAngleX = roll;
        gyroXangle = roll;
      } else {
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
      }

      if (abs(kalAngleX) > 90) gyroYrate = -gyroYrate;
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

      if (abs(kalAngleY) > 90) gyroXrate = -gyroXrate;
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
#endif

      updateYaw(); // Compute yaw from gyroscope data

      if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
        kalmanZ.setAngle(yaw);
        compAngleZ = yaw;
        kalAngleZ = yaw;
        gyroZangle = yaw;
      } else {
        kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
      }

      // Integrate gyro rates for raw angle estimation
      gyroXangle += gyroXrate * dt;
      gyroYangle += gyroYrate * dt;
      gyroZangle += gyroZrate * dt;

      // Correct gyro drift if angle exceeds [-180, 180]
      if (gyroXangle < -180 || gyroXangle > 180) gyroXangle = kalAngleX;
      if (gyroYangle < -180 || gyroYangle > 180) gyroYangle = kalAngleY;
      if (gyroZangle < -180 || gyroZangle > 180) gyroZangle = kalAngleZ;

      // Temperature estimation (raw to Celsius)
      double temperature = (double)tempRaw / 340.0 + 36.53;

      // Store all data in buffer
      vettore[0 + 10 * i] = millis();      // Timestamp [ms]
      vettore[1 + 10 * i] = accX;          // Acceleration X
      vettore[2 + 10 * i] = accY;          // Acceleration Y
      vettore[3 + 10 * i] = accZ;          // Acceleration Z
      vettore[4 + 10 * i] = gyroXrate;     // Gyro rate X
      vettore[5 + 10 * i] = gyroYrate;     // Gyro rate Y
      vettore[6 + 10 * i] = gyroZrate;     // Gyro rate Z
      vettore[7 + 10 * i] = kalAngleX;     // Kalman angle X
      vettore[8 + 10 * i] = kalAngleY;     // Kalman angle Y
      vettore[9 + 10 * i] = kalAngleZ;     // Kalman angle Z

      i++;
    }
  }

  // Write collected data to SD card
  File dataFile = SD.open("datalog.m", FILE_WRITE);
  if (dataFile) {
    for (int l = 0; l < 10 * dimBuff; l++) {
      dataFile.println(vettore[l]);
    }
    dataFile.close();
  }
}
