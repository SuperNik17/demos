// WRITE TO SERIAL (or optionally to web server in future implementations)
// Outputs sensor and orientation data in a structured format

void SerialWrite(float accX, float accY, float accZ,
                 float gyroXrate, float gyroYrate, float gyroZrate,
                 float kalAngleX, float kalAngleY, float kalAngleZ) {

  Serial.println(millis());       // Timestamp [ms]

  // Accelerometer readings [g]
  Serial.println(accX);
  Serial.println(accY);
  Serial.println(accZ);

  // Gyroscope readings [deg/s]
  Serial.println(gyroXrate);
  Serial.println(gyroYrate);
  Serial.println(gyroZrate);

  // Kalman filtered angles [deg]
  Serial.println(kalAngleX);
  Serial.println(kalAngleY);
  Serial.println(kalAngleZ);
}
