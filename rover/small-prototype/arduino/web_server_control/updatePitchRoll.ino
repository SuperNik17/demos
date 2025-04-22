// Compute roll and pitch angles from accelerometer data
// Uses different equations depending on RESTRICT_PITCH definition

void updatePitchRoll() {
  // atan2 returns values in the range [-π, π] (radians), which are then converted to degrees

#ifdef RESTRICT_PITCH  // Use Eq. 25 and 26 (avoids gimbal lock when pitch ~ ±90°)
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Use Eq. 28 and 29 (standard roll/pitch from 3-axis accelerometer)
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}
