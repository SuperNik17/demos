void updatePitchRoll() {
  // Compute roll and pitch angles from accelerometer data
  // atan2 returns values from -π to +π (radians), then converted to degrees
  // See: http://en.wikipedia.org/wiki/Atan2

#ifdef RESTRICT_PITCH // Use equations 25 and 26 (when pitch is limited and yaw is calculated separately)
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Use equations 28 and 29 (general case, pitch and roll both estimated)
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}
