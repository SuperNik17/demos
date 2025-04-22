void updateYaw() {

  // Ignore the gyroscope reading if the angular velocity is below a noise threshold
  // This avoids integrating small fluctuations when the robot is stationary
  if (double(gyroZ - gyroDriftZ) > 1 || double(gyroZ - gyroDriftZ) < -1) {

    // Integrate yaw using drift-compensated Z-axis gyroscope rate and time delta
    yaw += double(gyroZ - gyroDriftZ) * dt;

    // Alternative: directly assign yaw from gyro-only angle (commented out)
    // yaw = gyroZangle;
  }
}
