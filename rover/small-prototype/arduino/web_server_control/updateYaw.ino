// Update yaw angle using gyroscope data (Z-axis)
// Ignores small angular velocities below a threshold to reduce drift accumulation

void updateYaw() {
  // Apply a simple deadband threshold to ignore minor gyro noise
  if (double(gyroZ - gyroDriftZ) > 1 || double(gyroZ - gyroDriftZ) < -1) {
    yaw += double(gyroZ - gyroDriftZ) * dt;  // Integrate angular velocity over time
    // Alternative approach:
    // yaw = gyroZangle;
  }
}
