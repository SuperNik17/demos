// =============================================================================
// Title: MarvelMind Serial Publisher for ROS 2 Integration
// Authors: Armando Nicolella, Rafa Carbonell
// Description:
//     This Arduino sketch interfaces with a MarvelMind SuperBeacon via serial
//     and publishes 3D position data (X, Y, Z) over USB. The output format is
//     designed to be compatible with ROS 2 nodes that process serial input and
//     publish geometry_msgs/Pose messages.
//
//     Position values are converted from millimeters to meters and sent as a
//     space-separated line every 50 ms:
//
//         Example: 1.234 2.345 0.678
//
//     Optionally, fused IMU data can be printed to the serial monitor.
//
// Usage:
//     - Connect MarvelMind beacon to Arduino via UART
//     - Upload this sketch
//     - Use a ROS 2 script to parse and publish the data
//
// Notes:
//     This script is designed for low-latency streaming and assumes the use
//     of a 115200 baud rate over the default Serial interface.
// =============================================================================


#include "Marvelmind.h"  // Include the Marvelmind library for handling beacon data

#define BAUDRATE 115200  // Set the serial communication baud rate

Stream * _HardSerial; // Declare a pointer to a hardware serial stream (used internally by the Marvelmind library)

MarvelmindHedge hedge;  // Create an instance of the MarvelmindHedge class to interface with the beacon
PositionValue position;  // Structure to hold position data (x, y, z)
FusionIMUValuePro fusionIMUValuePro;  // Structure to hold IMU fusion data from the beacon (if used)

float X, Y, Z; // Variables to store the position in millimeters


void setup() {
  Serial.begin(BAUDRATE);  // Initialize the serial communication with the defined baud rate (115200)
  _HardSerial = &Serial;   // Assign the hardware serial port to the pointer (used by Marvelmind library)
  hedge.begin(_HardSerial);  // Initialize the Marvelmind beacon interface with the serial stream
}

const unsigned long delay_ = 50000; // 0.05 seconds in microseconds (50 ms)

void loop() {
  hedge.read(); // Read and update the latest data from the Marvelmind beacon over serial

  // Check if a new valid position is available
  if(hedge.getPositionFromMarvelmindHedge(&position)) {
      // Convert position from millimeters to meters
      X = position.x / 1000.0;
      Y = position.y / 1000.0;
      Z = position.z / 1000.0;

      // Optional: print IMU fusion data (false = print to Serial)
      hedge.printFusionIMUFromMarvelmindHedge(false);

      // Send position over serial as space-separated values
      Serial.print(X);
      Serial.print(" ");
      Serial.print(Y);
      Serial.print(" ");
      Serial.println(Z);
  }

  delayActive(delay_); // Custom non-blocking delay of 50 ms
}

// Function to wait actively for a specified amount of microseconds
void delayActive(unsigned long delay_) {
  unsigned long start_t = micros(); // Get the current time in microseconds

  // Wait until the specified time has passed
  while (micros() - start_t < delay_) {
    // Busy wait (non-blocking to allow fine control)
  }
}
