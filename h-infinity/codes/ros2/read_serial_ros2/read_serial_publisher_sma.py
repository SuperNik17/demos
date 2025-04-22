# =============================================================================
# Title: Serial to ROS 2 Pose Publisher with SMA Filtering
# Author: Armando Nicolella
# Description:
#     This ROS 2 Python node reads 3D positional data (x, y, z) from a serial
#     port, applies a Simple Moving Average (SMA) filter over a configurable
#     window of the last 10 samples, and publishes the smoothed position as a
#     geometry_msgs/Pose message on the 'pose' topic. The orientation is fixed
#     and set to no rotation.
#
# Dependencies:
#     - ROS 2 (rclpy)
#     - pyserial (install via `pip install pyserial`)
#
# Usage:
#     Make sure the serial device is accessible at /dev/ttyUSB1 and streaming
#     space-separated float values (x y z) at 115200 baud.
# =============================================================================



import rclpy
from rclpy.node import Node
import serial # pip install pyserial
from geometry_msgs.msg import Pose 

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')  # Initialize the ROS node with the name 'serial_node'
        self.get_logger().info('Serial Node Initialized')  # Log successful initialization

        # Try to open the serial port
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Open the serial port with 115200 baud rate
        except serial.SerialException as e:
            self.get_logger().fatal(f'Failed to open serial port: {str(e)}')  # Log fatal error if port can't be opened
            rclpy.shutdown()
            exit(1)

        # Create a timer to read the serial port every 0.1 seconds
        self.timer = self.create_timer(0.1, self.read_serial)

        # Create a ROS 2 publisher for the 'pose' topic with geometry_msgs/Pose messages
        self.publisher = self.create_publisher(Pose, 'pose', 10)

        # Initialize SMA (Simple Moving Average) buffer and window size
        self.window_size = 10  # Number of recent samples used for averaging
        self.sma = []  # Buffer to hold recent values

    def read_serial(self):
        '''Read the serial port and return the data'''
        self.get_logger().info('Reading serial port')

        # Check if serial data is available
        if self.serial_port and self.serial_port.in_waiting > 0:
            try:
                # Read and decode the serial message
                self.get_logger().info('Data available')
                message = self.serial_port.readline().decode('utf-8').rstrip('')
                self.get_logger().info('Received message: "%s"' % message)

                # Split and convert string message into float values
                numbers_list_str = message.split()
                numbers_list_float = [float(number) for number in numbers_list_str]
                x = float(numbers_list_float[0])
                y = float(numbers_list_float[1])
                z = float(numbers_list_float[2])

                # Apply SMA filter to the values
                values = self.sma([x, y, z])
                x = values[0]
                y = values[1]
                z = values[2]

                self.get_logger().info('X: '+ str(x) +' Y: '+ str(y)+ ' Z: '+ str(z))

                # Publish the filtered position
                self.publish_pose_function(x, y, z)

            except ValueError:
                self.get_logger().error('Failed to parse message: "%s"' % message)  # Error parsing float values
            except Exception as e:
                self.get_logger().error(f'Failed to read serial port: {str(e)}')  # General read failure
        else:
            self.get_logger().debug('No data available')  # No data in serial buffer
            self.get_logger().debug('Waiting for data...')

    def sma(self, values):
        '''Calculate the Simple Moving Average'''
        self.sma.append(values)  # Add new value to the SMA buffer
        if len(self.sma) > self.window_size:
            self.sma.pop(0)  # Keep the buffer size fixed to the window
        return sum(self.sma) / len(self.sma)  # Compute the average across the buffer

    def publish_pose_function(self, x, y, z):
        '''Publish the pose to the topic'''
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        # Fixed orientation (no rotation)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        self.publisher.publish(msg)  # Publish the Pose message

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)

    serial_node.destroy_node()
    if serial_node.serial_port.is_open:
        serial_node.serial_port.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
