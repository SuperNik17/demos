# =============================================================================
# Title: Serial to ROS 2 Pose Publisher with EMA Filtering
# Author: Armando Nicolella
# Description:
#     This ROS 2 Python node reads 3D positional data (x, y, z) from a serial
#     port, applies an Exponential Moving Average (EMA) filter to smooth the
#     data, and publishes the filtered position as a geometry_msgs/Pose message
#     on the 'pose' topic. The node waits until 10 initial samples are collected
#     before starting the EMA. The orientation is fixed and set to no rotation.
#
# Dependencies:
#     - ROS 2 (rclpy)
#     - pyserial (install via `pip install pyserial`)
#
# Usage:
#     Make sure the serial device is accessible at /dev/ttyUSB1 and streaming
#     space-separated float values (x y z) at 115200 baud.
# =============================================================================



import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating ROS 2 nodes
import serial  # Serial communication library (requires: pip install pyserial)
from geometry_msgs.msg import Pose  # ROS 2 message type for representing position and orientation


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')  # Initialize the ROS 2 node with the name 'serial_node'
        self.get_logger().info('Serial Node Initialized')  # Log that the node has been initialized

        # Try to open the serial port
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Open serial port with baudrate 115200 and 1s timeout
        except serial.SerialException as e:
            self.get_logger().fatal(f'Failed to open serial port: {str(e)}')  # Log and exit if the port can't be opened
            rclpy.shutdown()
            exit(1)

        # Create a timer to periodically read data from the serial port (every 0.1s)
        self.timer = self.create_timer(0.1, self.read_serial)

        # Create a publisher to publish Pose messages on the 'pose' topic
        self.publisher = self.create_publisher(Pose, 'pose', 10)

        # Define smoothing factor for EMA (closer to 1 = more weight to new data)
        self.alpha = 0.1

        # Initialize buffers to collect the first 10 values before starting EMA
        self.initial_values_x = []
        self.initial_values_y = []
        self.initial_values_z = []

        # Initialize EMA variables for x, y, and z
        self.ema_x = None
        self.ema_y = None
        self.ema_z = None

    # Legacy/unused EMA function for scalar values (commented out)
    # def calculate_ema(self, value):
    #     '''Calculate the Exponential Moving Average'''
    #     if self.ema is None:
    #         self.initial_values.append(value)
    #         if len(self.initial_values) == 10:
    #             self.ema = sum(self.initial_values) / 10
    #     else:
    #         self.ema = (1 - self.alpha) * self.ema + self.alpha * value
    #     return self.ema

    def calculate_ema(self, value):
        '''Calculate the Exponential Moving Average for a 3D vector'''
        x = value[0]
        y = value[1]
        z = value[2]

        # If EMA hasn't started yet, accumulate initial values
        if self.ema_x is None and self.ema_y is None and self.ema_z is None:
            self.initial_values_x.append(x)
            self.initial_values_y.append(y)
            self.initial_values_z.append(z)

            # Start EMA only after collecting 10 samples
            if len(self.initial_values_x) == 10 and len(self.initial_values_y) == 10 and len(self.initial_values_z) == 10:
                self.ema_x = sum(self.initial_values_x) / 10
                self.ema_y = sum(self.initial_values_y) / 10
                self.ema_z = sum(self.initial_values_z) / 10
        else:
            # Apply EMA update formula
            self.ema_x = (1 - self.alpha) * self.ema_x + self.alpha * x
            self.ema_y = (1 - self.alpha) * self.ema_y + self.alpha * y
            self.ema_z = (1 - self.alpha) * self.ema_z + self.alpha * z

        return self.ema_x , self.ema_y, self.ema_z

    def read_serial(self):
        '''Read data from the serial port and publish it after EMA filtering'''
        self.get_logger().info('Reading serial port')

        # Check if serial data is available
        if self.serial_port and self.serial_port.in_waiting > 0:
            try:
                # Read one line from the serial buffer
                self.get_logger().info('Data available')
                message = self.serial_port.readline().decode('utf-8').rstrip('')
                self.get_logger().info('Received message: "%s"' % message)

                # Parse the message string into a list of floats
                numbers_list_str = message.split()
                numbers_list_float = [float(number) for number in numbers_list_str]

                # Extract x, y, z values
                x = float(numbers_list_float[0])
                y = float(numbers_list_float[1])
                z = float(numbers_list_float[2])
                value = [x, y, z]

                # Apply EMA filtering
                x_new, y_new, z_new = self.calculate_ema(value)
                self.get_logger().info('X: '+ str(x_new) +' Y: '+ str(y_new)+ ' Z: '+ str(z_new))

                # Only publish after the initial EMA is ready
                if self.ema_x is not None and self.ema_y is not None and self.ema_z is not None:
                    x = x_new
                    y = y_new
                    z = z_new
                    self.publish_pose_function(x, y, z)
                else:
                    self.get_logger().info('Initial values not yet reached')

            except ValueError:
                self.get_logger().error('Failed to parse message: "%s"' % message)  # Log parsing errors
            except Exception as e:
                self.get_logger().error(f'Failed to read serial port: {str(e)}')  # Catch-all for other errors
        else:
            self.get_logger().debug('No data available')  # No data in serial buffer
            self.get_logger().debug('Waiting for data...')

    def publish_pose_function(self, x, y, z):
        '''Create and publish a Pose message'''
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        # Fixed orientation (no rotation)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # Publish the Pose message on the 'pose' topic
        self.publisher.publish(msg)


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
