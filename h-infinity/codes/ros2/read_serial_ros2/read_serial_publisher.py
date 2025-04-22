import rclpy
from rclpy.node import Node
import serial # pip install pyserial
from geometry_msgs.msg import Pose 

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')  # Initialize the ROS 2 node with the name 'serial_node'
        self.get_logger().info('Serial Node Initialized')  # Log that the node has started successfully

        # Try to open the serial port at /dev/ttyUSB1 with 115200 baud rate and 1s timeout
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Failed to open serial port: {str(e)}')  # Log the error if the port can't be opened
            rclpy.shutdown()
            exit(1)

        # Create a timer that calls read_serial() every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.read_serial)

        # Create a ROS 2 publisher that publishes Pose messages on the 'pose' topic
        self.publisher = self.create_publisher(Pose, 'pose', 10)

    def read_serial(self):
        '''Read the serial port and return the data'''
        self.get_logger().info('Reading serial port')  # Log read attempt

        # Check if the serial port is open and data is available
        if self.serial_port and self.serial_port.in_waiting > 0:
            try:
                # Read a line of data from the serial port, decode it, and strip trailing characters
                self.get_logger().info('Data available')
                message = self.serial_port.readline().decode('utf-8').rstrip('')
                self.get_logger().info('Received message: "%s"' % message)

                # Split the message string into a list of substrings
                numbers_list_str = message.split()

                # Convert each substring to float
                numbers_list_float = [float(number) for number in numbers_list_str]

                # Assign the first three values as x, y, z coordinates
                x = float(numbers_list_float[0])
                y = float(numbers_list_float[1])
                z = float(numbers_list_float[2])

                # Log the received coordinates
                self.get_logger().info('X: '+ str(x) +' Y: '+ str(y)+ ' Z: '+ str(z))

                # Publish the coordinates as a Pose message
                self.publish_pose_function(x, y, z)

            except ValueError:
                self.get_logger().error('Failed to parse message: "%s"' % message)  # Log parsing error
            except Exception as e:
                self.get_logger().error(f'Failed to read serial port: {str(e)}')  # Log any other read error
        else:
            self.get_logger().debug('No data available')  # Log when no data is ready
            self.get_logger().debug('Waiting for data...')

    def publish_pose_function(self, x, y, z):
        '''Publish the pose to the topic'''
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        # Set a fixed orientation (no rotation)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # Publish the Pose message to the 'pose' topic
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
