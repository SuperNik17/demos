"""
Leader Robot Obstacle Avoidance Node
------------------------------------
This ROS 2 node controls the motion of the leader TurtleBot 4 to autonomously detect and avoid obstacles
using data from a 2D LiDAR (`/scan`) and odometry (`/odom`). The obstacle avoidance behavior is coordinated
with a follower robot via a shared obstacle status topic.

Behavior:
- The robot moves forward until an obstacle is detected in front (within 0.5 meters).
- When an obstacle is detected, the robot performs a 90° counterclockwise turn.
- It then follows the obstacle edge while monitoring lateral distance.
- Once the obstacle is cleared (lateral distance increases), it rotates 90° clockwise to return to its original path.
- The node publishes a Boolean signal to `/obstacle_status` to notify the follower of the current obstacle state.

Topics:
- Subscribed: `/doc_robolab/scan`, `/doc_robolab/odom`
- Published: `/doc_robolab/cmd_vel`, `/obstacle_status`

Author: Armando Nicolella (Federico II, LAM4R Lab), Pasquale Stingo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class MoveTurtlebot(Node):

    def __init__(self):
        super().__init__('move_turtlebot_node')

        # Define a QoS profile for odometry messages (best effort, keep last 10)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,  
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        )

        # Publisher to send velocity commands to the leader robot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'doc_robolab/cmd_vel', 10)

        # Subscriber to LaserScan data
        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'doc_robolab/scan', self.laser_scan_callback, 10)

        # Subscriber to odometry data (used to estimate yaw angle)
        self.odom_subscriber = self.create_subscription(Odometry, 'doc_robolab/odom', self.odom_callback, qos_profile=qos_profile)

        # Threshold distance to consider an obstacle detected (in meters)
        self.threshold_distance = 0.5
        
        # Twist message used to send velocity commands
        self.twist_msg = Twist()

        # Flag indicating if the robot is currently avoiding an obstacle
        self.a = False

        # Publisher to notify the follower about obstacle presence
        self.obstacle_status_publisher = self.create_publisher(Bool, '/obstacle_status', 10)
        self.obstacle_status_msg = Bool()

        # Yaw angle at the moment obstacle is detected
        self.yaw_ist = 0

        # Flag for controlling 90° counterclockwise rotation
        self.c = 0

        # Flag for controlling 90° clockwise rotation
        self.d = 0
        
        # Store current yaw angle in degrees (0–360)
        self.yaw_ist = 0
        self.yaw_angle_abs = 0

    def move_turtlebot(self):
        # Move forward with constant linear speed, no angular velocity
        self.twist_msg.angular.z = 0.0
        self.twist_msg.linear.x = 0.33

        self.yaw_ist = self.yaw_angle_abs  # Save current yaw angle before potential rotation
        self.cmd_vel_publisher.publish(self.twist_msg)
        
    def laser_scan_callback(self, msg):
        # Callback triggered upon receiving LaserScan data

        # Compute angle resolution
        total_angle = 360.0
        angle_increment_measurement = total_angle / len(msg.ranges)

        # Define angle windows for detecting front and side obstacles
        start_angle = 88.0 
        end_angle = 92.0
        start_angle2 = 345
        end_angle2 = 360

        # Compute index ranges
        start_index = int(start_angle / angle_increment_measurement)
        end_index = int(end_angle / angle_increment_measurement)
        start_index2 = int(start_angle2 / angle_increment_measurement)
        end_index2 = int(end_angle2 / angle_increment_measurement)

        # Extract front and side laser ranges
        front_ranges = msg.ranges[start_index:end_index]
        self.lateral_distance2 = msg.ranges[start_index2:end_index2]

        # Minimum lateral distance
        self.lateral_distance = min(self.lateral_distance2)

        # Detect obstacle in front or if already started rotation
        if min(front_ranges) < self.threshold_distance or self.c == 1:
            self.obstacle_status_msg.data = True
            self.obstacle_status_publisher.publish(self.obstacle_status_msg)

            self.get_logger().info("Obstacle detected. Starting left rotation.")
            self.c = 1
            self.rotate_turtlebot(2)
            self.a = True  # Mark that we started obstacle avoidance

        # Keep going forward along obstacle edge
        elif self.a == True and self.lateral_distance < self.threshold_distance:
            self.move_turtlebot()

        # Obstacle passed, perform reverse rotation
        elif self.a == True and self.lateral_distance > self.threshold_distance or self.d == 1:
            self.get_logger().info("Obstacle cleared. Rotating back to original heading.")
            self.d = 1
            self.rotate_turtlebot(1)
            self.a = False
            self.obstacle_status_msg.data = False
            self.obstacle_status_publisher.publish(self.obstacle_status_msg)

        # No obstacle: go straight
        elif self.a == False:
            self.move_turtlebot()

    def rotate_turtlebot(self, n):
        # Perform rotation by comparing yaw angles

        if n == 2:
            # Counterclockwise rotation (+90°)
            if self.yaw_angle_abs - self.yaw_ist <= 90:
                self.twist_msg.angular.z = 0.3
                self.twist_msg.linear.x = 0.0
                self.cmd_vel_publisher.publish(self.twist_msg)
            else:
                self.get_logger().info("Left rotation complete.")
                self.c = 0

        if n == 1:
            # Clockwise rotation (-90°)
            if self.yaw_angle_abs > self.yaw_ist - 90:
                self.twist_msg.angular.z = -0.3
                self.twist_msg.linear.x = 0.1
                self.cmd_vel_publisher.publish(self.twist_msg)
            else:
                self.d = 0

    def odom_callback(self, msg):
        # Extract quaternion from odometry message and convert to Euler angles
        quaternion = msg.pose.pose.orientation
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Quaternion to roll-pitch-yaw conversion
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert yaw from [-180, +180] to [0, 360]
        self.yaw_angle_abs = math.degrees(yaw) + 180
        print(self.yaw_angle_abs)


def main(args=None):
    rclpy.init()
    move_turtlebot_node = MoveTurtlebot()
    rclpy.spin(move_turtlebot_node)
    move_turtlebot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
