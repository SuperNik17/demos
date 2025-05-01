"""
Follower Robot Obstacle-Aware Navigation Node
--------------------------------------------
This ROS 2 node controls the follower TurtleBot 4 in a leader–follower setup. It receives the leader’s position
through ArUco-based pose estimation and follows it using PID control, while reacting to obstacle avoidance
maneuvers performed by the leader.

Main Features:
- PID control for tracking the leader’s distance and heading
- Real-time obstacle status reception from the leader via a Boolean topic
- Autonomous rotation (±60°) to adapt trajectory during obstacle avoidance
- Pose transformation from camera to robot-centric frame
- Publishes tracking error for performance analysis

Topics:
- Subscribed: `/aruco_pose`, `/obstacle_status`, `/tes_robolab/odom`
- Published: `/tes_robolab/cmd_vel`, `/distanza_turtlebot`

Author: Armando Nicolella (Federico II, LAM4r Lab), Pasqaule Stingo
"""


import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry


class ArucoPoseSubscriber(Node):

    def __init__(self):
        super().__init__('aruco_pose_subscriber')

        # QoS profile for odometry messages
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,  
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)

        # Subscribe to follower odometry
        self.odom_subscriber = self.create_subscription(Odometry, 'tes_robolab/odom', self.odom_tes_callback, qos_profile=qos_profile)

        # Subscribe to obstacle presence (sent by the leader)
        self.obstacle_status_subscriber = self.create_subscription(Bool, '/obstacle_status', self.callback, 10)

        # Subscribe to ArUco pose (estimated by vision node)
        self.aruco_pose_subscription = self.create_subscription(PoseStamped, '/aruco_pose', self.aruco_pose_callback, 3)

        # Publisher for velocity commands to follower robot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'tes_robolab/cmd_vel', 10)

        # Publisher for distance error (used for post-processing)
        self.distance_publisher = self.create_publisher(String, '/distanza_turtlebot', 10)

        # Periodic timer callback (every 0.3s)
        self.timer = self.create_timer(0.3, self.on_timer)

        # Obstacle state flag
        self.obstacle_status_msg = Bool()

        # PID parameters — linear velocity
        self.target = 1.8
        self.integral = 0
        self.time = 0
        self.time_prev = -0.01
        self.e_prev = 0
        self.Kp = 2
        self.Ki = 0.4
        self.kd = 0.0

        # PID parameters — angular velocity
        self.e_prev2 = 0
        self.integral2 = 0
        self.Kp2 = 1
        self.Ki2 = 0.0
        self.kd2 = 0.0

        # State flags
        self.obstacle_detected = False  # Obstacle detected
        self.yaw_ist = 0                # Current yaw
        self.yaw_angle_abs = 0          # Yaw in [0, 360] degrees

        self.twist_msg = Twist()

        # Rotation control flags
        self.c = 1      # Start of first rotation
        self.a = False  # Obstacle avoidance active
        self.d = 0      # Start of second rotation
        self.t = 0      # Use to adjust target distance after obstacle

    def odom_tes_callback(self, msg):
        # Extract yaw angle from follower odometry quaternion
        quaternion = msg.pose.pose.orientation
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

        # Convert quaternion to Euler angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
      
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
      
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert yaw to [0, 360] degrees
        self.yaw_angle_abs = math.degrees(yaw) + 180

    def callback(self, msg):
        # Obstacle state update
        self.obstacle_detected = msg.data

    def move_turtlebot(self):
        # Move forward at fixed linear speed during obstacle avoidance
        self.twist_msg.angular.z = 0.0
        self.twist_msg.linear.x = 0.15
        self.yaw_ist = self.yaw_angle_abs
        self.cmd_vel_publisher.publish(self.twist_msg)

    def rotate_turtlebot(self, n):
        # Perform obstacle-following rotations based on yaw

        if n == 2:
            # First rotation — counterclockwise (60°)
            if self.yaw_angle_abs - self.yaw_ist < 60:
                self.twist_msg.angular.z = 0.2
                self.twist_msg.linear.x = 0.1
                self.cmd_vel_publisher.publish(self.twist_msg)
            else:
                self.get_logger().info("End of left rotation.")
                self.c = 0
                self.f = 1

        if n == 1:
            # Second rotation — clockwise (60°)
            if self.yaw_angle_abs > self.yaw_ist - 60:
                print("rotating back to original direction")
                self.twist_msg.angular.z = -0.2
                self.twist_msg.linear.x = 0.1
                self.cmd_vel_publisher.publish(self.twist_msg)
            else:
                self.d = 0

    def aruco_pose_callback(self, msg):
        # Transform the ArUco pose to robot-centric coordinates
        transformation_matrix = np.array([  
            [0, 0, 1, -0.06],
            [-1, 0, 0, 0],
            [0, -1, 0, 0.244],
            [0, 0, 0, 1]
        ])
        tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1])
        self.new_pose = np.dot(transformation_matrix, tvec)

    def on_timer(self):
        # Main control logic called periodically

        if self.obstacle_detected == True and self.c == 1:
            self.get_logger().info("Obstacle detected. Start rotation.")
            self.rotate_turtlebot(2)
            self.a = True

        elif self.obstacle_detected != False and self.a == True:
            self.move_turtlebot()

        elif self.obstacle_detected == False and self.a == 1 or self.d == 1:
            self.get_logger().info("Obstacle cleared. Rotate back.")
            self.d = 1
            self.rotate_turtlebot(1)
            self.a = False
            self.t = 1

        else:
            try:
                msg = Twist()

                # Angular PID control
                self.error2 = math.atan2(self.new_pose[1], self.new_pose[0])
                P2 = self.Kp2 * self.error2
                self.integral2 += self.Ki2 * self.error2 * (self.time - self.time_prev)
                D2 = self.kd2 * (self.error2 - self.e_prev2) / (self.time - self.time_prev)
                MV2 = P2 + self.integral2 + D2

                if MV2 > 0.05 or MV2 < -0.05:
                    msg.angular.z = MV2
                    self.cmd_vel_publisher.publish(msg)

                self.e_prev2 = self.error2

                # Distance calculation
                self.distance = math.sqrt(self.new_pose[0]**2 + self.new_pose[1]**2)

                if self.t == 1:
                    self.target = self.distance - 0.3
                    self.t = 0

                self.error = self.distance - self.target if self.distance != 0 else 0
                print(self.error)

                # Publish distance error
                msg_distance = String()
                msg_distance.data = str(self.error)
                self.distance_publisher.publish(msg_distance)

                # Linear PID control
                P = self.Kp * self.error
                self.integral += self.Ki * self.error * (self.time - self.time_prev)
                D = self.kd * (self.error - self.e_prev) / (self.time - self.time_prev)
                MV = P + self.integral + D

                if MV > 0:
                    msg.linear.x = MV

                self.cmd_vel_publisher.publish(msg)

                self.e_prev = self.error

                self.yaw_ist = self.yaw_angle_abs
                self.get_logger().info('Initial yaw: {}'.format(self.yaw_ist))

            except Exception as e:
                self.get_logger().error(f"Error during control loop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node2 = ArucoPoseSubscriber()
    rclpy.spin(node2)
    node2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
