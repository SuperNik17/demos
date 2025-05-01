#!/usr/bin/env python3
"""
Follower Robot Obstacle-Aware Navigation Node (Gazebo Simulation)
----------------------------------------------------------------
This ROS 2 node controls the simulated follower TurtleBot 4 in a leader–follower navigation scenario within Gazebo.
It receives the leader’s position via TF2 transformation and follows using PID control, adapting its trajectory in
real-time based on obstacle avoidance behavior initiated by the leader.

Main Features:
- Uses TF2 to retrieve the leader’s relative pose (in Gazebo)
- Performs PID control on linear and angular velocities
- Reacts to obstacle presence by performing 60° detours and then resuming the original path
- Publishes tracking error for performance evaluation

Topics:
- Subscribed: `/obstacle_status`, `/tes_robolab/odom`, TF2 frames
- Published: `/tes_robolab/cmd_vel`, `/distanza_turtlebot`

Authors: Armando Nicolella (LAM4R-University of Naples Federico II), Pasquale Stingo
"""

import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np
from std_msgs.msg import String
import time


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtlebot_tf2_frame_listener')

        # Setup QoS profile for odometry
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,  
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)

        # Subscribe to follower's odometry
        self.odom_subscriber = self.create_subscription(Odometry, 'tes_robolab/odom', self.odom_tes_callback, qos_profile=qos_profile)

        # Subscribe to leader's obstacle status
        self.obstacle_status_subscriber = self.create_subscription(Bool, '/obstacle_status', self.callback, 10)

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for distance error
        self.distance_publisher = self.create_publisher(String, '/distanza_turtlebot', 10)

        # Publisher for follower's velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/tes_robolab/cmd_vel', 10)

        # Timer callback at 100 Hz
        self.timer = self.create_timer(0.01, self.on_timer)

        # Internal state
        self.obstacle_status_msg = Bool()

        # PID parameters for linear velocity
        self.target = 1.8
        self.integral = 0
        self.dt = 0.01 
        self.e_prev = 0
        self.Kp = 6
        self.Ki = 0.8
        self.kd = 0.0

        # PID parameters for angular velocity
        self.e_prev2 = 0
        self.integral2 = 0
        self.Kp2 = 8
        self.Ki2 = 0.8
        self.kd2 = 0.0

        # Obstacle avoidance flags
        self.obstacle_detected = False # Flag for obstacle presence
        self.yaw_ist = 0  # Initial yaw angle
        self.yaw_angle_abs = 0 # Absolute yaw angle
        self.twist_msg = Twist()
        self.c = 1  # Flag to initiate the first (left) rotation when an obstacle is detected
        self.a = False # Indicates whether the robot is currently in the obstacle avoidance phase
        self.d = 0 # Flag to initiate the second (right) rotation after the obstacle has been passed
        self.t = 0 # Flag used to trigger a one-time update of the target distance after avoidance

    def odom_tes_callback(self, msg):
        # Convert quaternion from odometry to yaw angle in degrees [0, 360]
        quaternion = msg.pose.pose.orientation
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.yaw_angle_abs = math.degrees(yaw) + 180

    def callback(self, msg):
        # Update obstacle presence flag
        self.obstacle_detected = msg.data

    def move_turtlebot(self):
        # Move forward at fixed linear velocity
        self.twist_msg.angular.z = 0.0
        self.twist_msg.linear.x = 0.15
        self.yaw_ist = self.yaw_angle_abs
        self.cmd_vel_publisher.publish(self.twist_msg)

    def rotate_turtlebot(self, n):
        # Rotate robot based on current yaw and direction code
        if n == 2:
            # Counterclockwise (left) rotation
            if self.yaw_angle_abs - self.yaw_ist < 60:
                self.twist_msg.angular.z = 0.2
                self.twist_msg.linear.x = 0.1
                self.cmd_vel_publisher.publish(self.twist_msg)
            else:
                self.get_logger().info("Left rotation complete.")
                self.c = 0
                self.f = 1

        if n == 1:
            # Clockwise (right) rotation
            if self.yaw_angle_abs > self.yaw_ist - 50:
                print("Rotating back to original heading")
                self.twist_msg.angular.z = -0.2
                self.twist_msg.linear.x = 0.1
                self.cmd_vel_publisher.publish(self.twist_msg)
            else:
                self.d = 0

    def on_timer(self):
        # Main control loop, called periodically
        try:
            # Try to get TF2 transform between leader and follower
            t = self.tf_buffer.lookup_transform('tes_robolab/turtlebot4', 'doc_robolab/turtlebot4', rclpy.time.Time())

        except TransformException as ex:
            return  # TF not available yet

        if self.obstacle_detected == True and self.c == 1:
            self.get_logger().info("Obstacle detected. Starting rotation.")
            self.rotate_turtlebot(2)
            self.a = True

        elif self.obstacle_detected != False and self.a == True:
            self.move_turtlebot()

        elif self.obstacle_detected == False and self.a == 1 or self.d == 1:
            self.get_logger().info("Obstacle passed. Rotating back.")
            self.d = 1
            self.rotate_turtlebot(1)
            self.a = False
            self.t = 1

        else:
            # Compute relative heading error
            msg = Twist()
            self.error2 = math.atan2(t.transform.translation.y, t.transform.translation.x)

            # PID control (angular velocity)
            P2 = self.Kp2 * self.error2
            self.integral2 += self.error2 * self.dt
            I2 = self.Ki2 * self.integral2
            D2 = self.kd2 * (self.error2 - self.e_prev2) / self.dt
            MV2 = P2 + I2 + D2

            if abs(MV2) > 0.05:
                msg.angular.z = MV2
                self.cmd_vel_publisher.publish(msg)

            self.e_prev2 = self.error2

            # Compute distance between leader and follower
            self.distance = math.sqrt(t.transform.translation.x ** 2 + t.transform.translation.y ** 2)

            if self.t == 1:
                self.target = self.distance
                self.t = 0

            self.error = self.distance - self.target

            # Publish tracking error
            msg_distance = String()
            msg_distance.data = str(self.error)
            self.distance_publisher.publish(msg_distance)

            # PID control (linear velocity)
            P = self.Kp * self.error
            self.integral += self.error * self.dt
            I = self.Ki * self.integral
            D = self.kd * (self.error - self.e_prev) / self.dt
            MV = P + I + D

            if abs(MV) > 0.05:
                msg.linear.x = MV

            self.cmd_vel_publisher.publish(msg)
            self.e_prev = self.error
            self.yaw_ist = self.yaw_angle_abs
            self.get_logger().info('Initial yaw: {}'.format(self.yaw_ist))


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
