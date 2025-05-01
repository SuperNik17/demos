#!/usr/bin/env python3
"""
Gazebo Follower Control Node (TF2 + PID)
----------------------------------------
This ROS 2 node implements a follower robot controller for simulation in Gazebo. It uses TF2 to track
the leader's position and applies PID control for both angular and linear velocity. The controller is 
activated only once the robot starts moving (based on wheel velocity readings).

Main Features:
- TF2-based relative pose acquisition
- PID control on distance and heading
- Integral term enabled only after wheel movement
- Publishes tracking error for post-processing

Topics:
- Subscribed: `tf`, `/doc_robolab/wheel_vels`
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
from std_msgs.msg import Bool, String
import time
from irobot_create_msgs.msg import WheelVels


class FrameListener(Node):
    def __init__(self):
        super().__init__('turtlebot_tf2_frame_listener')

        # TF2 buffer and listener to access relative transforms between leader and follower
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for distance error tracking (String)
        self.distance_publisher = self.create_publisher(String, '/distanza_turtlebot', 10)

        # Subscribe to wheel velocities from the leader robot
        # Used to enable integral action only after leader starts moving
        self.vel_subscriber = self.create_subscription(
            WheelVels, 'doc_robolab/wheel_vels', self.wheel_vel_callback, 10)
        self.wheelvels = 0  # Becomes 1 when wheel velocities are non-zero

        # Publisher for velocity commands to the follower robot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'tes_robolab/cmd_vel', 10)

        # Timer triggering the control loop at 100 Hz
        self.timer = self.create_timer(0.01, self.on_timer)

        # Obstacle status placeholder (unused in this script)
        self.obstacle_status_msg = Bool()
        self.obstacle_status_msg = False

        # PID parameters for linear velocity control (distance error)
        self.target = 0.8          # Desired distance to maintain
        self.integral_prev = 0     # Previous integral term
        self.dt = 0.01             # Time step for control loop
        self.e_prev = 0            # Previous distance error

        self.Kp = 6
        self.Ki = 1                # Set to 1 for trapezoidal profile tests
        self.kd = 0.0

        # PID parameters for angular velocity control (orientation error)
        self.e_prev2 = 0
        self.integral2_prev = 0
        self.Kp2 = 3
        self.Ki2 = 0.0
        self.kd2 = 0.0

    def wheel_vel_callback(self, msg):
        # If either wheel is moving, enable integral term
        if msg.velocity_right > 0.01 or msg.velocity_left > 0.01:
            self.wheelvels = 1

    def on_timer(self):
        # Main control loop — compute control commands based on TF transform

        try:
            # Lookup relative transform from follower to leader
            t = self.tf_buffer.lookup_transform(
                'tes_robolab/turtlebot4',
                'doc_robolab/turtlebot4',
                rclpy.time.Time())
        except TransformException as ex:
            # TF not available yet — skip this cycle
            return

        # Create a Twist message for velocity commands
        msg = Twist()

        # Compute angular error (orientation) using atan2
        self.error2 = math.atan2(t.transform.translation.y, t.transform.translation.x)

        # PID for angular velocity
        P2 = self.Kp2 * self.error2
        self.integral2 = self.integral2_prev + self.error2 * self.dt
        I2 = self.Ki2 * self.integral2
        D2 = self.kd2 * (self.error2 - self.e_prev2) / self.dt
        MV2 = P2 + I2 + D2

        # Send angular velocity only if the error is significant
        if MV2 > 0.05 or MV2 < -0.05:
            msg.angular.z = MV2
            self.cmd_vel_publisher.publish(msg)

        # Store angular terms for next iteration
        self.e_prev2 = self.error2
        self.integral2_prev = self.integral2

        # Compute distance between follower and leader
        self.distance = math.sqrt(
            t.transform.translation.x ** 2 + t.transform.translation.y ** 2)

        # Compute linear error
        if self.distance != 0:
            self.error = self.distance - self.target
        else:
            self.error = 0

        # Publish distance error to topic for logging or analysis
        msg_distance_error = String()
        msg_distance_error.data = str(self.error)
        self.distance_publisher.publish(msg_distance_error)

        # PID for linear velocity
        P = self.Kp * self.error

        # Enable integral term only after robot has started moving
        if self.wheelvels == 0:
            self.integral = 0
        else:
            self.integral = self.integral_prev + self.error * self.dt

        I = self.Ki * self.integral
        D = self.kd * (self.error - self.e_prev) / self.dt
        print(self.integral)  # Debug print for integral term

        MV = P + I + D

        # Apply linear velocity only if meaningful
        if MV > 0.05:
            msg.linear.x = MV
            self.cmd_vel_publisher.publish(msg)

        # Store linear terms for next iteration
        self.e_prev = self.error
        self.integral_prev = self.integral


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