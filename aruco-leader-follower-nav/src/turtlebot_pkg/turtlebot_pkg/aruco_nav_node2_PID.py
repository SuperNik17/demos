"""
Leader–Follower Control Node (PID-based)
----------------------------------------
This ROS 2 node subscribes to the pose of an ArUco marker (assumed to be mounted on the leader robot),
applies a fixed transformation to account for frame differences, and uses a PID controller to command
the follower robot (TurtleBot 4) to track the leader in both linear and angular velocity.

Main Features:
- Pose transformation from camera frame to robot-centric frame
- PID control for distance (linear velocity) and heading (angular velocity)
- Velocity commands are sent to the follower via /cmd_vel topic
- Error messages are published for logging and post-analysis
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import String
from irobot_create_msgs.msg import WheelVels

class ArucoPoseSubscriber(Node):

    def __init__(self):
        super().__init__('aruco_pose_subscriber')

        # Subscribe to ArUco pose (published by aruco_nav_node.py)
        self.aruco_pose_subscription = self.create_subscription(PoseStamped, '/aruco_pose', self.aruco_pose_callback, 3)

        # Publisher to send velocity commands to the follower TurtleBot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'tes_robolab/cmd_vel', 10)

        # Publisher to broadcast distance error for external logging
        self.distance_publisher = self.create_publisher(String, '/distanza_turtlebot', 10)

     



        # PID parameters for linear velocity control (distance error)
        self.target = 0.8 # Target distance to maintain from the leader
        self.integral_prev = 0 # Previous integral value for PID
        self.dt = 0.01 # Time step for PID calculations
        self.e_prev = 0 # Previous error value for PID

        self.Kp = 4  
        self.Ki = 1.5
        self.kd = 0.0


        # PID parameters for angular velocity control (orientation error)
        self.e_prev2 = 0
        self.integral2_prev = 0

        self.Kp2 = 0.0 # gain proportional to heading error(eg 2.5)
        self.Ki2 = 0.0  # small integral to reduce steady-state offset (eg 0.1)
        self.kd2 = 0.0 # derivative to damp oscillations (eg 0.05)


    def aruco_pose_callback(self, msg):

        # Fixed transformation from ArUco pose (camera frame) to robot frame
        transformation_matrix = np.array([  
            [0, 0, 1, -0.06],
            [-1, 0, 0, 0,],
            [0, -1, 0, 0.244],
            [0, 0, 0, 1]
        ])

        # Convert pose to homogeneous coordinates
        tvec = np.array ([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1 ])

        # Apply the transformation to get pose in robot-centric frame
        new_pose = np.dot(transformation_matrix, tvec)
        
        #self.get_logger().info('New_Pose: ' + str(new_pose))

        # Call the method that performs PID control and sends velocity commands
        self.move_turtlebot(new_pose)
        
      

    def move_turtlebot(self, new_pose):

        
        # Create Twist message for velocity commands   
        msg = Twist()
        
       
        # --- ANGULAR VELOCITY CONTROL (heading alignment) ---
        self.error2= math.atan2(new_pose[1],new_pose[0])

        
        # PID angular terms
        P2 = self.Kp2*self.error2
        self.integral2 = self.integral2_prev + self.error2*(self.dt)
        I2 = self.Ki2*self.integral2
        D2 = self.kd2*(self.error2 - self.e_prev2)/(self.dt)

        MV2 = P2 + I2 + D2
        
        ## If the heading error is significant, send rotation command
        #if MV2 > 0.05 or MV2 <-0.05:
        #    msg.angular.z = MV2
        #    self.cmd_vel_publisher.publish(msg)
        
        # Always publish angular.z, even if low (to ensure convergence)
        msg.angular.z = MV2
        self.cmd_vel_publisher.publish(msg)

        # Update angular PID state
        self.e_prev2 = self.error2
        self.integral2_prev = self.integral2
        # ---- END OF ANGULAR VELOCITY CONTROL ----

        # --- LINEAR VELOCITY CONTROL (distance to leader) ---
        # Calculate the distance to the leader
        self.distance = math.sqrt(new_pose[0]** 2 +new_pose[1]** 2)
        # Calculate the distance error
        if self.distance !=0:
            self.error =  self.distance - self.target
        else:
            self.error = 0


        # Publish distance error for logging
        msg_error = String()
        msg_error.data = str(self.error)
        self.distance_publisher.publish(msg_error)

    
        # PID calculations
        P = self.Kp*self.error
        self.integral = self.integral_prev + self.error*self.dt    
        I = self.Ki*self.integral
        D = self.kd*(self.error - self.e_prev)/(self.dt)

        #self.get_logger().info('P: ' + str(P))
        #self.get_logger().info('I: ' + str(self.integral))
            
        MV = P + I + D

        # STOP ZONE: avoid jitter and noise near the target
        if abs(self.error) > 0.05:
            msg.linear.x = MV
        else:
            msg.linear.x = 0.0  # Stop when close enough


        # Always publish linear.x, either movement or stop
        self.cmd_vel_publisher.publish(msg)

        self.e_prev = self.error
        self.integral_prev = self.integral

        # Pubblicazione del messaggio Twist
        #self.get_logger().info(f"cmd_vel: {str(MV)}")
        #self.cmd_vel_publisher.publish(msg)


        
def main(args=None):
    rclpy.init(args=args)
    node2 = ArucoPoseSubscriber()

    
    rclpy.spin(node2)
   

    node2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
