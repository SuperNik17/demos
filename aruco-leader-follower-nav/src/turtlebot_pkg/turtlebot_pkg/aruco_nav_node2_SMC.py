"""
Leader–Follower Control Node (Sliding Mode Control)
---------------------------------------------------
This ROS 2 node uses a Sliding Mode Control (SMC) approach to regulate the linear velocity
of a follower robot based on the distance to a leader identified via an ArUco marker.

Main Features:
- Subscribes to marker pose from aruco_nav_node.py
- Applies a fixed transformation to correct camera-frame errors
- Computes angular correction using PID
- Applies SMC for smooth and robust linear tracking
- Publishes velocity commands on /cmd_vel and logs distance error
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import String


class ArucoPoseSubscriber(Node):

    def __init__(self):
        super().__init__('aruco_pose_subscriber')

        # Subscribe to the ArUco pose topic
        self.aruco_pose_subscription = self.create_subscription(PoseStamped, '/aruco_pose', self.aruco_pose_callback, 3)

        # Publisher to command the follower robot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'tes_robolab/cmd_vel', 10)

        # Publisher for distance error logging
        self.distance_publisher = self.create_publisher(String, '/distanza_turtlebot', 10)



        # Parameters for SMC linear control
        self.target = 0.8 # Target distance to maintain from the leader
        self.K1 = 0.25   #0.42 # Gain for SMC
        self.K = 0.25 #
        self.C = 0.1 # SMC parameter
        self.dt = 0.01 # Time step for control loop
        

        # PID parameters for angular velocity
        self.e_prev2 = 0
        self.integral2_prev = 0
        
        self.Kp2 = 0.5
        self.Ki2 = 0.0
        self.kd2 = 0.0


    def aruco_pose_callback(self, msg):
        # Transformation from camera to robot frame
        transformation_matrix = np.array([  
            [0, 0, 1, -0.06],
            [-1, 0, 0, 0,],
            [0, -1, 0, 0.244],
            [0, 0, 0, 1]
        ])

        # Homogeneous vector of detected position
        tvec = np.array ([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1 ])

        # Transformed pose (robot-centric)
        new_pose = np.dot(transformation_matrix, tvec)
        
        #self.get_logger().info('New_Pose: ' + str(new_pose))

        # Call motion logic
        self.move_turtlebot(new_pose)
        
      

    def move_turtlebot(self, new_pose):

        
 
        msg = Twist()
        
        # --- ANGULAR CONTROL (PID) ---
        # Calcolo l'errore tra le direzioni di avanzamento dei due turtlebot
        self.error2= math.atan2(new_pose[1],new_pose[0])

        
        # PID rotation calculations
        P2 = self.Kp2*self.error2
        self.integral2 = self.integral2_prev + self.error2*(self.dt)
        I2 = self.Ki2*self.integral2
        D2 = self.kd2*(self.error2 - self.e_prev2)/(self.dt)

        MV2 = P2 + I2 + D2
        

        # Apply angular correction only if significant
        if MV2 > 0.05 or MV2 <-0.05:
            msg.angular.z = MV2
            self.cmd_vel_publisher.publish(msg)

        # update stored data for next iteration
        self.e_prev2 = self.error2
        self.integral2_prev = self.integral2

        ####### END ANGULAR CONTROL #######
        # --- LINEAR CONTROL (SMC) ---


        # Compute the distance to the target
        self.distance = math.sqrt(new_pose[0]** 2 +new_pose[1]** 2)
        # Compute the distance error
        self.error =  self.distance - self.target
        # Log the error
        msg_error = String()
        msg_error.data = str(self.error)
        self.distance_publisher.publish(msg_error)
       



        # SMC control law       # Compute the control input
        usmc =  self.K1 * self.error + self.K* math.tanh(100*self.C*self.error)

      
       

        #print(usmc)
        print(self.error)

        # Stop zone logic and always publish Twist
        if abs(self.error) < 0.05:
            msg.linear.x = 0.0
        else:
            msg.linear.x = max(0.0, usmc)  # Only forward motion

        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node2 = ArucoPoseSubscriber()

    
    rclpy.spin(node2)
   

    node2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()