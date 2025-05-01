"""
Trapezoidal Velocity Generator for Leader Robot
-----------------------------------------------
This ROS 2 node generates a periodic trapezoidal velocity profile for the leader TurtleBot.
It publishes linear velocity commands on the /doc_robolab/cmd_vel topic, alternating between
acceleration and deceleration phases in a loop.

Main Features:
- Outputs a trapezoidal speed profile
- Automatically switches between acceleration and deceleration
- No feedback required (open-loop)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time




class MoveTurtlebot(Node):

    def __init__(self):
        super().__init__('move_turtlebot_node')
        

        # Publisher for sending velocity commands to the leader robot
        # /doc_robolab is a namespace for the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/doc_robolab/cmd_vel', 2)

        
        # Timer: sends velocity commands every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.move_turtlebot)
        # Velocity and state initialization
        self.vel_ini = 0 # starting velocity
        self.vel = 0 # current velocity
        # state switch: 2 = acceleration, 1 = deceleration
        self.n = 2
      
       

        self.move_start_time = None 
        self.elapsed_time = 0.0

        # Message and utility variables
        self.twist_msg = Twist()
        self.a = 0
        self.b = 0
        
        

    def move_turtlebot(self):
        
        # Update velocity by adding or subtracting 0.005 depending on phase
        self.vel = self.vel_ini +(-1)**self.n*0.005
        # Phase 1: Acceleration
        if  self.n == 2 and  self.vel <= 0.2:
            self.twist_msg.linear.x = self.vel
            self.cmd_vel_publisher.publish(self.twist_msg)
            print(self.twist_msg.linear.x )

        # Phase 2: Cruise (hold at 0.2 m/s)
        elif self.n == 2 and self.vel > 0.2 and self.vel <= 0.3:

            self.twist_msg.linear.x = 0.2
            self.cmd_vel_publisher.publish(self.twist_msg)
            print(self.twist_msg.linear.x )

        # Switch to Deceleration
        elif self.n == 2 and self.vel >= 0.3:

            self.n =1


        # Phase 3: Deceleration
        elif self.n == 1 and self.vel >= 0.2:

            self.twist_msg.linear.x = 0.2
            self.cmd_vel_publisher.publish(self.twist_msg)
            print(self.twist_msg.linear.x )

        
        elif self.n == 1 and self.vel < 0.2 and self.vel >=0:

            self.twist_msg.linear.x = self.vel
            self.cmd_vel_publisher.publish(self.twist_msg)
            print(self.twist_msg.linear.x )

        # Switch to Acceleration
        elif self.n == 1 and self.vel < 0:

            self.n = 2

        # Update velocity memory
        self.vel_ini = self.vel
       



def main(args=None):
    rclpy.init()
    
    move_turtlebot_node = MoveTurtlebot()
    rclpy.spin(move_turtlebot_node)
    move_turtlebot_node.destroy_node()
    rclpy.shutdown()
