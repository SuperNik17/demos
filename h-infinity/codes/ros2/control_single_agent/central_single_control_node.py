# listen_control_trajectory_single_agent2.py
# ------------------------------------------
# Centralized control node for a single differential-drive robot in ROS 2.
# This node generates a trajectory, tracks it using a feedback controller,
# and publishes velocity commands to the robot. The reference trajectory
# follows an 8-shape ("lemniscate") path with configurable parameters.
#
#    The controller supports multiple architectures: P, PD, PID, SS, full-state.
#    Only one is activated at a time (see the active matrices A, B, C, D).
#    The control law is designed with off-center kinematics compensation (e).
#
#    Input:
#   - Odometry from 'odom' topic (used as feedback)
#
#    Output:
#   - Velocity commands on 'cmd_vel'
#   - Reference trajectory on 'reference'
#   - Diagnostic data on 'controller'
#
#    This code was developed for real robot experiments and supports integration
#     with AMCL or simulation environments.
#
# Contributors:
#      Armando Nicolella, Ricardo Nunez Saez, Prof. Leopoldo Armesto
#      Developed during research at Universitat Politècnica de València (UPV)




# ------------------------
# IMPORTS AND CONSTANTS
# ------------------------

import math
import numpy as np
import time

# ROS 2 message types
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

# ROS 2 client library
import rclpy
from rclpy.node import Node

# TF2 for transform listening (used for localization/frames)
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Additional geometry and navigation messages
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry

# Action server interface (for optional trajectory requests)
from rclpy.action import ActionServer
from geometry_msgs.msg import Quaternion
from action_tutorials_interfaces.action import Trajectory

# ------------------------
# ⏱️ CONTROL LOOP PERIOD
# ------------------------
max_delta_t = 0.125  # Control loop period in seconds (8 Hz)


class FrameListener(Node):
    def __init__(self):
        # Initialize the ROS 2 node with the name 'tf2_listener_ref'
        super().__init__('tf2_listener_ref')

        # ------------------------------------
        #  TF2 Transform Listener (optional)
        # ------------------------------------
        # Creates a TF2 buffer and listener in case transform frames are needed
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------------
        #  ROS 2 Publishers and Subscribers
        # ------------------------------------

        # Publisher to send velocity commands to the robot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # Subscriber to receive odometry data from the robot
        # This is the main feedback source for localization
        self.subscriber = self.create_subscription(Odometry, 'odom', self.trajectory_callback, 10)

        # Alternative: uncomment this if you prefer AMCL pose over raw odometry
        # self.subscriber = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.trajectory_callback, 10)

        # Publisher for controller data (e.g., errors, feedback)
        self.publish_error = self.create_publisher(Float32MultiArray, 'controller', 1)

        # Publisher for reference trajectory (for debugging or visualization)
        self.publish_reference = self.create_publisher(Float32MultiArray, 'reference', 1)

        # ------------------------------------
        #  Timer Setup
        # ------------------------------------
        # Create a timer that triggers the control loop at fixed intervals
        self.timer = self.create_timer(max_delta_t, self.on_timer)

        # Timer interval in seconds (sampling time)
        self.delta_t = max_delta_t

        # Internal clock for the reference trajectory
        self.t = 0.0

        # Initialization flags
        self.initialized = False               # True when system is initialized
        self.initialized_odom = False          # True when odometry has been initialized

        # Initial robot pose (used to zero out odometry at startup)
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0

        #Odometry from Laser_odom
        #self.laser_odom_x = 0.0
        #self.laser_odom_y = 0.0
        #self.laser_odom_qx = 0.0
        #self.laser_odom_qy = 0.0
        #self.laser_odom_qz = 0.0
        #self.laser_odom_qw = 0.0

        #self.quaternion_laser = [0.0,0.0,0.0,0.0]
        k=1.0
        
        #Initialize Control

        #SS_Controller
        # self.A = np.array([[0.9131, -0.08906, 0.0, 0.0], [-0.02377, 0.9817, -0.07065, 0.0], [0.0, 0.03579, 0.8042, -0.3002], [0.0, 0.0, 0.1367, 0.8294]])
        # self.B = np.array([[-0.1232],[0.1753],[0.2484],[0.6851]])
        # self.C = np.array([[-0.08305, 0.1611, -0.02515, 0.5723]])
        # self.D = np.array([[0.2207]])

        # Central Controller
        # self.A = np.array([[-0.137464992736094,  -2.148543904879864,  -0.046047706533225,      0.122607586856279],[0.071630609794725,   0.689219678131928,   0.002911918036840,   -0.034039904638467],[0.014162818306142,  -1.006040001907836,   0.999244117999721,   -0.397988726101997],[-0.000000000000000,   2.928938942763931,   0.000000000000000,    0.048477580620565]])
        # self.B = np.array([[0.998756448689353],[-0.277287687835267],[-3.242000082474299],[8.284290352438934]])
        # self.C = np.array([[0.200209235122113,  -0.537135976219966,  -0.011511926633306,   0.030651896714070]])
        # self.D = np.array([[0.249689112172338]])

        # Central Controller 10/05/2024
        #self.B = np.array([[1.227],[-0.9132],[-1.474],[ 1.578]])
        #self.D = np.array([[0.3067]])
        #self.A = np.array([[9.213e-05,     -0.7131,    -0.04765,       0.502],[0.08303,      0.5766,    0.004283,     -0.3737],[0.001096,     -0.5104,       0.989,      -0.603],[9.342e-20,       0.558,  -2.094e-19,     -0.3227]])
        #self.C = np.array([[0.2346,   -0.1783,  -0.01191,    0.1255]])

        #PD_Controller 10/05/2024
        #self.A = np.array([[1.0, 0.0],[0.0, 0.777]])
        #self.B = np.array([[0.125],[-0.223]])
        #self.C = np.array([[0.08271,   -0.452]])
        #self.D = np.array([[0.06198]])

        #PD_Controller
        #self.A = np.array([[1.0, 0.0],[0.0, 0.7357]])
        #self.B = np.array([[0.125],[-0.2643]])
        #self.C = np.array([[0.5704,   -1.1530]])
        #self.D = np.array([[0.2113]])

        #P_Controller
        self.A = np.array([[0.0]])
        self.B = np.array([[0.0]])
        self.C = np.array([[0.0]])
        self.D = np.array([[k]])

        ############################################ NEW_CONTROLLERS ####################################################################3

        #P_Filter_Controller 17/05/2024
        #self.A = np.array([[0.88235294117647056]])
        #self.B = np.array([[0.25]])
        #self.C = np.array([[0.47143603410464058]])
        #self.D = np.array([[0.062612598279522577]])
   
        #PID_Controller_17/05/2024:
        #self.A = np.array([[1,0],[0,0.76922422505584354]])
        #self.B = np.array([[0.125],[-0.23077577494415644]])
        #self.C = np.array([[0.0856031754027057, -0.4355004482339031]])
        #self.D = np.array([[0.062306738525232608]])

        #SS_Controller_17/05/2024:
        #self.A = np.array([[0.57070324527924288,-0.22072945750075068,0,0],[-0.34089474954949694,0.66719523977116735,-0.26950937900805205,0],[0,-0.98638786275080126,-0.17366375928154848,-0.99996213378500731],[0,0,0.40084753192398526,0.43049176852055493]])
        #self.B = np.array([[0.64712288780234062],[0.416484751949411],[-0.48168532707675626],[0.3025829505778]])
        #self.C = np.array([[0.17251287106354021,0.58929922600574747,-0.48258486316529625,0.30730036430021707]])
        #self.D = np.array([[0.31340173929861526]])

        #Full_Controller 17/05/2024:
        #self.B = np.array([[1.2266825351266926],[-0.913221221219195],[-1.4735197883704783],[1.5781746841111532]])
        #self.D = np.array([[0.30667063378167314]])
        #self.A = np.array([[9.2129130281781713E-5,-0.71305712755323469,-0.0476509810607668,0.50195949729283718],[0.083030412588251779,0.57660591632176694,0.0042828614166354256,-0.37369087110463683],[0.0010963192367159694,-0.51044386782446638,0.98901473088619551,-0.60296550333220633],[-3.6341362467156941E-18,0.55796901051596692,2.2469540586934628E-19,-0.32271356468884538]])
        #self.C = np.array([[0.23459851558870726,-0.17826428188830867,-0.0119127452651917,0.12548987432320929]])

        ##################################################################################################################################################

        # ----------------------------------------
        #  CONTROLLER STATE INITIALIZATION (x/y)
        # ----------------------------------------

        # --- X Axis ---

        # Feedback control signal for x-axis (u_fb_x)
        # This stores the computed control input from the controller
        self.u_fb_x = np.zeros([self.D.shape[1], 1])

        # Internal controller state vector for x-axis (x_c_x)
        # Its dimension matches the number of controller states (from A matrix)
        self.x_c_x = np.zeros([self.A.shape[0], 1])

        # Error term between reference and current state in x (e1_x)
        # Used as the input to the controller equation
        self.e1_x = np.zeros([self.B.shape[1], 1])

        # --- Y Axis ---

        # Feedback control signal for y-axis (u_fb_y)
        self.u_fb_y = np.zeros([self.D.shape[1], 1])

        # Internal controller state vector for y-axis
        self.x_c_y = np.zeros([self.A.shape[0], 1])

        # Error term for the y-axis
        self.e1_y = np.zeros([self.B.shape[1], 1])


        # self.get_logger().info('A "%s"' % self.A.shape)
        # self.get_logger().info('B "%s"' % self.B.shape)
        # self.get_logger().info('C "%s"' % self.C.shape)
        # self.get_logger().info('D "%s"' % self.D.shape)
        #print(self.A.shape)
        #print(self.B.shape)
        #print(self.C.shape)
        #print(self.D.shape)

        #print(self.u_fb_x.shape)
        #print(self.x_c_x.shape)
        #print(self.e1_x.shape)

        #print(self.A.shape[0])

        # ----------------------------------------
        # 🤖 Robot Pose Initialization
        # ----------------------------------------

        # Initial robot position (in map or odom frame)
        self.x_rob = 0.0
        self.y_rob = 0.0

        # Initial orientation (in radians)
        self.theta_rob = 0.0

        # Optional: could be used to store full quaternion if needed
        # self.theta_rob_q = (0.0, 0.0, 0.0, 0.0)

        # ----------------------------------------
        # 🎯 Action Server Setup
        # ----------------------------------------

        # This action server allows external nodes (e.g. a launcher or GUI) to trigger
        # the initialization and execution of the control loop via the 'fibonacci' action.
        self._action_server = ActionServer(
            self,
            Trajectory,                # Custom action interface (Trajectory)
            'fibonacci',               # Action name
            self.execute_callback      # Callback function on goal
        )

    def execute_callback(self, goal_handle):
        """
        Callback executed when an external goal is sent to the action server.
        This resets the internal controller state and prepares the system to start.
        """

        # Reset all controller states for X axis
        self.u_fb_x = np.zeros([self.D.shape[1], 1])
        self.x_c_x  = np.zeros([self.A.shape[0], 1])
        self.e1_x   = np.zeros([self.B.shape[1], 1])

        # Reset all controller states for Y axis
        self.u_fb_y = np.zeros([self.D.shape[1], 1])
        self.x_c_y  = np.zeros([self.A.shape[0], 1])
        self.e1_y   = np.zeros([self.B.shape[1], 1])

        # Reset internal timer for reference trajectory
        self.t = 0.0

        # You can optionally reinitialize pose here (commented out)
        # self.x0 = self.x_rob
        # self.y0 = self.y_rob
        # self.theta0 = self.theta_rob

        # Log startup values for debugging
        self.get_logger().info(f'Init_x_0 "{self.x0}"')
        self.get_logger().info(f'Init_x_rob "{self.x_rob}"')
        self.get_logger().info(f'Init_y_0 "{self.y0}"')
        self.get_logger().info(f'Init_y_rob "{self.y_rob}"')
        self.get_logger().info(f'Init_Theta_0 "{self.theta0}"')
        self.get_logger().info(f'Init_Theta_rob "{self.theta_rob}"')

        # Set initialization flags to start control
        self.initialized = True
        self.initialized_odom = True

        # --- Feedback and Result Placeholder ---
        # Optionally: send periodic feedback during execution (commented)
        # while self.t < 120.0:
        #     feedback_msg = Trajectory.Feedback()
        #     feedback_msg.t = self.t
        #     goal_handle.publish_feedback(feedback_msg)
        #     time.sleep(1)

        # Notify the client that the goal succeeded
        goal_handle.succeed()

        # Return result object
        result = Trajectory.Result()
        return result


    def euler_from_quaternion(self, q):
        """
        Convert a quaternion to roll, pitch, yaw (Euler angles).
        
        Parameters:
        - q (geometry_msgs.msg.Quaternion): input quaternion

        Returns:
        - tuple: (roll, pitch, yaw) in radians
        """
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        # Compute roll (rotation around X-axis)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Compute pitch (rotation around Y-axis)
        sinp = 2.0 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        # Compute yaw (rotation around Z-axis)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)

    def quaternion2theta(self, q):
        """
        Extract the yaw angle (theta) from a quaternion.

        Parameters:
        - q (geometry_msgs.msg.Quaternion): input quaternion

        Returns:
        - float: yaw angle in radians
        """
        rpy = self.euler_from_quaternion(q)
        return rpy[2]  # yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert roll, pitch, yaw to a quaternion.

        Parameters:
        - roll (float): rotation around X axis
        - pitch (float): rotation around Y axis
        - yaw (float): rotation around Z axis

        Returns:
        - geometry_msgs.msg.Quaternion: output quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q

    def theta2quaternion(self, theta):
        """
        Generate a quaternion from a single yaw angle (theta).
        Assumes roll = pitch = 0.

        Parameters:
        - theta (float): yaw angle in radians

        Returns:
        - geometry_msgs.msg.Quaternion: quaternion representing pure yaw rotation
        """
        return self.quaternion_from_euler(0.0, 0.0, theta)


    def on_timer(self):
        """
        This function is called periodically based on the timer frequency.
        It computes the control action to follow a time-varying reference trajectory
        using a feedback + feedforward approach.
        """

        msg = Twist()  # message to be published to cmd_vel

        # Robot geometric parameters
        b = 0.287 / 2  # Half the wheel separation
        e = 0.2        # Distance from the center to the control point (off-center model)

        if self.initialized:
            # Get current reference trajectory and its derivatives
            x_ref, y_ref, theta_ref, x_ref_vel, y_ref_vel, theta_ref_vel, t = self.reference_trajectory()

            # ======================
            #  ACTIVE CONTROLLER
            # ======================
            # Controller with transformed reference and control point offset

            # Feedforward velocity terms (transformed to control point)
            self.x_ff = x_ref_vel - e * theta_ref_vel * math.sin(theta_ref)
            self.y_ff = y_ref_vel + e * theta_ref_vel * math.cos(theta_ref)

            # Reference position at control point
            xe = x_ref + e * math.cos(theta_ref)
            ye = y_ref + e * math.sin(theta_ref)

            # Robot position at control point
            rex = self.x_rob + e * math.cos(self.theta_rob)
            rey = self.y_rob + e * math.sin(self.theta_rob)

            # Compute tracking error at control point
            self.e1_x = xe - rex
            self.e1_y = ye - rey

            # Controller logic (state-space feedback)
            self.controller()

            # Total velocity command (feedforward + feedback)
            x_vel = self.u_fb_x + self.x_ff
            y_vel = self.u_fb_y + self.y_ff

            # ===========================================
            #  Other control strategies (for testing)
            # ===========================================

            # ---- P Control on feedback point (x_rob) ----
            # x_vel = x_ref_vel + k * (x_ref - (x_rob + e*cos(theta_rob)))

            # ---- PD Control (based on velocity error) ----
            # x_vel = ((x_ref_vel - e*theta_ref_vel*sin(theta_ref)) - (x_vel_1 - e*theta_rob_vel*sin(theta_rob))) + ...

            # ---- Feedback without reference transformation ----
            # self.e1_x = x_ref - (x_rob + e*cos(theta_rob))
            # self.e1_y = y_ref - (y_rob + e*sin(theta_rob))

            # self.u_fb_x = k * self.e1_x
            # self.u_fb_y = k * self.e1_y

            # x_vel = self.u_fb_x + x_ref_vel
            # y_vel = self.u_fb_y + y_ref_vel

            # ===========================================
            #  Differential Drive Mapping
            # ===========================================

            # Compute right and left wheel velocities from (x_vel, y_vel)
            v_i = (1/e) * ((e*math.cos(self.theta_rob) + b*math.sin(self.theta_rob)) * x_vel + 
                           (e*math.sin(self.theta_rob) - b*math.cos(self.theta_rob)) * y_vel)

            v_d = (1/e) * ((e*math.cos(self.theta_rob) - b*math.sin(self.theta_rob)) * x_vel + 
                           (e*math.sin(self.theta_rob) + b*math.cos(self.theta_rob)) * y_vel)

            # Populate velocity command
            msg.linear.x = float((v_i + v_d) / 2.0)  # linear velocity
            msg.angular.z = float((v_d - v_i) / (2.0 * b))  # angular velocity

            # Publish command to robot
            self.publisher.publish(msg)

            # ===========================================
            #  Publish Controller State for Debugging
            # ===========================================

            msg1 = Float32MultiArray()
            msg1.data = [
                float(t), 
                float(self.e1_x), float(self.e1_y),
                float(x_vel), float(y_vel),
                float(self.x_ff), float(self.y_ff),
                float(self.u_fb_x), float(self.u_fb_y),
                float(xe), float(ye), float(rex), float(rey),
                float(self.x_rob), float(self.y_rob),
                float(x_ref), float(y_ref),
                float(self.theta_rob), float(theta_ref)
            ]
            self.publish_error.publish(msg1)

    
    def controller(self):
        """
        Computes the feedback control inputs using a discrete-time state-space controller.
        Updates the controller's internal state using the error signals (e1_x and e1_y).
        """

        # --- X Axis Control ---
        # Control law: u = C*x + D*e
        self.u_fb_x = np.dot(self.C, self.x_c_x) + np.dot(self.D, self.e1_x)
        # State update: x_next = A*x + B*e
        self.x_c_x  = np.dot(self.A, self.x_c_x) + np.dot(self.B, self.e1_x)

        # --- Y Axis Control ---
        self.u_fb_y = np.dot(self.C, self.x_c_y) + np.dot(self.D, self.e1_y)
        self.x_c_y  = np.dot(self.A, self.x_c_y) + np.dot(self.B, self.e1_y)



 ### Tomamos la referencia mediante una suscripción al tópico /reference:

    def trajectory_callback(self, Odometry):
        """
        Callback to receive odometry data and transform it into the robot's control reference frame.
        This provides real-time position and orientation updates of the robot.
        """

        msg = Odometry  # if using PoseWithCovarianceStamped, change accordingly

        if self.initialized_odom:
            # Compute delta pose from initial position
            x = msg.pose.pose.position.x - self.x0
            y = msg.pose.pose.position.y - self.y0

            # Transform to robot's local frame (rotation around initial orientation)
            self.x_rob =  math.cos(self.theta0) * x + math.sin(self.theta0) * y
            self.y_rob = -math.sin(self.theta0) * x + math.cos(self.theta0) * y

            # Orientation update (relative to initial orientation)
            self.theta_rob = self.quaternion2theta(msg.pose.pose.orientation) - self.theta0

            # Debug logs for verification
            self.get_logger().info(f'Odom_x_0 "{self.x0}"')
            self.get_logger().info(f'Odom_y_0 "{self.y0}"')
            self.get_logger().info(f'Odom_Theta_0 "{self.theta0}"')
            self.get_logger().info(f'Odom_x_rob "{self.x_rob}"')
            self.get_logger().info(f'Odom_y_rob "{self.y_rob}"')
            self.get_logger().info(f'Odom_Theta_rob "{self.theta_rob}"')

        else:
            # First-time initialization of pose reference
            self.x0 = msg.pose.pose.position.x
            self.y0 = msg.pose.pose.position.y
            self.theta0 = self.quaternion2theta(msg.pose.pose.orientation)

            # Log initial frame
            self.get_logger().info(f'OdomInit_x_0 "{self.x0}"')
            self.get_logger().info(f'OdomInit_y_0 "{self.y0}"')
            self.get_logger().info(f'OdomInit_Theta_0 "{self.theta0}"')
            self.get_logger().info(f'OdomInit_x_rob "{self.x_rob}"')
            self.get_logger().info(f'OdomInit_y_rob "{self.y_rob}"')
            self.get_logger().info(f'OdomInit_Theta_rob "{self.theta_rob}"')
    
    
    # def pub_position(self):
    #     """
    #     Optional: Publish the robot's estimated pose for visualization or logging.
    #     """
    #     msg = Pose()
    #     msg.position.x = self.rob_position_x
    #     msg.position.y = self.rob_position_y
    #     msg.orientation = self.theta2quaternion(self.rob_orientation_z)
    #     self.publish_position.publish(msg)

    # def pub_error(self):
    #     """
    #     Optional: Publish error and control data for analysis/debugging.
    #     """
    #     msg = Float32MultiArray()
    #     msg.data = [t, self.e1_x, self.e1_y, x_vel, y_vel, self.x_ff, self.y_ff, u_fb_x, u_fb_y]
    #     self.publish_error.publish(msg)




    def reference_trajectory(self):
        """
        Generates a time-varying reference trajectory shaped as an ∞-loop (lemniscate of Gerono).
        The trajectory includes position, velocity, and angular velocity references.

        It also applies a coordinate transformation to rotate the trajectory based on
        a predefined angle (`theta0_comp`) to match the robot's global frame orientation.
        """

        if self.initialized:
            # Increment time
            self.t += self.delta_t
            t = self.t

            # Trajectory amplitude parameters
            Ax = 1.5  # Amplitude along x-axis
            Ay = 0.6  # Amplitude along y-axis

            # Trajectory center
            x0 = 0.0
            y0 = 0.0

            # Angular frequency (completes one loop in 120s)
            w = 2 * math.pi / 120

            # Rotation angle to orient the figure-8 properly
            theta0_comp = -math.atan2(2 * Ay, Ax)

            # ======================================
            # INFINITY LOOP TRAJECTORY (FIGURE-8)
            # ======================================

            # Position equations (lemniscate)
            x = Ax * math.sin(w * t)
            y = Ay * math.sin(2 * w * t)

            # First derivatives (velocities)
            xp = Ax * math.cos(w * t) * w
            yp = Ay * math.cos(2 * w * t) * 2 * w

            # Second derivatives (accelerations)
            xpp = -Ax * math.sin(w * t) * w**2
            ypp = -Ay * math.sin(2 * w * t) * 4 * w**2

            # ======================================
            # ROTATE THE TRAJECTORY
            # ======================================

            c = math.cos(theta0_comp)
            s = math.sin(theta0_comp)

            # Rotated position
            x1 = c * x - s * y + x0
            y1 = s * x + c * y + y0

            # Rotated velocity
            xp1 = c * xp - s * yp
            yp1 = s * xp + c * yp

            # Rotated acceleration
            xpp1 = c * xpp - s * ypp
            ypp1 = s * xpp + c * ypp

            # ======================================
            #  Orientation and Angular Velocity
            # ======================================

            theta1 = math.atan2(yp1, xp1)  # Heading
            thetap1 = (ypp1 * xp1 - yp1 * xpp1) / (yp1**2 + xp1**2 + 1e-9)  # Derivative (avoid division by 0)

            # ======================================
            #  Publish the reference to topic
            # ======================================

            msg = Float32MultiArray()
            msg.data = [
                float(t), 
                float(x1), float(y1), float(theta1), 
                float(xp1), float(yp1), float(thetap1)
            ]

            self.publish_reference.publish(msg)

            # Return all needed components for control
            return (x1, y1, theta1, xp1, yp1, thetap1, t)

        else:
            self.get_logger().info('Trajectory not initialized')


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
