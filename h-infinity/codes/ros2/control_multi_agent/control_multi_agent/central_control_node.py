# =============================================================================
# Title: Centralized Controller Node for Multi-Robot Systems in ROS 2
# Authors: Ricardo Núñez Saez, Armando Nicolella, Prof. Leopoldo Armesto
#
# Description:
#     This ROS 2 node implements a centralized control strategy for two
#     differential-drive robots (e.g., TurtleBot3) using either open-loop
#     or closed-loop feedback. The system supports dynamic trajectory 
#     generation, configurable controllers via JSON, and multiple sources 
#     of localization including:
#         - Raw Odometry
#         - Laser Scan Matcher (LSM)
#         - AMCL
#         - ArUco marker-based localization
#
#     The node manages:
#         - Real-time trajectory generation
#         - State-space feedback control
#         - Reference broadcasting (for visualization/debugging)
#         - Sensor fusion options between odometry and external estimators
#
#     Output commands are sent to the two robots under different namespaces.
#     Designed for collaborative navigation, leader-follower coordination,
#     and relative positioning experiments.
#
# Requirements:
#     - ROS 2 (Foxy/Humble)
#     - `central_pkg/config/<controller_type>.json` file with A, B, C, D, Ts matrices
#
# Launch arguments (via ROS 2 parameters) allow runtime configuration of:
#     - Controller type
#     - Trajectory generation strategy
#     - Sensor usage (AMCL, ArUco, LSM, simulated odom)
#     - Open vs closed loop
#
# =============================================================================

import math
import numpy as np
import os

import rclpy  # ROS 2 client library
from rclpy.node import Node  # Base class for creating ROS 2 nodes

# ROS 2 message types
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

# QoS profile configuration for subscriptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# For parameter description metadata
from rcl_interfaces.msg import ParameterDescriptor 

# To locate shared directories in installed packages
from ament_index_python.packages import get_package_share_directory

# ROS 2 standard service message
from std_srvs.srv import SetBool

import json  # For reading controller parameters from .json configuration

# ========================== Global Variables ===========================

# Namespaces for the two robots
namespace1 = 'tb3_0'  # First robot namespace
namespace2 = 'tb3_1'  # Second robot namespace

# Physical parameters of the robots
b = (0.287 / 2)  # Half the distance between the robot's wheels (meters)
e = 0.2  # Offset distance from robot center to control point (meters)

# Trajectory design parameters (commented defaults for context/documentation)
# vel = 0.1
# Ax = 2.5
# Ay = 0.81
# distance = 0.8
# phi0 = -0.275  # Critical phase angle, depends on Ax, Ay, and vel
#                # Must be recalculated if those values change


class CentralizedController(Node):
    def __init__(self):
        super().__init__('Centralized_Controller')  # Initialize the ROS 2 node

        # Set up QoS profile for all subscriptions (best effort, minimal buffer)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Internal flags for system initialization
        self.initialized = False
        self.initialized_odom_1 = False
        self.initialized_odom_2 = False

        ########################### CONTROL PARAMETERS ###########################

        # Robot velocity (used in trajectory generation when use_phi=True)
        vel_parameter_descriptor = ParameterDescriptor(description='This parameter determines the robot velocity')
        self.declare_parameter('velocity', 0.1, vel_parameter_descriptor)
        self.vel = self.get_parameter('velocity').get_parameter_value().double_value
        self.get_logger().info('Velocity: "%s"' % self.vel)

        # Amplitude of trajectory along x-axis
        Ax_parameter_descriptor = ParameterDescriptor(description='This parameter determines trajectory amplitude on x axis')
        self.declare_parameter('Ax', 2.5, Ax_parameter_descriptor)
        self.Ax = self.get_parameter('Ax').get_parameter_value().double_value
        self.get_logger().info('Ax: "%s"' % self.Ax)

        # Amplitude of trajectory along y-axis
        Ay_parameter_descriptor = ParameterDescriptor(description='This parameter determines trajectory amplitude on y axis')
        self.declare_parameter('Ay', 0.81, Ay_parameter_descriptor)
        self.Ay = self.get_parameter('Ay').get_parameter_value().double_value
        self.get_logger().info('Ay: "%s"' % self.Ay)

        # Distance between the two robots in the swarm
        distance_parameter_descriptor = ParameterDescriptor(description='This parameter determines the distance between the robots')
        self.declare_parameter('distance', -0.8, distance_parameter_descriptor)
        self.distance = self.get_parameter('distance').get_parameter_value().double_value
        self.get_logger().info('Distance between the robots: "%s"' % self.distance)

        # Initial phase offset for the trajectory
        phi0_parameter_descriptor = ParameterDescriptor(description='This parameter determines phi value to go through the trajectory')
        self.declare_parameter('phi0', -0.275, phi0_parameter_descriptor)
        self.phi0 = self.get_parameter('phi0').get_parameter_value().double_value
        self.get_logger().info('phi0: "%s"' % self.phi0)

        # Control signal update period (used for timer and integration)
        max_delta_t_parameter_descriptor = ParameterDescriptor(description='This parameter determines the signal period to set the frecuency control')
        self.declare_parameter('period', 0.125, max_delta_t_parameter_descriptor)
        self.max_delta_t = self.get_parameter('period').get_parameter_value().double_value
        self.get_logger().info('Period of signal control: "%s"' % self.max_delta_t)

        ########################### CONFIGURABLE SENSOR & CONTROL FLAGS ###########################

        # Use odometry from Laser Scan Matcher (instead of standard odometry)
        lsm_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use the odometry topic from LSM instead of the odom topic from diff_drive_controller')
        self.declare_parameter('use_lsm', False, lsm_parameter_descriptor)
        self.use_lsm = self.get_parameter('use_lsm').get_parameter_value().bool_value
        self.get_logger().info('Laser Scan Matcher: "%s"' % self.use_lsm)

        # Use AMCL (Adaptive Monte Carlo Localization)
        amcl_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use AMCL')
        self.declare_parameter('use_amcl', False, amcl_parameter_descriptor)
        self.use_amcl = self.get_parameter('use_amcl').get_parameter_value().bool_value
        self.get_logger().info('AMCL: "%s"' % self.use_amcl)

        # Use ArUco markers for localization
        aruco_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use Aruco')
        self.declare_parameter('use_aruco', False, aruco_parameter_descriptor)
        self.use_aruco = self.get_parameter('use_aruco').get_parameter_value().bool_value
        self.get_logger().info('Aruco: "%s"' % self.use_aruco)

        # Use simulated odometry
        odom_sim_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use the odometry simulation')
        self.declare_parameter('use_odom_sim', False, odom_sim_parameter_descriptor)
        self.use_odom_sim = self.get_parameter('use_odom_sim').get_parameter_value().bool_value
        self.get_logger().info('Simulated Odometry: "%s"' % self.use_odom_sim)

        # Broadcast reference trajectory
        broadcast_ref_parameter_descriptor = ParameterDescriptor(description='This parameter determines to broadcast the reference')
        self.declare_parameter('use_broadcast_ref', False, broadcast_ref_parameter_descriptor)
        self.use_broadcast_ref = self.get_parameter('use_broadcast_ref').get_parameter_value().bool_value
        self.get_logger().info('Reference Trasmission: "%s"' % self.use_broadcast_ref)

        # Enable AMCL + odometry fusion
        fusion_amcl_odom_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use the fusion from AMCL and Odometry')
        self.declare_parameter('use_fusion_amcl_odom', False, fusion_amcl_odom_parameter_descriptor)
        self.use_fusion_amcl_odom = self.get_parameter('use_fusion_amcl_odom').get_parameter_value().bool_value
        self.get_logger().info('Odometry and AMCL fusion: "%s"' % self.use_fusion_amcl_odom)

        # Enable ArUco + odometry fusion
        fusion_aruco_odom_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use the fusion from aruco and Odometry')
        self.declare_parameter('use_fusion_aruco_odom', False, fusion_aruco_odom_parameter_descriptor)
        self.use_fusion_aruco_odom = self.get_parameter('use_fusion_aruco_odom').get_parameter_value().bool_value
        self.get_logger().info('Aruco and Odometry Fusion: "%s"' % self.use_fusion_aruco_odom)

        # Use phi-based trajectory (instead of time-based)
        phi_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use AMCL')
        self.declare_parameter('use_phi', True, phi_parameter_descriptor)
        self.use_phi = self.get_parameter('use_phi').get_parameter_value().bool_value
        self.get_logger().info('Phi trajectory: "%s"' % self.use_phi)

        # Enable closed-loop control (vs open-loop)
        close_loop_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use close or open loop')
        self.declare_parameter('use_close_loop', True, close_loop_parameter_descriptor)
        self.use_close_loop = self.get_parameter('use_close_loop').get_parameter_value().bool_value
        self.get_logger().info('Close Loop: "%s"' % self.use_close_loop)

        # Select controller type (used to load .json file with system matrices)
        controller_type_parameter_descriptor = ParameterDescriptor(description='This parameter determines the controller_type')
        self.declare_parameter('controller_type', 'Kp', controller_type_parameter_descriptor)
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value
        self.get_logger().info('Control_type: "%s"' % self.controller_type)

        # Load controller parameters (A, B, C, D, Ts) from config JSON file
        pkg_dir = get_package_share_directory('central_pkg')
        self.controller_type_file_path = os.path.join(pkg_dir, 'config', self.controller_type + '.json')

        # Load matrices from JSON
        with open(self.controller_type_file_path, 'r') as file:
            params = json.load(file)

        self.A = np.asarray(params["A"])
        self.B = np.asarray(params["B"])
        self.C = np.asarray(params["C"])
        self.D = np.asarray(params["D"])
        self.max_delta_t = params["Ts"]  # Override period if defined in JSON



#################################### Publishers and Subscribers #################################################

        # Publishers for velocity commands (cmd_vel) for both robots
        self.publisher1 = self.create_publisher(Twist, f'{namespace1}/cmd_vel', 1)
        self.publisher2 = self.create_publisher(Twist, f'{namespace2}/cmd_vel', 1)

        # Optionally publish reference data (for monitoring/debugging)
        if self.use_broadcast_ref:
            self.publish_reference_1 = self.create_publisher(Float32MultiArray, f'{namespace1}/reference', 1)
            self.publish_reference_2 = self.create_publisher(Float32MultiArray, f'{namespace2}/reference', 1)
            self.publish_reference_broad_1 = self.create_publisher(Odometry, f'{namespace1}/reference_broad', 1)
            self.publish_reference_broad_2 = self.create_publisher(Odometry, f'{namespace2}/reference_broad', 1)

        # Optionally publish simulated odometry values
        if self.use_odom_sim:
            self.publish_odom_1 = self.create_publisher(Float32MultiArray, f'{namespace1}/odom_1_control', 1)
            self.publish_odom_2 = self.create_publisher(Float32MultiArray, f'{namespace2}/odom_2_control', 1)

        # Always publish internal controller state for both robots
        self.publish_controller_data_1 = self.create_publisher(Float32MultiArray, f'{namespace1}/controller_data', 1)
        self.publish_controller_data_2 = self.create_publisher(Float32MultiArray, f'{namespace2}/controller_data', 1)

        # Subscriptions to odometry topics (LSM or standard odometry)
        if self.use_lsm:
            self.subscriber = self.create_subscription(Odometry, f'{namespace1}/scan_odom', self.odom_callback_1, qos_profile)
            self.subscriber = self.create_subscription(Odometry, f'{namespace2}/scan_odom', self.odom_callback_2, qos_profile)
        else:
            self.subscriber = self.create_subscription(Odometry, f'{namespace1}/odom', self.odom_callback_1, qos_profile)
            self.subscriber = self.create_subscription(Odometry, f'{namespace2}/odom', self.odom_callback_2, qos_profile)

        # Subscription to ArUco-based localization from robot 2
        if self.use_aruco:
            self.aruco_subscriber = self.create_subscription(PoseStamped, f'{namespace2}/servo_pose', self.aruco_callback, qos_profile)

        # Subscription to AMCL pose from robot 1
        if self.use_amcl:
            self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, f'{namespace1}/amcl_pose', self.amcl_callback, qos_profile)

        #################################### Internal Variables Initialization ####################################

        # Control-related parameters
        self.target_distance = self.distance  # Desired spacing between robots
        self.delta_t = self.max_delta_t  # Time step for the control loop
        self.phi1 = 0.0  # Phase for robot 1
        self.phi2 = self.phi0  # Phase for robot 2
        self.initialized = False  # Controller state flag
        self.initialized_low_pass_filter = False  # Flag for filter initialization

        # Relative reference (robot 2 w.r.t robot 1)
        self.ref12x = self.distance  # Constant x offset
        self.ref12y = 0  # Constant y offset

        # Initial positions from odometry (for reset compensation)
        self.x_rob1 = 0.0
        self.y_rob1 = 0.0
        self.theta_rob1 = 0.0

        self.x_rob_0_1 = 0.0
        self.y_rob_0_1 = 0.0
        self.theta_rob_0_1 = 0.0

        self.x_rob_0_2 = 0.0
        self.y_rob_0_2 = 0.0
        self.theta_rob_0_2 = 0.0

        # Latest odometry readings (used in closed-loop)
        self.x_rob_odom1 = 0.0
        self.y_rob_odom1 = 0.0
        self.theta_rob_odom1 = 0.0

        self.x_rob_odom2 = 0.0
        self.y_rob_odom2 = 0.0
        self.theta_rob_odom2 = 0.0

        self.last_odom_x1 = 0.0
        self.last_odom_y1 = 0.0
        self.last_odom_theta1 = 0.0

        self.last_odom_x2 = 0.0
        self.last_odom_y2 = 0.0
        self.last_odom_theta2 = 0.0

        # AMCL estimated pose for robot 1
        self.x_rob_AMCL_1 = 0.0
        self.y_rob_AMCL_1 = 0.0
        self.theta_rob_AMCL_1 = 0.0

        # ArUco estimated pose for robot 2
        self.x_rob2A = 0.0
        self.y_rob2A = 0.0
        self.theta_rob2A = 0.0

        # Timer to execute control loop at fixed frequency
        self.timer = self.create_timer(self.max_delta_t, self.on_timer)

        # Service to trigger initialization externally
        self.srv = self.create_service(SetBool, 'Controller_Initialization', self.handle_service_request)

    def handle_service_request(self, request, response):
        # Handle the external service call to initialize the controller
        self.get_logger().info(f'Service request received: {request.data}')
        response.success = True
        response.message = "Initialization successfully processed"
        self.initialization(response.success)
        return response
    
    def initialization(self, initialization_flag):
        # Internal initialization function, called upon receiving a valid service request
        if initialization_flag:
            self.t = 0.0  # Reset internal timer

            # Initialize state-space controller variables (for X and Y separately)
            self.u_fb_x = np.zeros([self.D.shape[1], 1])
            self.x_c_x = np.zeros([self.A.shape[0], 1])
            self.e1_x = np.zeros([self.B.shape[1], 1])

            self.u_fb_y = np.zeros([self.D.shape[1], 1])
            self.x_c_y = np.zeros([self.A.shape[0], 1])
            self.e1_y = np.zeros([self.B.shape[1], 1])

            # Initial positions for both robots
            self.x_rob1 = 0.0
            self.y_rob1 = 0.0
            self.theta_rob1 = 0.0

            self.x_rob2 = self.ref12x
            self.y_rob2 = self.ref12y
            self.theta_rob2 = 0.0

            self.phi1 = 0.0
            self.phi2 = self.phi0

            # Set flags to true
            self.initialized = True
            self.initialized_odom_1 = True
            self.initialized_odom_2 = True


    def reference_trajectory(self):
        # Generate the reference trajectory for both agents
        if self.initialized:
            self.t += self.delta_t
            t = self.t

            Ax = self.Ax
            Ay = self.Ay

            x0 = 0.0
            y0 = 0.0
            w = 2 * math.pi / 90

            # Rotation compensation for trajectory orientation
            theta0_comp = -math.atan2(2 * Ay, Ax)

            ####################################################### First Agent ############################################################

            if self.use_phi:
                # Phi-based parametric trajectory (constant velocity)
                dx1_phi = Ax * math.cos(self.phi1)
                dy1_phi = Ay * math.cos(2 * self.phi1) * 2
                dphi1_t = self.vel / math.sqrt(dx1_phi ** 2 + dy1_phi ** 2)
                self.phi1 += dphi1_t * self.delta_t
                x1 = Ax * math.sin(self.phi1)
                y1 = Ay * math.sin(2 * self.phi1)
                xp1 = dx1_phi * dphi1_t
                yp1 = dy1_phi * dphi1_t
                dx1_phi2 = -Ax * math.sin(self.phi1)
                dy1_phi2 = -Ay * math.sin(2 * self.phi1) * 4
                xpp1 = dx1_phi2 * (dphi1_t ** 2)
                ypp1 = dy1_phi2 * (dphi1_t ** 2)
            else:
                # Sinusoidal trajectory with time-based control
                x1 = Ax * math.sin(w * t)
                y1 = Ay * math.sin(2 * w * t)
                xp1 = Ax * math.cos(w * t) * w
                yp1 = Ay * math.cos(2 * w * t) * 2 * w
                xpp1 = -Ax * math.sin(w * t) * w ** 2
                ypp1 = -Ay * math.sin(2 * w * t) * 4 * w ** 2

            # Rotate and translate the trajectory
            c = math.cos(theta0_comp)
            s = math.sin(theta0_comp)
            x1ref = c * x1 - s * y1 + x0
            y1ref = s * x1 + c * y1 + y0
            x1refp = c * xp1 - s * yp1
            y1refp = s * xp1 + c * yp1
            x1refpp = c * xpp1 - s * ypp1
            y1refpp = s * xpp1 + c * ypp1

            theta1 = math.atan2(y1refp, x1refp)
            thetap1 = (y1refpp * x1refp - y1refp * x1refpp) / (y1refp ** 2 + x1refp ** 2)

            if self.use_broadcast_ref:
                # Publish reference for monitoring
                msg_reference_1 = Float32MultiArray()
                msg_reference_1.data = [float(self.t), float(x1refp), float(y1ref), float(theta1), float(x1refp), float(y1refp), float(thetap1)]
                self.publish_reference_1.publish(msg_reference_1)

                msg_reference_broad_1 = Odometry()
                msg_reference_broad_1.pose.pose.position.x = x1ref
                msg_reference_broad_1.pose.pose.position.y = y1ref
                msg_reference_broad_1.pose.pose.orientation.z = theta1
                msg_reference_broad_1.twist.twist.linear.x = x1refp
                msg_reference_broad_1.twist.twist.linear.y = y1refp
                msg_reference_broad_1.twist.twist.angular.z = thetap1
                self.publish_reference_broad_1.publish(msg_reference_broad_1)

            # Transform for off-center model
            x1eref = x1ref + e * math.cos(theta1)
            y1eref = y1ref + e * math.sin(theta1)
            x1erefp = x1refp - e * thetap1 * math.sin(theta1)
            y1erefp = y1refp + e * thetap1 * math.cos(theta1)

            ####################################################### Second Agent ############################################################

            if self.use_phi:
                dx2_phi = Ax * math.cos(self.phi2)
                dy2_phi = Ay * math.cos(2 * self.phi2) * 2
                dphi2_t = self.vel / math.sqrt(dx2_phi ** 2 + dy2_phi ** 2)
                self.phi2 += dphi2_t * self.delta_t
                x2 = Ax * math.sin(self.phi2)
                y2 = Ay * math.sin(2 * self.phi2)
                xp2 = dx2_phi * dphi2_t
                yp2 = dy2_phi * dphi2_t
                dx2_phi2 = -Ax * math.sin(self.phi2)
                dy2_phi2 = -Ay * math.sin(2 * self.phi2) * 4
                xpp2 = dx2_phi2 * (dphi2_t ** 2)
                ypp2 = dy2_phi2 * (dphi2_t ** 2)

                # Rotate
                x2ref = c * x2 - s * y2 + x0
                y2ref = s * x2 + c * y2 + y0
                x2refp = c * xp2 - s * yp2
                y2refp = s * xp2 + c * yp2
                x2refpp = c * xpp2 - s * ypp2
                y2refpp = s * xpp2 + c * ypp2
            else:
                # Relative position with respect to the first agent
                x2ref = x1ref + math.cos(theta1) * self.ref12x - math.sin(theta1) * self.ref12y
                y2ref = y1ref + math.sin(theta1) * self.ref12x + math.cos(theta1) * self.ref12y
                x2refp = x1refp - thetap1 * (math.sin(theta1) * self.ref12x + math.cos(theta1) * self.ref12y)
                y2refp = y1refp - thetap1 * (-math.cos(theta1) * self.ref12x + math.sin(theta1) * self.ref12y)

            theta2 = math.atan2(y2refp, x2refp)
            thetap2 = (y2refpp * x2refp - x2refpp * y2refp) / (x2refp ** 2 + y2refp ** 2)

            if self.use_broadcast_ref:
                # Publish reference data for second agent
                msg_reference_2 = Float32MultiArray()
                msg_reference_2.data = [float(self.t), float(x2refp), float(y2ref), float(theta2), float(x2refp), float(y2refp), float(thetap2)]
                self.publish_reference_2.publish(msg_reference_2)

                msg_reference_broad_2 = Odometry()
                msg_reference_broad_2.pose.pose.position.x = x2ref
                msg_reference_broad_2.pose.pose.position.y = y2ref
                msg_reference_broad_2.pose.pose.orientation.z = theta2
                msg_reference_broad_2.twist.twist.linear.x = x2refp
                msg_reference_broad_2.twist.twist.linear.y = y2refp
                msg_reference_broad_2.twist.twist.angular.z = thetap2
                self.publish_reference_broad_2.publish(msg_reference_broad_2)

            # Off-center transformation
            x2eref = x2ref + e * math.cos(theta2)
            y2eref = y2ref + e * math.sin(theta2)
            x2erefp = x2refp - e * thetap2 * math.sin(theta2)
            y2erefp = y2refp + e * thetap2 * math.cos(theta2)

            ################################################## Relative Pose ##################################################
            self.x12ref = x1ref - x2ref
            self.y12ref = y1ref - y2ref
            self.x12eref = x1eref - x2eref
            self.y12eref = y1eref - y2eref

            return x1ref, y1ref, x1eref, y1eref, theta1, x1erefp, y1erefp, thetap1, \
                x2ref, y2ref, x2eref, y2eref, theta2, x2erefp, y2erefp, thetap2, self.t
        else:
            self.get_logger().info('Reference trajectory not initialized yet!')

    
    def euler_from_quaternion(self, q):
        # Convert a quaternion into Euler angles (roll, pitch, yaw)
        x, y, z, w = q.x, q.y, q.z, q.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    def quaternion2theta(self, q):
        # Return only the yaw angle from a quaternion
        rpy = self.euler_from_quaternion(q)
        return rpy[2]
    
    
    def on_timer(self):
        # Called periodically to compute control commands and publish to cmd_vel
        msg_1 = Twist()
        msg_2 = Twist()

        if self.initialized:
            self.get_logger().info('Control initialized!')
            # Retrieve reference trajectories and positions
            (x1, y1, x_ref1, y_ref1, theta_ref1, x_ref_vel1, y_ref_vel1, theta_ref_vel1,
            x2, y2, x_ref2, y_ref2, theta_ref2, x_ref_vel2, y_ref_vel2, theta_ref_vel2, t) = self.reference_trajectory()

            # Compute errors for both robots (off-center model)
            r1ex = self.x_rob1 + e * math.cos(self.theta_rob1)
            r1ey = self.y_rob1 + e * math.sin(self.theta_rob1)
            self.e1_x = x_ref1 - r1ex
            self.e1_y = y_ref1 - r1ey

            r2ex = self.x_rob2 + e * math.cos(self.theta_rob2)
            r2ey = self.y_rob2 + e * math.sin(self.theta_rob2)
            x12 = r1ex - r2ex
            y12 = r1ey - r2ey
            self.e12_x = -(self.x12eref - x12)
            self.e12_y = -(self.y12eref - y12)

            self.x_ff_1 = x_ref_vel1
            self.y_ff_1 = y_ref_vel1
            self.x_ff_2 = x_ref_vel2
            self.y_ff_2 = y_ref_vel2

            self.controller()

            # Apply controller (open or closed loop)
            if self.use_close_loop:
                x_vel1 = self.x_ff_1 + self.u_fb_x_1
                y_vel1 = self.y_ff_1 + self.u_fb_y_1
                x_vel2 = self.x_ff_2 + self.u_fb_x_2
                y_vel2 = self.y_ff_2 + self.u_fb_y_2
            else:
                x_vel1, y_vel1 = self.x_ff_1, self.y_ff_1
                x_vel2, y_vel2 = self.x_ff_2, self.y_ff_2

            # Differential drive velocity computation
            v_i1 = (1 / e) * ((e * math.cos(self.theta_rob1) + b * math.sin(self.theta_rob1)) * x_vel1 + (e * math.sin(self.theta_rob1) - b * math.cos(self.theta_rob1)) * y_vel1)
            v_d1 = (1 / e) * ((e * math.cos(self.theta_rob1) - b * math.sin(self.theta_rob1)) * x_vel1 + (e * math.sin(self.theta_rob1) + b * math.cos(self.theta_rob1)) * y_vel1)
            v_i2 = (1 / e) * ((e * math.cos(self.theta_rob2) + b * math.sin(self.theta_rob2)) * x_vel2 + (e * math.sin(self.theta_rob2) - b * math.cos(self.theta_rob2)) * y_vel2)
            v_d2 = (1 / e) * ((e * math.cos(self.theta_rob2) - b * math.sin(self.theta_rob2)) * x_vel2 + (e * math.sin(self.theta_rob2) + b * math.cos(self.theta_rob2)) * y_vel2)

            msg_1.linear.x = float((v_i1 + v_d1) / 2.0)
            msg_2.linear.x = float((v_i2 + v_d2) / 2.0)
            msg_1.angular.z = float((v_d1 - v_i1) / (2.0 * b))
            msg_2.angular.z = float((v_d2 - v_i2) / (2.0 * b))

            self.publisher1.publish(msg_1)
            self.publisher2.publish(msg_2)

            # Publish control diagnostics (for debugging)
            msg_controller_1 = Float32MultiArray()
            msg_controller_2 = Float32MultiArray()
            msg_controller_1.data = [float(t), float(self.e1_x), float(self.e1_y), float(x_vel1), float(y_vel1),
                                    float(self.x_ff_1), float(self.y_ff_1), float(self.u_fb_x_1), float(self.u_fb_y_1),
                                    float(x_ref1), float(y_ref1), float(r1ex), float(r1ey), float(self.x_rob1), float(self.y_rob1),
                                    float(self.theta_rob1), float(x1), float(y1), float(theta_ref1),
                                    float(self.x_rob_odom1), float(self.y_rob_odom1), float(self.theta_rob_odom1),
                                    float(self.x_rob_AMCL_1), float(self.y_rob_AMCL_1), float(self.theta_rob_AMCL_1)]

            msg_controller_2.data = [float(t), float(self.e12_x), float(self.e12_y), float(x_vel2), float(y_vel2),
                                    float(self.x_ff_2), float(self.y_ff_2), float(self.u_fb_x_2), float(self.u_fb_y_2),
                                    float(x_ref2), float(y_ref2), float(r2ex), float(r2ey), float(self.x_rob2), float(self.y_rob2),
                                    float(self.theta_rob2), float(x2), float(y2), float(theta_ref2),
                                    float(self.x_rob_odom2), float(self.y_rob_odom2), float(self.theta_rob_odom2),
                                    float(self.x_rob2A), float(self.y_rob2A), float(self.theta_rob2A)]

            self.publish_controller_data_1.publish(msg_controller_1)
            self.publish_controller_data_2.publish(msg_controller_2)

    
    
    def controller(self):
        # State-space feedback controller for X and Y directions
        ux = [[self.e1_x], [self.e12_x]]
        u_fb_x = np.dot(self.C, self.x_c_x) + np.dot(self.D, ux)
        self.x_c_x = np.dot(self.A, self.x_c_x) + np.dot(self.B, ux)

        uy = [[self.e1_y], [self.e12_y]]
        u_fb_y = np.dot(self.C, self.x_c_y) + np.dot(self.D, uy)
        self.x_c_y = np.dot(self.A, self.x_c_y) + np.dot(self.B, uy)

        # Extract input control for each robot
        self.u_fb_x_1 = u_fb_x[0][0]
        self.u_fb_x_2 = u_fb_x[1][0]
        self.u_fb_y_1 = u_fb_y[0][0]
        self.u_fb_y_2 = u_fb_y[1][0]


    def odom_callback_1(self, msg):
        # Process odometry data from robot 1
        if self.initialized_odom_1:
            # Compute relative odometry w.r.t. the initial pose
            odom_x = msg.pose.pose.position.x - self.x_rob_0_1
            odom_y = msg.pose.pose.position.y - self.y_rob_0_1
            odom_theta = self.quaternion2theta(msg.pose.pose.orientation) - self.theta_rob_0_1

            # Compute the increment in the local robot frame using rotation matrix
            inc_x_odom = math.cos(self.theta_rob_0_1) * (odom_x - self.last_odom_x1) + math.sin(self.theta_rob_0_1) * (odom_y - self.last_odom_y1)
            inc_y_odom = -math.sin(self.theta_rob_0_1) * (odom_x - self.last_odom_x1) + math.cos(self.theta_rob_0_1) * (odom_y - self.last_odom_y1)
            inc_theta_odom = odom_theta - self.last_odom_theta1

            # Update absolute position estimate using rotation-based dead reckoning
            self.x_rob1 += math.cos(inc_theta_odom) * inc_x_odom - math.sin(inc_theta_odom) * inc_y_odom
            self.y_rob1 += math.sin(inc_theta_odom) * inc_x_odom + math.cos(inc_theta_odom) * inc_y_odom
            self.theta_rob1 += inc_theta_odom

            # Save the odometry-only pose as well
            self.x_rob_odom1 += math.cos(inc_theta_odom) * inc_x_odom - math.sin(inc_theta_odom) * inc_y_odom
            self.y_rob_odom1 += math.sin(inc_theta_odom) * inc_x_odom + math.cos(inc_theta_odom) * inc_y_odom
            self.theta_rob_odom1 += inc_theta_odom

            # Store current measurements for next step
            self.last_odom_x1 = odom_x
            self.last_odom_y1 = odom_y
            self.last_odom_theta1 = odom_theta
        else:
            # First-time initialization of odometry reference
            self.get_logger().info('Waiting to initialize Odometry_1 from action')
            self.x_rob_0_1 = msg.pose.pose.position.x
            self.y_rob_0_1 = msg.pose.pose.position.y
            self.theta_rob_0_1 = self.quaternion2theta(msg.pose.pose.orientation)


    def amcl_callback(self, msg):
        # Update AMCL pose for robot 1
        self.x_rob_AMCL_1 = msg.pose.pose.position.x
        self.y_rob_AMCL_1 = msg.pose.pose.position.y
        self.theta_rob_AMCL_1 = self.quaternion2theta(msg.pose.pose.orientation)

        # If fusion is enabled, overwrite odometry-based position with AMCL
        if self.use_fusion_amcl_odom:
            self.x_rob1 = self.x_rob_AMCL_1
            self.y_rob1 = self.y_rob_AMCL_1
            self.theta_rob1 = self.theta_rob_AMCL_1
        

    def odom_callback_2(self, msg):
        # Process odometry data from robot 2
        if self.initialized_odom_2:
            odom_x = msg.pose.pose.position.x - self.x_rob_0_2
            odom_y = msg.pose.pose.position.y - self.y_rob_0_2
            odom_theta = self.quaternion2theta(msg.pose.pose.orientation) - self.theta_rob_0_2

            # Compute local frame increments
            inc_x_odom = math.cos(self.theta_rob_0_2) * (odom_x - self.last_odom_x2) + math.sin(self.theta_rob_0_2) * (odom_y - self.last_odom_y2)
            inc_y_odom = -math.sin(self.theta_rob_0_2) * (odom_x - self.last_odom_x2) + math.cos(self.theta_rob_0_2) * (odom_y - self.last_odom_y2)
            inc_theta_odom = odom_theta - self.last_odom_theta2

            # Update pose estimate
            self.x_rob2 += math.cos(inc_theta_odom) * inc_x_odom - math.sin(inc_theta_odom) * inc_y_odom
            self.y_rob2 += math.sin(inc_theta_odom) * inc_x_odom + math.cos(inc_theta_odom) * inc_y_odom
            self.theta_rob2 += inc_theta_odom

            self.x_rob_odom2 += math.cos(inc_theta_odom) * inc_x_odom - math.sin(inc_theta_odom) * inc_y_odom
            self.y_rob_odom2 += math.sin(inc_theta_odom) * inc_x_odom + math.cos(inc_theta_odom) * inc_y_odom
            self.theta_rob_odom2 += inc_theta_odom

            # Save data for next update
            self.last_odom_x2 = odom_x
            self.last_odom_y2 = odom_y
            self.last_odom_theta2 = odom_theta
        else:
            self.get_logger().info('Waiting to initialize Odometry_2 from action')
            self.x_rob_0_2 = msg.pose.pose.position.x
            self.y_rob_0_2 = msg.pose.pose.position.y
            self.theta_rob_0_2 = self.quaternion2theta(msg.pose.pose.orientation)
        

    def aruco_callback(self, msg2):
        # Transform ArUco pose into the robot 2 frame (custom fixed rotation matrix)
        transformation_matrix = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Original position from ArUco (in camera frame)
        tvec = np.array([msg2.pose.position.x, msg2.pose.position.y, msg2.pose.position.z, 1.0])
        new_pose = np.dot(transformation_matrix, tvec)  # Convert to base_link frame of robot 2

        # Convert orientation to yaw
        rpy = self.euler_from_quaternion(msg2.pose.orientation)
        if rpy[0] < 0:
            thetaA = -(math.pi + rpy[0])
        else:
            thetaA = -(rpy[0] - math.pi)

        xA = new_pose[0]  # Transformed X position
        yA = new_pose[1]  # Transformed Y position

        # Compute robot 2 pose using the known transform from robot 1 to ArUco
        # Robot 2 = Robot 1 - (Camera + ArUco offset)
        theta = self.theta_rob1 - thetaA
        lc = 0.21  # Distance from robot 2 base_link to camera
        la = 0.08  # Distance from robot 1 base_link to ArUco marker

        self.x_rob2A = self.x_rob1 - (math.cos(theta) * (lc + xA) - math.sin(theta) * yA) - math.cos(self.theta_rob1) * la
        self.y_rob2A = self.y_rob1 - (math.sin(theta) * (lc + xA) + math.cos(theta) * yA) - math.sin(self.theta_rob1) * la
        self.theta_rob2A = theta

        # If fusion is enabled, update robot 2 pose directly
        if self.use_fusion_aruco_odom:
            self.x_rob2 = self.x_rob2A
            self.y_rob2 = self.y_rob2A
            self.theta_rob2 = self.theta_rob2A



def main():
    rclpy.init()
    central_control_node = CentralizedController()

    try:
        rclpy.spin(central_control_node)
    except KeyboardInterrupt:
        if rclpy.ok():
            central_control_node.get_logger().info('Keyboard interrupt received, shutting down node...')
    finally:
        if rclpy.ok():
            central_control_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()