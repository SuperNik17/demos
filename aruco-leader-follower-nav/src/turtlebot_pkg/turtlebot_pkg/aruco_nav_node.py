"""
ArUco Marker Pose Estimation Node
---------------------------------
This ROS 2 node subscribes to the RGB image and camera info topics from an Intel RealSense D435i camera,
detects ArUco markers using OpenCV, and publishes the estimated 6-DOF pose (position and orientation)
of the detected marker as a geometry_msgs/PoseStamped message.

Main Features:
- Works with ArUco 6x6 dictionary (250 markers)
- Converts rotation vectors to quaternions for ROS compatibility
- Publishes the pose of the first detected marker
- Compatible with TurtleBot 4 using RealSense D435i
"""

import rclpy
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion



class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Subscribe to the RGB image topic from the RealSense camera
        self.create_subscription(Image, '/D435_i/color/image_raw', self.image_callback, 10)
        # Subscribe to the camera intrinsic parameters
        self.create_subscription(CameraInfo, '/D435_i/color/camera_info', self.camera_info_callback, 10)


        # Publisher for the ArUco marker pose
        self.publisher_ = self.create_publisher(PoseStamped, 'aruco_pose', 10)

        # Timer to regularly publish the latest estimated pose
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)

        
  
        # Initialization for Camera intrinsic matrix and distortion coefficients
        self.intrinsics = np.array([])
        self.distortion_params = np.array([])
        
        # OpenCV bridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # ArUco dictionary and detection parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        

    def image_callback(self, msg):
         
        #self.get_logger().info('Received image')
        
        # Convert the ROS image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
 
        # Detect ArUco markers in the image      
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        
        
        if ids is not None:
            # Draw the detected markers
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Estimate pose of the first detected marker (only the first is used)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.094, self.intrinsics, self.distortion_params)
            #self.get_logger().info('Marker_tvecs: ' + str(tvecs))
            #self.get_logger().info('Marker_rvecs: ' + str(rvecs))

             
             
             # Draw the marker coordinate axes on the image
            for i in range(len(ids)):
                 
                cv_image = cv2.drawFrameAxes(cv_image, self.intrinsics, self.distortion_params, rvecs[i], tvecs[i], length=0.2)
            else:
                
                self.get_logger().info('No markers detected')
                

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvecs) 
            
            
            # Convert rotation matrix to quaternion (w, x, y, z)
            q = Quaternion(matrix=rotation_matrix)
            

          

            # Create the PoseStamped message
            self.pose = PoseStamped()
            self.pose.header.stamp = self.get_clock().now().to_msg()
            self.pose.header.frame_id = "D435_i_color_optical_frame"
            self.pose.pose.position.x = tvecs[0, 0, 0]
            self.pose.pose.position.y = tvecs[0, 0, 1]
            self.pose.pose.position.z = tvecs[0, 0, 2]
            self.pose.pose.orientation.x = q[1]
            self.pose.pose.orientation.y = q[2]
            self.pose.pose.orientation.z = q[3]
            self.pose.pose.orientation.w = q[0]  
    
            # Pubblica la posa nel sistema riferimemto camera
            #self.publisher_.publish(pose)
            #self.get_logger().info('Pose: ' + str(pose))
    
        # Show the processed image with ArUco overlays    
        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)
    

    def publish_pose(self):
        # Publish the pose only if it has been estimated
        if hasattr(self, 'pose') and self.pose is not None:
            self.publisher_.publish(self.pose)
            #self.get_logger().info('Pose: ' + str(self.pose))
        

    def camera_info_callback(self, msg):

        #self.get_logger().info('Received camera info')

        # Store the camera intrinsic matrix and distortion coefficients
        self.intrinsics = np.array(msg.k).reshape((3, 3))
        self.distortion_params = np.array(msg.d)  
        



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()