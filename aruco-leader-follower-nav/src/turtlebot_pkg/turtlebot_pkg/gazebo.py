
"""
Gazebo TF Broadcaster for Follower Robot (tes_robolab)
-------------------------------------------------------
This ROS 2 node subscribes to the simulated ground truth pose of the follower TurtleBot in Gazebo
and publishes a TF2 transformation from the world frame ("warehouse") to the robot frame
("tes_robolab/turtlebot4"). This is used for simulated multi-robot tracking and navigation.

Main Features:
- Reads `/tes_robolab/sim_ground_truth_pose` (nav_msgs/Odometry)
- Publishes TF2 transform as `warehouse → tes_robolab/turtlebot4`
- Uses best-effort QoS settings for compatibility with simulation

Topics:
- Subscribed: `/tes_robolab/sim_ground_truth_pose`
- TF Published: `warehouse → tes_robolab/turtlebot4`

Authors: Armando Nicolella (LAM4R - University of Napoli Federico II), Pasquale Stingo
"""

from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class FramePublisher(Node):

    def __init__(self):
        # Initialize ROS node named 'tes_robo_turtlebot4'
        super().__init__('tes_robo_turtlebot4')

        # Create a TF2 broadcaster to publish transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Set QoS profile: keep last 10 messages, best-effort delivery
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,  
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        )

        # Subscribe to the simulated odometry topic for the follower TurtleBot
        self.subscription = self.create_subscription(
            Odometry,
            '/tes_robolab/sim_ground_truth_pose',
            self.handle_turtle_pose,
            qos_profile=qos_profile)

    def handle_turtle_pose(self, msg):
        # Create a TransformStamped message to represent the transform
        t = TransformStamped()

        # Fill in header and child frame information
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'warehouse'                     # Parent frame
        t.child_frame_id = 'tes_robolab/turtlebot4'         # Child frame

        # Fill in translation (position) from odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Fill in rotation (orientation) from odometry
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info('Follower position:\n{}'.format(t))

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
